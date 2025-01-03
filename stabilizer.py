import asyncio
import re
import math
import logging
import time
from dataclasses import dataclass
from collections import deque
from async_serial import AsyncSerial
from cyber_gear_motor import CyberGearMotor

# 设置日志
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# 正则表达式匹配 "AT,yaw,pitch,roll\r\n"
IMU_PATTERN = re.compile(r"AT,([+-]?\d+(?:\.\d+)?),([+-]?\d+(?:\.\d+)?),([+-]?\d+(?:\.\d+)?)\r?$")


@dataclass
class EulerAngles:
    yaw: float
    pitch: float
    roll: float


class OneEuroFilter:
    """
    简易的一欧元滤波器示例，用于实时平滑 IMU 数据。
    你可根据实际情况再进行改进，比如把频率 freq, min_cutoff, beta 等做成可调参。
    """
    def __init__(self, freq=20.0, min_cutoff=1.0, beta=0.02, dcutoff=1.0):
        """
        freq: 采样频率（与 update_stabilizer 保持一致或相近）
        min_cutoff: 最小截断频率（越大滤波越“强”，平滑度越高，但延迟也会增大）
        beta: 帮助控制噪声带来的影响，适当增大可以减小过冲
        dcutoff: 微分分量的截止频率
        """
        self.freq = freq
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.dcutoff = dcutoff

        self.x_prev = None
        self.dx_prev = 0.0
        self.last_time = None

    def alpha(self, cutoff):
        # 一欧元滤波器的 alpha 计算
        tau = 1.0 / (2.0 * math.pi * cutoff)
        te = 1.0 / self.freq
        return 1.0 / (1.0 + tau / te)

    def filter(self, x, timestamp=None):
        # 如果是首次滤波，直接初始化
        if self.x_prev is None:
            self.x_prev = x
            self.last_time = timestamp if timestamp else time.time()
            return x

        # 计算采样周期
        if timestamp and self.last_time:
            elapsed = timestamp - self.last_time
            if elapsed > 0:
                self.freq = 1.0 / elapsed  # 动态调整频率(可选)
            self.last_time = timestamp
        else:
            elapsed = 1.0 / self.freq  # 兜底

        # 估计当前导数 dx
        dx = (x - self.x_prev) * self.freq

        # 对导数做低通滤波
        ed_cutoff = self.dcutoff
        ed_alpha = self.alpha(ed_cutoff)
        dx_hat = ed_alpha * dx + (1 - ed_alpha) * self.dx_prev

        # 动态计算 cutoff
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = self.alpha(cutoff)

        # 最终输出
        x_hat = a * x + (1 - a) * self.x_prev

        # 更新状态
        self.x_prev = x_hat
        self.dx_prev = dx_hat
        return x_hat


class Stabilizer:
    """
    云台稳定器类，负责接收IMU数据并控制左右电机以实现稳定。
    引入一欧元滤波并缩短队列时间窗口，增强实时性和稳定性。
    """
    def __init__(self):
        # 初始化左右电机
        self.motor_left = CyberGearMotor(
            can_id=127,
            serial_port='COM5',
            master_id=0x00FD,
            position_request_interval=1.0/10.0# 电机 10hz更加当前位置信息
        )

        self.motor_right = CyberGearMotor(
            can_id=127,
            serial_port='COM4',
            master_id=0x00FD,
            position_request_interval=1.0/10.0 # 电机 10hz更加当前位置信息
        )

        # 初始化IMU串口
        self.imu_com = AsyncSerial(
            port='COM11',
            baudrate=921600,
            on_frame_received=self._on_frame_received,
            auto_reset=False
        )

        # 用于缓存 IMU 最近 0.3 秒的角度数据（yaw, pitch, roll），并保存对应的时间戳
        # 500Hz -> 150 帧左右; 这里 maxlen=200 即可
        self.imu_data_buffer = deque(maxlen=200)

        # 控制循环的运行状态
        self.running = False
        # 后台协程任务句柄
        self._stabilizer_task = None

        # 为 yaw, pitch, roll 各自建立一欧元滤波器实例
        # 如果你的 IMU 更新频率非常快，可设置初始化 freq=500; 但实际在 _update_stabilizer 里运行是 ~20Hz
        self.filter_yaw = OneEuroFilter(freq=20.0, min_cutoff=1.0, beta=0.02)
        self.filter_pitch = OneEuroFilter(freq=20.0, min_cutoff=1.0, beta=0.02)
        self.filter_roll = OneEuroFilter(freq=20.0, min_cutoff=1.0, beta=0.02)

    def _on_frame_received(self, frame: bytes):
        """
        处理接收到的IMU数据帧，将有效数据进行解析并缓存（仅缓存操作）。
        """
        match = IMU_PATTERN.match(frame.decode('utf-8', errors='replace').strip())
        if match:
            try:
                yaw, roll, pitch = map(float, match.groups())
                # 将数据和当前时间戳缓存
                self.imu_data_buffer.append((time.time(), yaw, pitch, roll))

                logger.debug(f"IMU缓存：yaw={yaw}, pitch={pitch}, roll={roll}, 缓存大小={len(self.imu_data_buffer)}")
            except ValueError:
                logger.error("Invalid IMU data format")
        else:
            logger.error("Invalid IMU data format")

    async def _update_stabilizer(self, update_frequency: float = 20.0):
        """
        后台协程：每秒 update_frequency 次，从 IMU 数据缓存中取出最近 0.3 秒数据，
        做简单平滑后，再用一欧元滤波进行二次滤波，最后控制电机。
        """
        interval = 1.0 / update_frequency
        while self.running:
            try:
                now = time.time()
                # 1) 移除超过 0.3 秒之前的数据
                while self.imu_data_buffer and (now - self.imu_data_buffer[0][0] > 0.3):
                    self.imu_data_buffer.popleft()

                if len(self.imu_data_buffer) == 0:
                    # 若无数据，跳过
                    await asyncio.sleep(interval)
                    continue

                # 2) 对剩余数据进行一次简单处理，可以取最新值，或平均值
                #   - 这里示例：取最近 0.1 秒内的数据做平均，以再次平滑噪声
                #   - 你也可以只取 self.imu_data_buffer[-1] 做最新值
                sub_data = []
                threshold_time = now - 0.1  # 取最近 0.1 秒
                for t, y, p, r in reversed(self.imu_data_buffer):
                    if t >= threshold_time:
                        sub_data.append((y, p, r))
                    else:
                        break

                if not sub_data:
                    # 如果 0.1 秒内没有数据，则直接取最后一帧
                    latest_timestamp, latest_yaw, latest_pitch, latest_roll = self.imu_data_buffer[-1]
                else:
                    # 计算平均值
                    avg_yaw = sum(d[0] for d in sub_data) / len(sub_data)
                    avg_pitch = sum(d[1] for d in sub_data) / len(sub_data)
                    avg_roll = sum(d[2] for d in sub_data) / len(sub_data)
                    latest_timestamp = now
                    latest_yaw = avg_yaw
                    latest_pitch = avg_pitch
                    latest_roll = avg_roll

                # 3) 进行一欧元滤波
                #    这里将平均后的值再传入滤波器
                yaw_filtered = self.filter_yaw.filter(latest_yaw, latest_timestamp)
                pitch_filtered = self.filter_pitch.filter(latest_pitch, latest_timestamp)
                roll_filtered = self.filter_roll.filter(latest_roll, latest_timestamp)

                # 4) 调用稳定控制
                await self._stabilize(yaw_filtered, pitch_filtered, roll_filtered)

            except Exception as e:
                logger.error(f"后台稳定控制时发生错误: {e}")

            # 5) 休眠 interval
            await asyncio.sleep(interval)

    async def _stabilize(self, yaw: float, pitch: float, roll: float):
        """
        根据接收到的欧拉角进行稳定控制。
        保留原有的角度到弧度转换、电机控制逻辑，并可根据需要微调p值。
        """
        try:
            yaw_compensation = -yaw
            pitch_compensation = -pitch
            roll_compensation = -roll

            # 运动角度阈值，避免小抖动反复纠偏
            if abs(pitch_compensation) < 0.5 and abs(roll_compensation) < 0.5:
                return

            left_motor_radian, right_motor_radian = self._calculate_motor_angles(
                yaw_compensation, pitch_compensation, roll_compensation
            )

            # 在此可适当加点 P 值，或简单倍乘抑制过冲
            # left_motor_radian *= 0.8
            # right_motor_radian *= 0.8

            await asyncio.gather(
                self.motor_left.set_position(left_motor_radian, 2.0),
                self.motor_right.set_position(right_motor_radian, 2.0)
            )

            logger.info(f"[Stabilize] yaw={yaw:.2f}, pitch={pitch:.2f}, roll={roll:.2f}, "
                        f"L={left_motor_radian:.3f}rad, R={right_motor_radian:.3f}rad")
        except Exception as e:
            logger.error(f"稳定控制时发生错误: {e}")

    def to_radians(self, degrees: float) -> float:
        """
        将角度转换为弧度，并限制在[-0.7, 0.7]范围内。
        """
        radians = degrees * math.pi / 180.0
        if radians > 0.7:
            return 0.7
        elif radians < -0.7:
            return -0.7
        return radians

    def _calculate_motor_angles(self, yaw: float, pitch: float, roll: float):
        """
        根据补偿的欧拉角计算左右电机的转动角度（单位：度），再转成弧度。
        保留你的逻辑，只是示例中注释了 /2，具体看你的云台结构。
        """
        L_roll = -roll
        R_roll = -roll
        L_pitch = -pitch
        R_pitch = pitch
        left_motor_angle = L_roll + L_pitch
        right_motor_angle = R_roll + R_pitch

        logger.debug(f"[CalcMotorAngles] Before radians: L_angle={left_motor_angle:.2f}°, R_angle={right_motor_angle:.2f}°")

        left_motor_radian = self.to_radians(left_motor_angle)
        right_motor_radian = self.to_radians(right_motor_angle)

        logger.debug(f"[CalcMotorAngles] After radians: L={left_motor_radian:.3f}, R={right_motor_radian:.3f}")
        return left_motor_radian, right_motor_radian

    async def start(self):
        """
        启动稳定器，连接电机并初始化IMU串口，并启动后台稳定控制协程。
        """
        try:
            self.running = True
            logger.info("启动稳定器...")

            # 并发连接左右电机
            await asyncio.gather(
                self.motor_left.connect(),
                self.motor_right.connect()
            )
            logger.info("已连接左右电机。")

            # 重置电机旋转
            await asyncio.gather(
                self.motor_left.reset_rotation(),
                self.motor_right.reset_rotation()
            )
            logger.info("已重置电机旋转。")

            # 设置电机零位
            await asyncio.gather(
                self.motor_left.set_zero_position(),
                self.motor_right.set_zero_position()
            )
            logger.info("已设置电机零位。")

            # 连接IMU串口
            await self.imu_com.connect()
            logger.info("已连接IMU串口。")

            # 手动设置IMU串口的DTR和RTS为False
            if self.imu_com.transport and hasattr(self.imu_com.transport, 'serial') and self.imu_com.transport.serial:
                self.imu_com.transport.serial.dtr = False
                self.imu_com.transport.serial.rts = False
                logger.info(f"IMU串口 DTR={self.imu_com.transport.serial.dtr}, RTS={self.imu_com.transport.serial.rts}")

            # 启动后台稳定控制协程，比如 20 Hz
            self._stabilizer_task = asyncio.create_task(self._update_stabilizer(update_frequency=20.0))

            logger.info("稳定器已启动并开始处理IMU数据。")

        except Exception as e:
            logger.error(f"启动稳定器时发生错误: {e}")
            await self.stop()

    async def stop(self):
        """
        停止稳定器，关闭电机和串口连接，并取消后台控制协程。
        """
        if not self.running:
            return

        self.running = False
        logger.info("正在停止稳定器...")

        # 取消后台协程任务
        if self._stabilizer_task:
            self._stabilizer_task.cancel()
            try:
                await self._stabilizer_task
            except asyncio.CancelledError:
                logger.info("后台稳定控制协程已取消。")

        # 关闭IMU串口
        try:
            await self.imu_com.close()
            logger.info("已关闭IMU串口。")
        except Exception as e:
            logger.error(f"关闭IMU串口时发生错误: {e}")

        # 停止电机
        try:
            await asyncio.gather(
                self.motor_left.stop_motor(),
                self.motor_right.stop_motor()
            )
            logger.info("已停止所有电机。")
        except Exception as e:
            logger.error(f"停止电机时发生错误: {e}")

        # 关闭电机连接
        try:
            await asyncio.gather(
                self.motor_left.close(),
                self.motor_right.close()
            )
            logger.info("已关闭所有电机连接。")
        except Exception as e:
            logger.error(f"关闭电机连接时发生错误: {e}")

        logger.info("稳定器已停止。")


# ----------------- 测试 main 函数 ------------------ #
async def main():
    """
    测试稳定器的主函数，启动稳定器，运行直到用户中断。
    """
    stabilizer = Stabilizer()
    await stabilizer.start()

    try:
        # 保持运行，直到用户中断（例如按下 Ctrl+C）
        await asyncio.Event().wait()
    except asyncio.CancelledError:
        logger.info("主任务被取消。")
    except KeyboardInterrupt:
        logger.info("检测到中断信号，准备停止稳定器。")
    finally:
        await stabilizer.stop()

if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("程序已中断。")
    except Exception as ex:
        logger.error(f"程序发生错误: {ex}")
