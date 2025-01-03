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

class Stabilizer:
    """
    云台稳定器类，负责接收IMU数据并控制左右电机以实现稳定。
    现改进逻辑：将IMU数据与电机控制解耦，通过一个固定频率的协程进行电机控制。
    """
    def __init__(self):
        # 初始化左右电机
        self.motor_left = CyberGearMotor(
            can_id=127,
            serial_port='COM5',
            master_id=0x00FD,
            position_request_interval=1
        )

        self.motor_right = CyberGearMotor(
            can_id=127,
            serial_port='COM4',
            master_id=0x00FD,
            position_request_interval=1
        )

        # 初始化IMU串口
        self.imu_com = AsyncSerial(
            port='COM11',
            baudrate=921600,
            on_frame_received=self._on_frame_received,
            auto_reset=False  # 禁用自动复位，以便后续手动设置DTR和RTS
        )

        # 用于缓存 IMU 最近 1 秒的角度数据（yaw, pitch, roll），并保存对应的时间戳
        # 注意：maxlen 可以适当设置大一点防止过快写入
        self.imu_data_buffer = deque(maxlen=1000)  # 每秒500帧的话，最多保留一两秒的数据都可以

        # 用于控制循环的运行状态
        self.running = False

        # 后台协程任务句柄（用于控制更新电机位置的任务）
        self._stabilizer_task = None

    def _on_frame_received(self, frame: bytes):
        """
        处理接收到的IMU数据帧，将有效数据进行解析并缓存。
        之前的逻辑是直接调用 motor set_position，这里做了解耦和优化，避免过度频繁地设定位置。
        """
        match = IMU_PATTERN.match(frame.decode('utf-8', errors='replace').strip())
        if match:
            try:
                # 解析并转换成 float
                yaw, roll, pitch = map(float, match.groups())
                # 将数据和当前时间戳缓存
                self.imu_data_buffer.append((time.time(), yaw, pitch, roll))

                logger.debug(f"IMU缓存：yaw={yaw}, pitch={pitch}, roll={roll}, 当前缓存大小={len(self.imu_data_buffer)}")
            except ValueError:
                logger.error("Invalid IMU data format")
        else:
            # 如果无法匹配正则
            logger.error("Invalid IMU data format")

    async def _update_stabilizer(self, update_frequency: float = 10.0):
        """
        后台协程：每秒 update_frequency 次（默认10Hz），从 IMU 数据缓存中取出最近 1 秒钟的数据，
        在此基础上做去抖/滤波处理后，再调用 _stabilize 方法控制电机。
        """
        interval = 1.0 / update_frequency
        while self.running:
            try:
                # 1) 获取当前时间
                now = time.time()
                # 2) 剔除超过 1 秒的历史数据（此处可以按照实际需求调整窗口大小）
                while self.imu_data_buffer and (now - self.imu_data_buffer[0][0] > 1.0):
                    self.imu_data_buffer.popleft()

                if len(self.imu_data_buffer) == 0:
                    # 没有数据就等待下一次
                    await asyncio.sleep(interval)
                    continue

                # 3) 对最近 1 秒内的数据进行处理，例如取最新值，或者做平均/中位数滤波
                # 这里示例：取**最新**一帧数据（也可取平均）
                latest_timestamp, latest_yaw, latest_pitch, latest_roll = self.imu_data_buffer[-1]

                # 4) 调用稳定控制
                await self._stabilize(latest_yaw, latest_pitch, latest_roll)

            except Exception as e:
                logger.error(f"后台稳定控制时发生错误: {e}")

            # 5) 休眠 interval 时间，下一次循环
            await asyncio.sleep(interval)

    async def _stabilize(self, yaw: float, pitch: float, roll: float):
        """
        根据接收到的欧拉角进行稳定控制。
        保留原有的角度到弧度和电机控制逻辑。
        """
        try:
            # 计算补偿角度
            yaw_compensation = -yaw
            pitch_compensation = -pitch
            roll_compensation = -roll

            # 运动角度阈值：大于这个阈值云台才纠偏，增加惰性放置抖动
            if abs(pitch_compensation) < 1.0 and abs(roll_compensation) < 1.0:
                return

            # 计算左右电机的转动角度（弧度）
            left_motor_radian, right_motor_radian = self._calculate_motor_angles(
                yaw_compensation, pitch_compensation, roll_compensation
            )

            # p控制
            left_motor_radian *= 0.5
            right_motor_radian *= 0.5

            # 设置电机位置 (异步并发执行)
            await asyncio.gather(
                self.motor_left.set_position(left_motor_radian, 5.0),
                self.motor_right.set_position(right_motor_radian, 5.0)
            )
            await asyncio.sleep(0.1)

            logger.info(f"设置电机位置: 左={left_motor_radian:.3f} rad, 右={right_motor_radian:.3f} rad")
        except Exception as e:
            logger.error(f"稳定控制时发生错误: {e}")

    def to_radians(self, degrees: float) -> float:
        """
        将角度转换为弧度，并限制在[-0.7, 0.7]范围内。

        参数：
            degrees (float): 角度值。

        返回：
            float: 限制在[-0.7, 0.7]范围内的弧度值。
        """
        radians = degrees * math.pi / 180.0
        # 限制弧度值范围
        if radians > 0.7:
            return 0.7
        elif radians < -0.7:
            return -0.7
        return radians

    def _calculate_motor_angles(self, yaw: float, pitch: float, roll: float):
        """
        根据补偿的欧拉角计算左右电机的转动角度。

        计算公式基于以下关系：
            roll_final  = -(L + R)
            pitch_final = (L - R) / 2

        由此推导出：
           left_motor_angle  = pitch - (roll / 2)
           right_motor_angle = -pitch - (roll / 2)

        因实际项目结构的需求，暂做如下方式计算（你原先的自定义方案）：
        """
        # 这里保留你自定义的计算方法
        L_roll = -roll #/ 2.0
        R_roll = -roll #/ 2.0
        L_pitch= -pitch
        R_pitch= pitch
        left_motor_angle = L_roll + L_pitch
        right_motor_angle = R_roll + R_pitch

        logger.debug(f"计算电机角度前: 左角度={left_motor_angle:.2f}°, 右角度={right_motor_angle:.2f}°")
        left_motor_radian = self.to_radians(left_motor_angle)
        right_motor_radian = self.to_radians(right_motor_angle)
        logger.debug(f"计算电机角度后: 左弧度={left_motor_radian:.3f}, 右弧度={right_motor_radian:.3f}")

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

            # 启动后台稳定控制协程，例如默认 10 Hz
            self._stabilizer_task = asyncio.create_task(self._update_stabilizer(update_frequency=10.0))

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

# ----------------- 测试main函数 ------------------ #
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
