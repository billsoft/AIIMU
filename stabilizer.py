import asyncio
import re
import math
import logging
from dataclasses import dataclass
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
    roll: float  # 添加 roll 属性

class Stabilizer:
    """
    云台稳定器类，负责接收IMU数据并控制左右电机以实现稳定。
    """
    def __init__(self):
        # 初始化左右电机
        self.motor_left = CyberGearMotor(
            can_id=127,
            serial_port='COM5',
            master_id=0x00FD,
            position_request_interval=1 / 50  # 设置为50Hz
        )

        self.motor_right = CyberGearMotor(
            can_id=127,
            serial_port='COM4',
            master_id=0x00FD,
            position_request_interval=1 / 50  # 设置为50Hz
        )

        # 初始化IMU串口
        self.imu_com = AsyncSerial(
            port='COM11',
            baudrate=921600,
            on_frame_received=self._on_frame_received,
            auto_reset=False  # 禁用自动复位，以便后续手动设置DTR和RTS
        )

        # 控制任务和运行状态
        self.running = False

    def _on_frame_received(self, frame: bytes):
        """
        处理接收到的IMU数据帧，将有效数据进行解析并调用稳定控制。
        """
        match = IMU_PATTERN.match(frame.decode('utf-8', errors='replace').strip())
        if match:
            try:
                yaw, pitch, roll = map(float, match.groups())
                logger.info(f"Received IMU data: yaw={yaw}, pitch={pitch}, roll={roll}")
                # 创建一个任务来异步执行稳定控制
                asyncio.create_task(self._stabilize(yaw, pitch, roll))
            except ValueError:
                logger.error("Invalid IMU data format")
        else:
            logger.error("Invalid IMU data format")

    async def _stabilize(self, yaw: float, pitch: float, roll: float):
        """
        根据接收到的欧拉角进行稳定控制。
        """
        try:
            # 计算补偿角度
            yaw_compensation = -yaw
            pitch_compensation = -pitch
            roll_compensation = -roll

            # 计算左右电机的转动角度（弧度）
            left_motor_radian, right_motor_radian = self._calculate_motor_angles(
                yaw_compensation, pitch_compensation, roll_compensation
            )

            # # 增大位置命令的幅度（可根据实际需求调整）
            # left_motor_radian *= 2
            # right_motor_radian *= 2

            # 设置电机位置
            await asyncio.gather(
                self.motor_left.set_position(left_motor_radian),
                self.motor_right.set_position(right_motor_radian)
            )
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
           left_motor_angle = roll - (pitch / 2)
           right_motor_angle = -roll - (pitch / 2)
        """
        # 根据补偿角度计算电机角度（单位：度）
        left_motor_angle = roll * -1 - (pitch / 2)
        right_motor_angle = -roll * -1 - (pitch / 2)

        logger.debug(f"计算电机角度前: 左角度={left_motor_angle:.2f}°, 右角度={right_motor_angle:.2f}°")
        left_motor_radian = self.to_radians(left_motor_angle)
        right_motor_radian = self.to_radians(right_motor_angle)
        logger.debug(f"计算电机角度后: 左弧度={left_motor_radian:.3f}, 右弧度={right_motor_radian:.3f}")

        return left_motor_radian, right_motor_radian

    async def start(self):
        """
        启动稳定器，连接电机并初始化IMU串口。
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

            # 设置电机控制模式为位置控制模式（假设 CyberGearMotor 提供 set_control_mode 方法）
            try:
                await asyncio.gather(
                    self.motor_left.set_control_mode('position'),
                    self.motor_right.set_control_mode('position')
                )
                logger.info("已设置电机控制模式为位置控制模式。")
            except AttributeError:
                logger.warning("CyberGearMotor 类没有 set_control_mode 方法，跳过设置控制模式。")
            except Exception as e:
                logger.error(f"设置电机控制模式时发生错误: {e}")

            # 连接IMU串口
            await self.imu_com.connect()
            logger.info("已连接IMU串口。")

            # 手动设置IMU串口的DTR和RTS为False
            if self.imu_com.transport and hasattr(self.imu_com.transport, 'serial') and self.imu_com.transport.serial:
                self.imu_com.transport.serial.dtr = False
                self.imu_com.transport.serial.rts = False
                logger.info(f"IMU串口 DTR={self.imu_com.transport.serial.dtr}, RTS={self.imu_com.transport.serial.rts}")

            logger.info("稳定器已启动并开始处理IMU数据。")

        except Exception as e:
            logger.error(f"启动稳定器时发生错误: {e}")
            await self.stop()

    async def stop(self):
        """
        停止稳定器，关闭电机和串口连接。
        """
        if not self.running:
            return

        self.running = False
        logger.info("正在停止稳定器...")

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
