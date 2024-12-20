# imu_reader.py

import asyncio
import logging
import re
import datetime
from typing import Optional
from collections import deque
from async_serial import AsyncSerial  # 确保此路径正确并存在

# 设置IMUReader专用日志
logger = logging.getLogger("IMUReader")
logger.setLevel(logging.INFO)
handler = logging.StreamHandler()
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

class IMUReader:
    """
    IMUReader类：
    1. 异步从串口读取IMU数据帧
    2. 使用正则解析Yaw, Pitch, Roll
    3. 数据以"AT,yaw,pitch,roll\r\n"格式输出
    4. 打印到控制台并记录到文件
    5. 可设max_lines限制，达到后自动停止
    6. 添加数据有效性检查
    7. 添加初始化参数验证
    8. 添加数据缓冲功能（可选）
    """

    # 正则表达式匹配"AT,yaw,pitch,roll"格式
    IMU_PATTERN = re.compile(
        r"AT,([+-]?\d+(?:\.\d+)?),([+-]?\d+(?:\.\d+)?),([+-]?\d+(?:\.\d+)?)"
    )

    def __init__(
        self,
        port: str,
        baudrate: int = 460800,
        stopbits: int = 1,
        parity: str = 'N',
        output_file: str = "imu_data.txt",
        max_lines: Optional[int] = None,
        logger_instance: Optional[logging.Logger] = None,
        buffer_size: int = 100,
        auto_reset: bool = True,
        reset_delay: float = 0.1
    ):
        # 参数验证
        if not port:
            raise ValueError("必须指定串口")
        if baudrate <= 0:
            raise ValueError("波特率必须为正整数")
        if max_lines is not None and max_lines <= 0:
            raise ValueError("max_lines必须为正整数")
        if buffer_size <= 0:
            raise ValueError("buffer_size必须为正整数")

        self.port = port
        self.baudrate = baudrate
        self.stopbits = stopbits
        self.parity = parity
        self.output_file = output_file
        self.max_lines = max_lines
        self.line_count = 0
        self.running = True

        self.logger = logger_instance if logger_instance else logger

        self.serial = AsyncSerial(
            port=self.port,
            baudrate=self.baudrate,
            stopbits=self.stopbits,
            parity=self.parity,
            on_frame_received=self.on_frame_received,
            logger=self.logger,
            auto_reset=auto_reset,
            reset_delay=reset_delay
        )
        self.file = None

        # 初始化数据缓冲区
        self.data_buffer = deque(maxlen=buffer_size)

    async def connect(self):
        """
        连接IMU串口并初始化输出文件。
        """
        try:
            await self.serial.connect()
            self.file = open(self.output_file, "w", encoding="utf-8", buffering=1)
            self.file.write("Timestamp(s) Yaw Pitch Roll\n")
            self.logger.info(f"IMU数据将记录到文件: {self.output_file}")
        except Exception as e:
            self.logger.error(f"无法连接IMU串口 {self.port}: {e}")
            raise e

    def on_frame_received(self, frame: bytes):
        """
        串口数据接收回调函数
        """
        try:
            if not self.running:
                return

            line = frame.decode('utf-8', errors='replace').strip()
            self.logger.debug(f"接收到IMU数据：{line}")
            match = self.IMU_PATTERN.search(line)
            if match:
                yaw = float(match.group(1))
                pitch = float(match.group(2))
                roll = float(match.group(3))

                # 数据有效性检查
                if not (0 <= yaw < 360 and -90 <= pitch <= 90 and -180 <= roll <= 180):
                    self.logger.warning(f"数据超出有效范围: Yaw={yaw}, Pitch={pitch}, Roll={roll}")
                    return

                # 将Yaw从 [0, 360) 转换到 [-180, 180)
                if yaw >= 180:
                    yaw -= 360
                    self.logger.debug(f"转换后的Yaw: {yaw}")

                # 使用datetime.now().timestamp()统一时间戳
                timestamp = datetime.datetime.now().timestamp()

                if self.max_lines and self.line_count >= self.max_lines:
                    self.logger.info(f"已记录 {self.line_count} 行数据，达到最大行数限制，准备停止。")
                    asyncio.create_task(self.stop())
                    return

                print(f"Timestamp: {timestamp:.6f}s | Yaw: {yaw:.2f}° | Pitch: {pitch:.2f}° | Roll: {roll:.2f}°")

                if self.file:
                    self.file.write(f"{timestamp:.6f} {yaw:.2f} {pitch:.2f} {roll:.2f}\n")
                    self.line_count += 1

                self.data_buffer.append({
                    'timestamp': timestamp,
                    'yaw': yaw,
                    'pitch': pitch,
                    'roll': roll
                })
        except Exception as e:
            self.logger.error(f"处理IMU数据时发生错误：{e}")

    async def stop(self):
        if not self.running:
            return
        self.running = False
        await self.serial.close()
        if self.file:
            self.file.close()
            self.logger.info(f"文件 {self.output_file} 已关闭。")
        self.logger.info("IMUReader 已停止。")

    def get_buffer_data(self):
        return list(self.data_buffer)

# 测试main不动
async def main():
    imu_port = 'COM7'
    imu_baudrate = 460800
    imu_output_file = "imu_data_debug.txt"
    imu_max_lines = 2250

    try:
        imu_reader = IMUReader(
            port=imu_port,
            baudrate=imu_baudrate,
            output_file=imu_output_file,
            max_lines=imu_max_lines,
            buffer_size=45000,
            auto_reset=True,
            reset_delay=0.1
        )
    except ValueError as ve:
        logger.error(f"初始化IMUReader时发生错误：{ve}")
        return

    try:
        await imu_reader.connect()
        logger.info("开始独立调试 IMUReader（改为AT开头数据格式）...")
        while imu_reader.running:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        print("测试程序被用户中断。")
    except Exception as e:
        logger.error(f"IMUReader 调试时发生异常：{e}")
    finally:
        await imu_reader.stop()

if __name__ == "__main__":
    asyncio.run(main())
