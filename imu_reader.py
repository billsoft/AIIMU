import asyncio
import logging
import re
import datetime
from typing import Optional
from collections import deque

# 这里根据你的文件名改动, 如:
from async_serial import AsyncSerial

logger = logging.getLogger("IMUReader")
logger.setLevel(logging.INFO)
handler = logging.StreamHandler()
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

class IMUReader:
    """
    IMUReader:
    1. 异步串口读"AT,yaw,pitch,roll\r\n"数据，回调中快速写入二级队列
    2. 后台协程批量取出队列，解析并写文件，以减少串口回调耗时
    3. data_buffer 中存储 dict: {'timestamp':..., 'yaw':..., 'pitch':..., 'roll':...}
    4. 达到 max_lines 后会自动 stop()
    """

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
        reset_delay: float = 0.2
    ):
        if not port:
            raise ValueError("必须指定port")
        if baudrate <= 0:
            raise ValueError("baudrate必须>0")
        if max_lines is not None and max_lines <= 0:
            raise ValueError("max_lines必须>0")
        if buffer_size <= 0:
            raise ValueError("buffer_size必须>0")

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
            reset_delay=reset_delay,
            use_watchdog=False,
            watchdog_interval=5.0,
            no_data_timeout=15.0
        )

        self.data_buffer = deque(maxlen=buffer_size)
        self._frame_queue: asyncio.Queue = asyncio.Queue(maxsize=buffer_size * 2)
        self._processing_task: Optional[asyncio.Task] = None
        self.file = None

    async def connect(self):
        """连接串口、打开文件、启动后台处理协程。"""
        await self.serial.connect()
        self.file = open(self.output_file, "w", encoding="utf-8", buffering=8*1024)
        self.file.write("Timestamp(s) Yaw Pitch Roll\n")
        self.logger.info(f"IMU数据记录到文件: {self.output_file}")

        self._processing_task = asyncio.create_task(self._process_loop())

    def on_frame_received(self, frame: bytes):
        if not self.running:
            return
        try:
            self._frame_queue.put_nowait(frame)
        except asyncio.QueueFull:
            self.logger.warning("IMUReader队列已满，丢弃帧")
        except Exception as e:
            self.logger.error(f"on_frame_received异常: {e}")

    async def _process_loop(self):
        self.logger.info("IMUReader 后台处理协程已启动")
        try:
            while self.running:
                frame = await self._frame_queue.get()
                self._handle_frame(frame)

                i = 0
                while not self._frame_queue.empty() and i < 50:
                    f = self._frame_queue.get_nowait()
                    self._handle_frame(f)
                    i += 1

                await asyncio.sleep(0)

        except asyncio.CancelledError:
            self.logger.info("IMUReader 后台协程被取消")
        except Exception as e:
            self.logger.error(f"IMUReader 后台协程异常: {e}")
        finally:
            self.logger.info("IMUReader 后台协程结束")

    def _handle_frame(self, frame: bytes):
        line = frame.decode('utf-8', errors='replace').strip()
        match = self.IMU_PATTERN.search(line)
        if not match:
            return

        try:
            yaw = float(match.group(1))
            pitch = float(match.group(2))
            roll = float(match.group(3))

            if not (0 <= yaw < 360 and -90 <= pitch <= 90 and -180 <= roll <= 180):
                return
            if yaw >= 180:
                yaw -= 360

            timestamp = datetime.datetime.now().timestamp()
            if self.max_lines and self.line_count >= self.max_lines:
                asyncio.create_task(self.stop())
                return

            print(f"IMU: ts={timestamp:.3f}, Yaw={yaw:.2f}, Pitch={pitch:.2f}, Roll={roll:.2f}")
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
            self.logger.error(f"_handle_frame异常: {e}")

    async def stop(self):
        """停止后台处理"""
        if not self.running:
            return
        self.running = False

        if self._processing_task:
            try:
                await self._processing_task
            except asyncio.CancelledError:
                pass
            self._processing_task = None

        await self.serial.close()
        if self.file:
            self.file.close()
            self.logger.info(f"文件 {self.output_file} 已关闭")
        self.logger.info("IMUReader 已停止")

    def get_buffer_data(self):
        return list(self.data_buffer)


async def main():
    imu_port = 'COM7'
    imu_baudrate = 230400
    imu_output_file = "imu_data_debug.txt"
    imu_max_lines = 50000

    imu_reader = IMUReader(
        port=imu_port,
        baudrate=imu_baudrate,
        output_file=imu_output_file,
        max_lines=imu_max_lines,
        buffer_size=10000*10,
        auto_reset=True,
        reset_delay=0.2
    )

    try:
        await imu_reader.connect()
        logger.info("开始独立调试 IMUReader...")
        while imu_reader.running:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        logger.warning("用户中断")
    except Exception as e:
        logger.error(f"IMUReader 调试时发生异常：{e}")
    finally:
        await imu_reader.stop()


if __name__ == "__main__":
    asyncio.run(main())
