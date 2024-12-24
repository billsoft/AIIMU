import asyncio
import logging
import re
import datetime
from typing import Optional
from collections import deque

# 这里请注意 import 的文件名，如果 async_serial.py 放在同文件夹，需要改为 from async_serial import AsyncSerial
# 我这里示范写法：
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

        # 创建串口实例
        self.serial = AsyncSerial(
            port=self.port,
            baudrate=self.baudrate,
            stopbits=self.stopbits,
            parity=self.parity,
            on_frame_received=self.on_frame_received,
            logger=self.logger,
            auto_reset=auto_reset,   # 修改点：仅在最初连接一次复位，不会自动多次复位
            reset_delay=reset_delay,
            use_watchdog=False,      # 修改点：暂时关掉或禁用 watchdog，防止二次复位
            watchdog_interval=5.0,
            no_data_timeout=15.0
        )

        # 1级缓存（对外 get_buffer_data() 使用）
        self.data_buffer = deque(maxlen=buffer_size)
        # 2级队列（收帧回调只做 put_nowait）
        self._frame_queue: asyncio.Queue = asyncio.Queue(maxsize=buffer_size * 2)

        self._processing_task: Optional[asyncio.Task] = None
        self.file = None

    async def connect(self):
        """连接串口、打开文件、启动后台处理协程。"""
        await self.serial.connect()
        self.file = open(self.output_file, "w", encoding="utf-8", buffering=8*1024)
        self.file.write("Timestamp(s) Yaw Pitch Roll\n")
        self.logger.info(f"IMU数据记录到文件: {self.output_file}")

        # 启动后台协程
        self._processing_task = asyncio.create_task(self._process_loop())

    def on_frame_received(self, frame: bytes):
        """串口回调：把帧扔到 _frame_queue，减轻回调耗时。"""
        if not self.running:
            return
        try:
            self._frame_queue.put_nowait(frame)
        except asyncio.QueueFull:
            self.logger.warning("IMUReader队列已满，丢弃帧")
        except Exception as e:
            self.logger.error(f"on_frame_received异常: {e}")

    async def _process_loop(self):
        """后台协程：批量从队列取帧->解析->写文件->更新data_buffer"""
        self.logger.info("IMUReader 后台处理协程已启动")
        try:
            while self.running:
                # 先阻塞等1帧
                frame = await self._frame_queue.get()
                self._handle_frame(frame)

                # 一次最多再额外拿 50 帧
                i = 0
                while not self._frame_queue.empty() and i < 50:
                    try:
                        f = self._frame_queue.get_nowait()
                        self._handle_frame(f)
                        i += 1
                    except asyncio.QueueEmpty:
                        break

                await asyncio.sleep(0)  # 让出事件循环

        except asyncio.CancelledError:
            self.logger.info("IMUReader 后台协程被取消")
        except Exception as e:
            self.logger.error(f"IMUReader 后台协程异常: {e}")
        finally:
            self.logger.info("IMUReader 后台协程结束")

    def _handle_frame(self, frame: bytes):
        """解析并记录到文件 / data_buffer"""
        line = frame.decode('utf-8', errors='replace').strip()
        match = self.IMU_PATTERN.search(line)
        if not match:
            return

        try:
            yaw = float(match.group(1))
            pitch = float(match.group(2))
            roll = float(match.group(3))

            # 有效性检查
            if not (0 <= yaw < 360 and -90 <= pitch <= 90 and -180 <= roll <= 180):
                return
            # yaw [0,360)->[-180,180)
            if yaw >= 180:
                yaw -= 360

            # 时间戳
            timestamp = datetime.datetime.now().timestamp()

            # 达到上限后 stop()
            if self.max_lines and self.line_count >= self.max_lines:
                asyncio.create_task(self.stop())
                return

            # 记录
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
        """停止后台处理，让后台协程先处理完队列后再真正close串口、文件。"""
        if not self.running:
            return
        self.running = False

        # 不直接cancel后台协程，给它时间drain队列
        if self._processing_task:
            try:
                await self._processing_task  # 等它自己退出
            except asyncio.CancelledError:
                pass
            self._processing_task = None

        await self.serial.close()
        if self.file:
            self.file.close()
            self.logger.info(f"文件 {self.output_file} 已关闭")
        self.logger.info("IMUReader 已停止")

    def get_buffer_data(self):
        """对外提供 IMU 数据的接口。"""
        return list(self.data_buffer)


# 测试main函数保持不变
async def main():
    """
    独立测试IMUReader的main函数:
    1. 创建IMUReader实例并连接IMU串口
    2. 记录数据，直到max_lines或用户中断
    """
    imu_port = 'COM7'  # 根据实际情况修改
    imu_baudrate = 230400
    imu_output_file = "imu_data_debug.txt"
    imu_max_lines = 50000

    try:
        imu_reader = IMUReader(
            port=imu_port,
            baudrate=imu_baudrate,
            output_file=imu_output_file,
            max_lines=imu_max_lines,
            buffer_size=10000*10,
            auto_reset=True,     # 只在connect时自动复位一次
            reset_delay=0.2
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
