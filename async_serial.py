import asyncio
import serial_asyncio
import logging
import time
from typing import Optional, Callable

class AsyncSerial(asyncio.Protocol):
    """
    异步串口通信类，支持自动复位功能（仅使用DTR）并可选看门狗机制来监控长时间无数据。
    修改点：
    1. 去掉 watchdog 中对 reset_device() 的调用，避免多次复位。
    2. 在 reset_device() 中只在 connect() 时调用一次，且最后保持 DTR=True。
    3. 优化 _process_buffer()，不盲目 clear()。
    """

    def __init__(
            self,
            port: str,
            baudrate: int = 115200,
            stopbits: int = 1,
            parity: str = 'N',
            on_frame_received: Optional[Callable[[bytes], None]] = None,
            logger: Optional[logging.Logger] = None,

            # 复位相关
            auto_reset: bool = False,  # 是否在 connect() 后自动通过DTR复位
            reset_delay: float = 0.2,  # 修改点：稍微加大复位脉冲时间

            # 看门狗相关
            use_watchdog: bool = False,
            watchdog_interval: float = 5.0,
            no_data_timeout: float = 15.0
    ):
        self.port = port
        self.baudrate = baudrate
        self.stopbits = stopbits
        self.parity = parity

        self.auto_reset = auto_reset
        self.reset_delay = reset_delay

        self.use_watchdog = use_watchdog
        self.watchdog_interval = watchdog_interval
        self.no_data_timeout = no_data_timeout

        self.on_frame_received = on_frame_received
        self._buffer = bytearray()
        self._connected = False
        self.transport: Optional[asyncio.Transport] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None

        # 日志
        if logger is None:
            logger = logging.getLogger("AsyncSerial")
            logger.setLevel(logging.DEBUG)
            if not logger.handlers:
                ch = logging.StreamHandler()
                ch.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
                logger.addHandler(ch)
        self.logger = logger

        self._frame_timeout = 0.1
        self._last_byte_time = 0.0
        self._watchdog_task: Optional[asyncio.Task] = None

    async def connect(self):
        """异步连接串口并根据需要执行DTR复位。"""
        try:
            self._loop = asyncio.get_event_loop()
            self.logger.info(f"[AsyncSerial] 连接 {self.port}@{self.baudrate}, auto_reset={self.auto_reset}")
            self.transport, _ = await serial_asyncio.create_serial_connection(
                loop=self._loop,
                protocol_factory=lambda: self,
                url=self.port,
                baudrate=self.baudrate,
                stopbits=self.stopbits,
                parity=self.parity
            )
            self._connected = True
            self.logger.info(f"[AsyncSerial] 串口 {self.port} 连接成功")

            # 自动复位（只在连接后第一次做一次）
            if self.auto_reset:
                await self.reset_device()
            else:
                # 普通设备：只置DTR=True保持稳定（有些板子需要）
                if hasattr(self.transport, 'serial') and self.transport.serial:
                    try:
                        self.transport.serial.dtr = True
                    except Exception as e:
                        self.logger.warning(f"[AsyncSerial] 设置DTR=True异常: {e}")

            # 启动看门狗
            if self.use_watchdog:
                self._watchdog_task = asyncio.create_task(self._watchdog_loop())

        except Exception as e:
            self.logger.error(f"[AsyncSerial] 串口连接失败: {e}")
            raise

    async def reset_device(self):
        """使用DTR复位，RTS在复位时暂时拉低。只在connect()时调用一次。"""
        if not (self.transport and hasattr(self.transport, 'serial') and self.transport.serial):
            self.logger.warning("[AsyncSerial] 无法访问底层serial进行DTR复位")
            return

        ser = self.transport.serial
        try:
            self.logger.info("[AsyncSerial] 开始DTR复位序列")
            orig_dtr = ser.dtr
            orig_rts = ser.rts

            # Step1: DTR=False, RTS=False
            ser.dtr = False
            ser.rts = False
            await asyncio.sleep(self.reset_delay)

            # Step2: DTR=True
            ser.dtr = True
            await asyncio.sleep(self.reset_delay)

            # Step3: DTR=False
            ser.dtr = False
            await asyncio.sleep(self.reset_delay)

            # 最后保持 DTR=True，避免板子一直复位
            ser.dtr = True
            ser.rts = orig_rts  # RTS 可恢复

            self.logger.info("[AsyncSerial] 发送DTR复位脉冲完成, 等待2秒重启")
            await asyncio.sleep(2.0)

        except Exception as e:
            self.logger.error(f"[AsyncSerial] reset_device异常: {e}")

    async def close(self):
        """关闭串口"""
        if not self._connected:
            self.logger.warning(f"[AsyncSerial] {self.port} 未连接或已关闭")
            return
        self.logger.info(f"[AsyncSerial] 准备关闭 {self.port}...")

        if self._watchdog_task:
            self._watchdog_task.cancel()
            try:
                await self._watchdog_task
            except asyncio.CancelledError:
                pass
            self._watchdog_task = None

        if self.transport:
            try:
                await asyncio.sleep(0.2)
                self.transport.close()
                self.logger.info(f"[AsyncSerial] 串口 {self.port} 已关闭")
            except Exception as e:
                self.logger.error(f"[AsyncSerial] 关闭串口异常: {e}")
        self._connected = False
        self.transport = None
        self._loop = None
        self._buffer.clear()

    def connection_made(self, transport):
        """连接建立时回调"""
        self.transport = transport
        self._connected = True
        self._last_byte_time = time.time()
        if hasattr(transport, 'serial') and transport.serial:
            self.logger.info(f"[AsyncSerial] {self.port} 已连接: DTR={transport.serial.dtr}, RTS={transport.serial.rts}")
            try:
                transport.serial.reset_input_buffer()
                transport.serial.reset_output_buffer()
            except Exception as e:
                self.logger.warning(f"[AsyncSerial] reset缓冲异常: {e}")
        else:
            self.logger.warning(f"[AsyncSerial] {self.port} 已连接, 但无 serial 属性")

    def data_received(self, data: bytes):
        """数据回调"""
        now = time.time()
        # 若超过一定时间才来一批字节，则认为是新的帧
        if (now - self._last_byte_time) > self._frame_timeout and len(self._buffer) > 0:
            self._buffer.clear()  # 可以适度清理，也可以保留部分
        self._last_byte_time = now

        self._buffer.extend(data)
        self._process_buffer()

    def _process_buffer(self):
        """
        查找 'AT'...'\r\n' 帧。
        修改点：如果找不到 'AT'，不直接清空全部 buffer，
        而是仅丢弃最前面无用的数据，以防止偶尔的分包丢失。
        """
        while True:
            start = self._buffer.find(b'AT')
            if start == -1:
                # 若未找到“AT”，丢弃前面一部分，但保留一些，防止漏掉分包。
                if len(self._buffer) > 10:
                    # 例如仅保留末尾 10 个字节
                    self._buffer = self._buffer[-10:]
                break

            end = self._buffer.find(b'\r\n', start)
            if end == -1:
                # 找到 'AT' 但没有找到结尾，则先不动
                break

            frame = self._buffer[start:end+2]
            del self._buffer[:end+2]

            if self.on_frame_received and callable(self.on_frame_received):
                try:
                    self.on_frame_received(frame)
                except Exception as e:
                    self.logger.error(f"[AsyncSerial] on_frame_received异常: {e}")
            else:
                self.logger.warning("[AsyncSerial] 未设置 on_frame_received")

    async def send_frame(self, frame: bytes):
        """异步发送"""
        if self._connected and self.transport:
            self.transport.write(frame)

    async def send_frame_threadsafe(self, frame: bytes):
        """线程安全发送"""
        if not self._connected or not self.transport:
            return
        loop = self._loop
        if not loop:
            return

        fut = loop.create_future()

        def do_write():
            try:
                self.transport.write(frame)
                loop.call_soon_threadsafe(fut.set_result, None)
            except Exception as ex:
                loop.call_soon_threadsafe(fut.set_exception, ex)

        loop.call_soon_threadsafe(do_write)
        await fut

    async def _watchdog_loop(self):
        """看门狗协程。修改点：只打日志，不再调用 reset_device()"""
        while self._connected:
            await asyncio.sleep(self.watchdog_interval)
            now = time.time()
            if (now - self._last_byte_time) > self.no_data_timeout:
                # 不去复位了，只提醒
                self.logger.warning(
                    f"[AsyncSerial] 超过 {self.no_data_timeout}s 无数据，请检查设备或手动复位"
                )
