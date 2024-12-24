import asyncio
import serial
import serial_asyncio
import logging
import time
import atexit
import signal
from typing import Optional, Callable

def _default_logger(name="AsyncSerial", level=logging.DEBUG):
    logger = logging.getLogger(name)
    logger.setLevel(level)
    if not logger.handlers:
        ch = logging.StreamHandler()
        ch.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
        logger.addHandler(ch)
    return logger

class AsyncSerial(asyncio.Protocol):
    """
    异步串口通信类，支持自动复位功能（仅使用DTR）并可选看门狗机制来监控长时间无数据。
    增强点：
    1. 添加_force_cleanup_port() 函数，可在异常后再次启动前，强制打开->关闭一次串口，企图恢复。
    2. 延长复位等待时长到3秒，并维持DTR=True。
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
            auto_reset: bool = False,
            reset_delay: float = 0.2,  # 每一步脉冲的时间

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

        if logger is None:
            logger = _default_logger()
        self.logger = logger

        self._frame_timeout = 0.1
        self._last_byte_time = 0.0
        self._watchdog_task: Optional[asyncio.Task] = None

        # 注册 atexit，尝试温和退出
        @atexit.register
        def close_on_exit():
            self.logger.debug("[AsyncSerial] atexit: trying to close port if open.")
            if self._connected:
                # 因为是同步函数，这里仅做简单处理
                try:
                    if self.transport:
                        self.transport.close()
                except:
                    pass

        # 注册信号处理器（只能拦截TERM/INT, 强杀无效）
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)

    def _signal_handler(self, signum, frame):
        self.logger.info(f"[AsyncSerial] Caught signal {signum}, will close port.")
        if self._connected and self.transport:
            try:
                self.transport.close()
            except:
                pass
        # 这里不能直接调用async函数，只能做简单处理

    @staticmethod
    def _force_cleanup_port(port: str, logger: logging.Logger):
        """
        强制打开->关闭一次串口，并设置 DTR=True，再关闭。
        用于异常退出后再次启动时，尝试让底层驱动恢复。
        """
        logger.info(f"[AsyncSerial] force_cleanup_port: try open->close {port}")
        try:
            ser = serial.Serial(port, 9600, timeout=0.5)
            ser.dtr = True
            ser.rts = False
            time.sleep(0.3)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            ser.close()
            logger.info(f"[AsyncSerial] force_cleanup_port done.")
        except Exception as e:
            logger.warning(f"[AsyncSerial] force_cleanup_port failed: {e}")

    async def connect(self):
        """异步连接串口并根据需要执行DTR复位。"""
        self._loop = asyncio.get_event_loop()

        # 先做一次强制清理（可选）
        self._force_cleanup_port(self.port, self.logger)

        try:
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

            if self.auto_reset:
                await self.reset_device()
            else:
                if hasattr(self.transport, 'serial') and self.transport.serial:
                    try:
                        # 维持 DTR=True, 避免意外复位
                        self.transport.serial.dtr = True
                    except Exception as e:
                        self.logger.warning(f"[AsyncSerial] 设置DTR=True异常: {e}")

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

            ser.dtr = False
            ser.rts = False
            await asyncio.sleep(self.reset_delay)

            ser.dtr = True
            await asyncio.sleep(self.reset_delay)

            ser.dtr = False
            await asyncio.sleep(self.reset_delay)

            # 最后保持 DTR=True
            ser.dtr = True
            ser.rts = orig_rts

            # 等待更久，让Arduino 101有充分的枚举时间
            wait_time = 3.0  # 修改：从2s增至3s或更长
            self.logger.info(f"[AsyncSerial] 发送DTR复位脉冲完成, 等待{wait_time}秒重启")
            await asyncio.sleep(wait_time)

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
        if (now - self._last_byte_time) > self._frame_timeout and len(self._buffer) > 0:
            self._buffer.clear()
        self._last_byte_time = now

        self._buffer.extend(data)
        self._process_buffer()

    def _process_buffer(self):
        """查找 'AT'...'\r\n' 帧"""
        while True:
            start = self._buffer.find(b'AT')
            if start == -1:
                if len(self._buffer) > 10:
                    self._buffer = self._buffer[-10:]
                break

            end = self._buffer.find(b'\r\n', start)
            if end == -1:
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
        """看门狗协程，不再自动复位，只做超时警告"""
        while self._connected:
            await asyncio.sleep(self.watchdog_interval)
            now = time.time()
            if (now - self._last_byte_time) > self.no_data_timeout:
                self.logger.warning(
                    f"[AsyncSerial] 超过 {self.no_data_timeout}s 无数据，请检查设备或手动复位"
                )
