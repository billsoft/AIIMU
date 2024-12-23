import asyncio
import serial_asyncio
import time
import logging
from typing import Optional, Callable
from collections import deque


class AsyncSerial(asyncio.Protocol):
    """
    改进的异步串口通信类，针对Arduino 101 (Intel Curie)在高波特率下可能出现的卡死问题:
      - 保持ASCII格式解析 (AT,xxx\r\n)，不做二进制帧处理
      - 仅操作DTR进行复位 (auto_reset=True 时)
      - 可启用看门狗，每隔N秒检测是否有数据
      - 关闭时不再反复置DTR/RTS=False，减少干扰
      - 优化的缓冲区管理和数据处理机制
    """

    def __init__(
            self,
            port: str,
            baudrate: int = 115200,
            stopbits: int = 1,
            parity: str = 'N',
            on_frame_received: Optional[Callable[[bytes], None]] = None,
            logger: Optional[logging.Logger] = None,
            auto_reset: bool = False,
            reset_delay: float = 0.1,
            enable_watchdog: bool = False,
            watchdog_timeout: float = 3.0
    ):
        # 基本参数
        self.port = port
        self.baudrate = baudrate
        self.stopbits = stopbits
        self.parity = parity

        # 复位相关
        self.auto_reset = auto_reset
        self.reset_delay = reset_delay
        self._reset_count = 0
        self._max_reset_attempts = 3

        # 回调函数
        self.on_frame_received = on_frame_received

        # 缓冲区管理
        self._buffer = bytearray()
        self._max_buffer_size = 8192  # 8KB最大缓冲
        self._frame_stats = deque(maxlen=100)  # 最近100帧的统计
        self._last_byte_time = 0.0
        self._frame_timeout = 0.5  # 500ms帧超时

        # 连接状态
        self._connected = False
        self.transport = None
        self._loop = None
        self._connection_time = 0.0

        # 看门狗
        self.enable_watchdog = enable_watchdog
        self.watchdog_timeout = watchdog_timeout
        self._last_data_time = 0.0
        self._watchdog_task = None
        self._watchdog_reset_count = 0
        self._max_watchdog_resets = 5

        # 统计信息
        self._stats = {
            'frames_received': 0,
            'frames_error': 0,
            'bytes_received': 0,
            'last_frame_time': 0.0,
            'buffer_overflows': 0
        }

        # 日志设置
        if logger is None:
            logger = logging.getLogger("AsyncSerial")
            logger.setLevel(logging.DEBUG)
            if not logger.handlers:
                ch = logging.StreamHandler()
                ch.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
                logger.addHandler(ch)
        self.logger = logger

    async def connect(self):
        """连接串口并根据设置执行DTR复位(可选)"""
        try:
            if self._connected:
                self.logger.warning("[AsyncSerial] 已经连接，请先关闭")
                return

            loop = asyncio.get_event_loop()
            self._loop = loop

            self.logger.info(f"[AsyncSerial] 正在连接: {self.port}, baud={self.baudrate}, auto_reset={self.auto_reset}")
            self.transport, _ = await serial_asyncio.create_serial_connection(
                loop,
                lambda: self,
                url=self.port,
                baudrate=self.baudrate,
                stopbits=self.stopbits,
                parity=self.parity
            )

            self._connected = True
            self._connection_time = time.time()
            self.logger.info(f"[AsyncSerial] 串口 {self.port} 连接成功")

            # 优化串口配置
            await self._optimize_serial_config()

            # DTR复位
            if self.auto_reset:
                self.logger.info("[AsyncSerial] auto_reset=True, 执行DTR复位")
                await self.reset_device()

            now = time.time()
            self._last_byte_time = now
            self._last_data_time = now

            # 看门狗
            if self.enable_watchdog:
                self.logger.debug("[AsyncSerial] 启动看门狗")
                self._watchdog_task = asyncio.create_task(self._watchdog_loop())

        except Exception as e:
            self.logger.error(f"[AsyncSerial] 串口 {self.port} 连接失败: {e}")
            self._connected = False
            raise

    async def _optimize_serial_config(self):
        """优化串口配置"""
        if not hasattr(self.transport, 'serial'):
            return

        s = self.transport.serial
        try:
            # 尝试优化串口配置
            if hasattr(s, '_reconfigure_port'):
                self.logger.debug("[AsyncSerial] 优化串口配置...")
                s._reconfigure_port(
                    exclusive=True,
                    xonxoff=False,
                    rtscts=False,
                    dsrdtr=False
                )

            # 清空缓冲区
            s.reset_input_buffer()
            s.reset_output_buffer()

            # 设置较大的缓冲区
            if hasattr(s, 'set_buffer_size'):
                s.set_buffer_size(rx_size=self._max_buffer_size, tx_size=self._max_buffer_size)
        except Exception as e:
            self.logger.warning(f"[AsyncSerial] 优化串口配置失败: {e}")

    def connection_made(self, transport):
        """串口连接建立时的回调"""
        self.transport = transport
        self.logger.debug("[AsyncSerial] connection_made回调触发")
        if hasattr(transport, 'serial') and transport.serial:
            self.logger.info(
                f"[AsyncSerial] 串口 {self.port} 已建立: DTR={transport.serial.dtr}, RTS={transport.serial.rts}"
            )
        else:
            self.logger.warning("[AsyncSerial] 无法访问 transport.serial")

    def data_received(self, data: bytes):
        """ASCII模式下, 解析 'AT,xxx\\r\\n'"""
        now = time.time()

        # 更新统计
        self._stats['bytes_received'] += len(data)
        self._last_byte_time = now
        self._last_data_time = now

        # 缓冲区管理
        if len(self._buffer) + len(data) > self._max_buffer_size:
            self._stats['buffer_overflows'] += 1
            self.logger.warning(
                f"[AsyncSerial] 缓冲区溢出 ({len(self._buffer)} + {len(data)} > {self._max_buffer_size})")
            self._buffer.clear()  # 清空旧数据

        self._buffer.extend(data)
        self.logger.debug(f"[AsyncSerial] 收到 {len(data)} bytes, buffer={len(self._buffer)}")

        # 处理数据
        self._process_buffer()

    def _process_buffer(self):
        """只做 'AT' + '\\r\\n' 搜索, 不再做二进制帧头/帧尾"""
        while True:
            # 检查帧超时
            if time.time() - self._last_byte_time > self._frame_timeout and self._buffer:
                self.logger.warning(f"[AsyncSerial] 帧超时，清空缓冲区: {len(self._buffer)} bytes")
                self._buffer.clear()
                return

            start = self._buffer.find(b'AT')
            if start == -1:
                # 未找到 'AT', 全丢
                if len(self._buffer) > 0:
                    self.logger.debug("[AsyncSerial] 未发现 'AT',清空")
                self._buffer.clear()
                return

            end = self._buffer.find(b'\r\n', start)
            if end == -1:
                # 不完整, 等待更多
                return

            # 提取一行
            frame = self._buffer[start:end + 2]
            del self._buffer[:end + 2]

            # 更新统计
            self._stats['frames_received'] += 1
            self._stats['last_frame_time'] = time.time()
            frame_len = len(frame)
            self._frame_stats.append(frame_len)

            self.logger.debug(f"[AsyncSerial] 解析到frame: {frame}")

            # 回调处理
            if self.on_frame_received and callable(self.on_frame_received):
                try:
                    self.on_frame_received(frame)
                except Exception as ex:
                    self._stats['frames_error'] += 1
                    self.logger.error(f"[AsyncSerial] 处理帧异常: {ex}")

    async def _watchdog_loop(self):
        """若超过watchdog_timeout无数据,则警告/或可执行 reset_device()"""
        while self._connected:
            await asyncio.sleep(1.0)
            now = time.time()
            if now - self._last_data_time > self.watchdog_timeout:
                self._watchdog_reset_count += 1
                self.logger.warning(
                    f"[AsyncSerial] {self.port} {self.watchdog_timeout}s无数据! "
                    f"(重置次数: {self._watchdog_reset_count}/{self._max_watchdog_resets})"
                )

                if self._watchdog_reset_count <= self._max_watchdog_resets:
                    await self.reset_device()
                else:
                    self.logger.error("[AsyncSerial] 看门狗重置次数超限，请检查硬件连接")

                self._last_data_time = now

    async def reset_device(self) -> bool:
        """仅操作DTR复位，不动RTS"""
        if not self.transport or not hasattr(self.transport, 'serial') or self.transport.serial is None:
            self.logger.warning("[AsyncSerial] 无法进行DTR复位")
            return False

        self._reset_count += 1
        if self._reset_count > self._max_reset_attempts:
            self.logger.error(f"[AsyncSerial] 复位次数超过限制({self._max_reset_attempts})")
            return False

        s = self.transport.serial
        try:
            orig_dtr = s.dtr
            self.logger.debug(f"[AsyncSerial] reset_device: 原DTR={orig_dtr}")

            # 清空缓冲区
            self._buffer.clear()
            s.reset_input_buffer()
            s.reset_output_buffer()

            # DTR复位序列
            s.dtr = False
            await asyncio.sleep(self.reset_delay)
            s.dtr = True
            await asyncio.sleep(self.reset_delay)
            s.dtr = False
            await asyncio.sleep(self.reset_delay)
            s.dtr = orig_dtr

            self.logger.info(f"[AsyncSerial] DTR复位完成(第{self._reset_count}次),额外等待2s")
            await asyncio.sleep(2.0)

            # 复位成功后重置计数
            self._watchdog_reset_count = 0
            return True

        except Exception as e:
            self.logger.error(f"[AsyncSerial] DTR复位错误: {e}")
            return False

    async def send_frame(self, frame: bytes):
        """异步发送数据"""
        if not self._connected or not self.transport:
            self.logger.warning("[AsyncSerial] 未连接,无法发送")
            return
        try:
            self.transport.write(frame)
            self.logger.debug(f"[AsyncSerial] 发送: {frame}")
        except Exception as e:
            self.logger.error(f"[AsyncSerial] 发送错误: {e}")

    async def send_frame_threadsafe(self, frame: bytes):
        """线程安全发送(若多线程访问时)"""
        if not self._connected or not self.transport:
            self.logger.warning("[AsyncSerial] 未连接,无法发送")
            return

        loop = self._loop
        if not loop:
            self.logger.error("[AsyncSerial] 无事件循环,无法发送")
            return

        future = loop.create_future()

        def do_write():
            try:
                self.transport.write(frame)
                self.logger.debug(f"[AsyncSerial][Threadsafe] 发送: {frame}")
                loop.call_soon_threadsafe(future.set_result, None)
            except Exception as ex:
                loop.call_soon_threadsafe(future.set_exception, ex)

        loop.call_soon_threadsafe(do_write)
        await future

    def get_statistics(self) -> dict:
        """获取通信统计信息"""
        stats = self._stats.copy()
        if self._frame_stats:
            stats.update({
                'avg_frame_size': sum(self._frame_stats) / len(self._frame_stats),
                'max_frame_size': max(self._frame_stats),
                'min_frame_size': min(self._frame_stats)
            })
        return stats

    async def close(self):
        """关闭串口,不再强制DTR/RTS=False"""
        if not self.transport:
            self.logger.warning(f"[AsyncSerial] {self.port} 已关闭或未连接")
            return
        self.logger.info(f"[AsyncSerial] 关闭 {self.port}")

        if self._watchdog_task:
            self._watchdog_task.cancel()
            try:
                await self._watchdog_task
            except asyncio.CancelledError:
                pass

        try:
            # 清理资源
            self._buffer.clear()
            self._frame_stats.clear()
            await asyncio.sleep(0.5)

            self.transport.close()
            self._connected = False
            self.logger.info(f"[AsyncSerial] 串口 {self.port} 已关闭")
        except Exception as e:
            self.logger.error(f"[AsyncSerial] 关闭异常: {e}")
        finally:
            self._connected = False
            self.transport = None
            self._loop = None
            self._watchdog_task = None
            self._last_byte_time = 0.0
            self._last_data_time = 0.0
            self._reset_count = 0
            self._watchdog_reset_count = 0


