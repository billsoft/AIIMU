import asyncio
import serial_asyncio
from typing import Optional, Callable
from logger import Logger

class AsyncSerial(asyncio.Protocol):
    """异步串口通信类，增加线程安全的发送方法。"""

    def __init__(
        self,
        port: str,
        baudrate: int = 921600,
        stopbits: int = 1,
        parity: str = 'N',
        on_frame_received: Optional[Callable[[bytes], None]] = None,
        logger: Optional[Logger] = None
    ):
        self.port = port
        self.baudrate = baudrate
        self.stopbits = stopbits
        self.parity = parity
        self.transport = None
        self.on_frame_received = on_frame_received
        self._buffer = bytearray()
        self._connected = False
        self.logger = logger
        self._loop = None  # 用于call_soon_threadsafe回调

    async def connect(self):
        """连接串口"""
        try:
            loop = asyncio.get_event_loop()
            self._loop = loop
            self.transport, _ = await serial_asyncio.create_serial_connection(
                loop,
                lambda: self,
                url=self.port,
                baudrate=self.baudrate,
                stopbits=self.stopbits,
                parity=self.parity
            )
            self._connected = True
            print(f"串口 {self.port} 连接成功")
        except Exception as e:
            print(f"串口连接失败: {e}")
            raise

    def connection_made(self, transport):
        """串口连接建立时的回调"""
        self.transport = transport

    def data_received(self, data: bytes):
        """接收到数据时的回调"""
        self._buffer.extend(data)
        self._process_buffer()

    def _process_buffer(self):
        """处理接收缓冲区，解决粘包和分包问题"""
        while True:
            start = self._buffer.find(b'AT')
            if start == -1:
                # 如果缓冲区中没有 'AT'，清空缓冲区
                self._buffer.clear()
                break

            end = self._buffer.find(b'\r\n', start)
            if end == -1:
                # 如果没有找到帧结束符，等待更多数据
                break

            frame = self._buffer[start:end + 2]
            del self._buffer[:end + 2]

            if self.logger:
                asyncio.create_task(self.logger.log_received(frame))

            if self.on_frame_received:
                self.on_frame_received(frame)

    async def send_frame(self, frame: bytes):
        """异步发送数据帧（原有方法，不删除）"""
        if self._connected and self.transport:
            self.transport.write(frame)
            if self.logger:
                await self.logger.log_send(frame)
        else:
            print("串口未连接")

    async def send_frame_threadsafe(self, frame: bytes):
        """
        新增的线程安全异步发送方法:
        即使在多线程环境调用此方法，也不会直接在其他线程调用transport.write，
        而是使用call_soon_threadsafe在主事件循环中执行写操作。
        """
        if not self._connected or self.transport is None:
            print("串口未连接")
            return

        loop = self._loop
        future = loop.create_future()

        def do_write():
            self.transport.write(frame)
            # 写入完成后通知future
            loop.call_soon_threadsafe(future.set_result, None)

        # 调度写入回到事件循环线程安全执行
        loop.call_soon_threadsafe(do_write)
        await future

        if self.logger:
            await self.logger.log_send(frame)

    async def close(self):
        """关闭串口连接"""
        if self.transport:
            self.transport.close()
            self._connected = False
            print(f"串口 {self.port} 已关闭")
