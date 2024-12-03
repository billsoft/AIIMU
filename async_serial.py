# async_serial.py

"""
异步串口通信类，处理串口连接和数据收发，使用事件回调机制。
"""

import asyncio
import serial_asyncio
from typing import Optional, Callable
from logger import Logger

class AsyncSerial(asyncio.Protocol):
    """异步串口通信类"""

    def __init__(
        self,
        port: str,
        baudrate: int = 921600,
        stopbits: int = 1,
        parity: str = 'N',
        on_frame_received: Optional[Callable[[bytes], None]] = None,
        logger: Optional[Logger] = None
    ):
        self.port = port                                  # 串口端口
        self.baudrate = baudrate                          # 波特率
        self.stopbits = stopbits                          # 停止位
        self.parity = parity                              # 校验位
        self.transport = None                             # 传输层
        self.on_frame_received = on_frame_received        # 帧接收回调
        self._buffer = bytearray()                        # 接收缓冲区
        self._connected = False                           # 连接状态
        self.logger = logger                              # 日志记录器

    async def connect(self):
        """连接串口"""
        try:
            loop = asyncio.get_event_loop()
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

            # 日志记录
            if self.logger:
                asyncio.create_task(self.logger.log_received(frame))

            # 回调处理接收到的帧
            if self.on_frame_received:
                self.on_frame_received(frame)

    async def send_frame(self, frame: bytes):
        """发送数据帧"""
        if self._connected and self.transport:
            self.transport.write(frame)
            # 日志记录
            if self.logger:
                await self.logger.log_send(frame)
        else:
            print("串口未连接")

    async def close(self):
        """关闭串口连接"""
        if self.transport:
            self.transport.close()
            self._connected = False
            print(f"串口 {self.port} 已关闭")
