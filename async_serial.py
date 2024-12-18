# async_serial.py

import asyncio
import serial_asyncio
from typing import Optional, Callable
import logging


class AsyncSerial(asyncio.Protocol):
    """异步串口通信类，支持自动复位功能"""

    def __init__(
            self,
            port: str,
            baudrate: int = 115200,  # 默认波特率与Arduino代码一致
            stopbits: int = 1,
            parity: str = 'N',
            on_frame_received: Optional[Callable[[bytes], None]] = None,
            logger: Optional[logging.Logger] = None,
            auto_reset: bool = False,  # 是否在连接时自动复位
            reset_delay: float = 0.1  # 复位信号持续时间（秒）
    ):
        self.port = port
        self.baudrate = baudrate
        self.stopbits = stopbits
        self.parity = parity
        self.auto_reset = auto_reset
        self.reset_delay = reset_delay
        self.transport = None
        self.on_frame_received = on_frame_received
        self._buffer = bytearray()
        self._connected = False

        # 设置logger
        if logger is None:
            logger = logging.getLogger("AsyncSerial")
            logger.setLevel(logging.DEBUG)  # 设置为DEBUG级别以记录更多信息
            ch = logging.StreamHandler()
            ch.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
            logger.addHandler(ch)

        self.logger = logger
        self._loop = None

    async def reset_device(self):
        """通过DTR线复位设备的异步方法"""
        if not hasattr(self.transport, 'serial'):
            self.logger.warning("无法访问串口控制信号")
            return

        serial_instance = self.transport.serial
        try:
            self.logger.debug("开始设备复位序列")

            # 保存原始状态
            original_dtr = serial_instance.dtr
            original_rts = serial_instance.rts
            self.logger.debug(f"原始DTR状态: {original_dtr}, 原始RTS状态: {original_rts}")

            # 第1步：DTR设为False，RTS设为False
            self.logger.info("发送复位信号: 设置DTR和RTS为False")
            serial_instance.dtr = False
            serial_instance.rts = False
            await asyncio.sleep(self.reset_delay)

            # 第2步：DTR设为True触发复位
            self.logger.info("发送复位信号: 设置DTR为True")
            serial_instance.dtr = True
            await asyncio.sleep(self.reset_delay)

            # 第3步：DTR恢复False
            self.logger.info("发送复位信号: 设置DTR为False")
            serial_instance.dtr = False
            await asyncio.sleep(self.reset_delay)

            # 恢复原始状态
            self.logger.info("恢复DTR原始状态")
            serial_instance.dtr = original_dtr
            serial_instance.rts = original_rts
            self.logger.debug(f"恢复后的DTR状态: {serial_instance.dtr}, RTS状态: {serial_instance.rts}")

            self.logger.info("设备复位信号已发送")

            # 等待设备启动
            self.logger.debug("等待设备启动...")
            await asyncio.sleep(2.0)  # 根据设备需求调整等待时间
            self.logger.info("设备启动完成")

        except Exception as e:
            self.logger.error(f"设备复位过程中发生错误: {e}")

    async def connect(self):
        """连接串口并根据设置执行自动复位"""
        try:
            loop = asyncio.get_event_loop()
            self._loop = loop
            self.transport, _ = await serial_asyncio.create_serial_connection(
                loop,
                lambda: self,
                url=self.port,
                baudrate=self.baudrate,
                stopbits=self.stopbits,
                parity=self.parity,
            )
            self._connected = True
            self.logger.info(f"串口 {self.port} 连接成功")

            if self.auto_reset:
                self.logger.info("执行自动复位...")
                await self.reset_device()
            else:
                # 确保DTR处于False状态，防止意外复位
                if hasattr(self.transport, 'serial'):
                    self.transport.serial.dtr = False

        except Exception as e:
            self.logger.error(f"串口连接失败: {e}")
            raise

    def connection_made(self, transport):
        """串口连接建立时的回调"""
        self.transport = transport
        if hasattr(transport, 'serial'):
            self.logger.info(f"串口 {self.port} 已建立连接，DTR初始状态: {transport.serial.dtr}, RTS初始状态: {transport.serial.rts}")
        else:
            self.logger.warning(f"串口 {self.port} 已建立连接，但无法访问 'serial' 属性")

    def data_received(self, data: bytes):
        """接收到数据时的回调"""
        self._buffer.extend(data)
        self.logger.debug(f"收到数据: {data}")
        self._process_buffer()

    def _process_buffer(self):
        """处理接收缓冲区，寻找'AT'开头和'\r\n'结尾的帧"""
        while True:
            start = self._buffer.find(b'AT')
            if start == -1:
                # 未找到 'AT'，清空缓冲区等待下一次数据
                if len(self._buffer) > 0:
                    self.logger.debug("未找到 'AT' 开头，清空缓冲区")
                self._buffer.clear()
                break

            end = self._buffer.find(b'\r\n', start)
            if end == -1:
                # 未找到结束符'\r\n'，数据不完整，等待更多数据
                break

            frame = self._buffer[start:end + 2]
            del self._buffer[:end + 2]

            self.logger.debug(f"Received frame: {frame}")

            if self.on_frame_received:
                self.on_frame_received(frame)

    async def send_frame(self, frame: bytes):
        """异步发送数据帧"""
        if self._connected and self.transport:
            self.transport.write(frame)
            self.logger.debug(f"Sent frame: {frame}")
        else:
            self.logger.warning("串口未连接，无法发送数据帧")

    async def send_frame_threadsafe(self, frame: bytes):
        """
        新增线程安全异步发送方法:
        在其他线程中调用时，通过call_soon_threadsafe回到事件循环线程中执行写操作。
        """
        if not self._connected or self.transport is None:
            self.logger.warning("串口未连接，无法发送数据帧")
            return

        loop = self._loop
        future = loop.create_future()

        def do_write():
            self.transport.write(frame)
            self.logger.debug(f"Threadsafe sent frame: {frame}")
            loop.call_soon_threadsafe(future.set_result, None)

        loop.call_soon_threadsafe(do_write)
        await future

    async def close(self):
        """关闭串口连接"""
        if self.transport:
            if hasattr(self.transport, 'serial'):
                # 关闭前确保DTR和RTS信号线处于安全状态
                self.transport.serial.dtr = False
                self.transport.serial.rts = False
                self.logger.debug("DTR和RTS信号已重置为False")
            self.transport.close()
            self._connected = False
            self.logger.info(f"串口 {self.port} 已关闭")
