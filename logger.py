# logger.py

"""
定义日志记录器类，负责日志的记录和线程安全。
"""

import asyncio
import time

class Logger:
    """
    日志记录器，用于记录发送和接收的报文，支持异步操作和线程安全。
    """

    def __init__(self, port='COM5'):
        self.message_number = 0        # 消息编号，用于标记日志顺序
        self.port = port               # 串口名称
        self.lock = asyncio.Lock()     # 异步锁，确保日志记录的线程安全

    async def log_send(self, data: bytes):
        """
        记录发送的报文。

        :param data: 发送的字节数据
        """
        async with self.lock:
            timestamp = time.strftime('%Y/%m/%d %H:%M:%S', time.localtime())
            hex_data = ' '.join(f'{byte:02X}' for byte in data)
            print(f"{self.message_number} [{timestamp}]  -> {self.port}: {hex_data}")
            self.message_number += 1

    async def log_received(self, data: bytes):
        """
        记录接收到的报文。

        :param data: 接收到的字节数据
        """
        async with self.lock:
            timestamp = time.strftime('%Y/%m/%d %H:%M:%S', time.localtime())
            hex_data = ' '.join(f'{byte:02X}' for byte in data)
            print(f"{self.message_number} [{timestamp}] Received: {hex_data}")
            self.message_number += 1
