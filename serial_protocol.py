# serial_protocol.py

"""
自定义串口通信协议类，处理串口连接和数据收发，解决分包、粘包和损坏的帧。
"""

import asyncio

class SerialProtocol(asyncio.Protocol):
    """
    串口通信协议类，继承自 asyncio.Protocol，用于处理串口连接和数据收发。
    """

    def __init__(self, motor):
        self.motor = motor       # 电机控制实例
        self.transport = None    # 串口传输实例
        self.buffer = bytearray()  # 接收缓冲区

    def connection_made(self, transport):
        """
        串口连接建立时的回调。

        :param transport: 串口传输实例
        """
        self.transport = transport
        print(f"串口 {self.motor.serial_port} 连接成功。")

    def data_received(self, data):
        """
        接收到数据时的回调。

        :param data: 接收到的字节数据
        """
        self.buffer.extend(data)
        self.handle_buffer()

    def handle_buffer(self):
        """
        处理缓冲区中的数据，解决分包、粘包和损坏的帧。
        """
        while True:
            # 查找帧的起始标志 'AT'
            start_index = self.buffer.find(b'AT')
            if start_index == -1:
                # 未找到起始标志，丢弃缓冲区内容
                self.buffer.clear()
                break

            # 查找帧的结束标志 '\r\n'
            end_index = self.buffer.find(b'\r\n', start_index)
            if end_index == -1:
                # 未找到结束标志，等待更多数据
                break

            # 提取完整的帧
            frame = self.buffer[start_index:end_index + 2]  # 包含 '\r\n'
            # 从缓冲区中移除已处理的数据
            del self.buffer[:end_index + 2]
            # 处理完整的帧
            asyncio.create_task(self.motor.handle_frame(frame))

    def connection_lost(self, exc):
        """
        串口连接断开时的回调。

        :param exc: 异常信息
        """
        print(f"串口 {self.motor.serial_port} 已关闭。")
        self.transport = None

    def send_data(self, data):
        """
        发送数据到串口。

        :param data: 待发送的字节数据
        """
        if self.transport:
            self.transport.write(data)
            # 日志记录
            asyncio.create_task(self.motor.logger.log_send(data))
        else:
            print("串口未连接，无法发送数据。")
