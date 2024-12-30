#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
IMUReader.py
============
本文件实现一个通用的异步 IMU 读取类（IMUReader），可兼容以下开发板：
1) Arduino 101 + BMI150 (输出 "AT,yaw,pitch,roll\r\n" ，约450Hz)
2) TDK DK-42688-P (输出 "AT,yaw,pitch,roll\r\n" ，约500Hz，无需DTR复位)

用户只需在初始化时，通过 `board_type`、`baudrate`、`auto_reset` 等参数进行配置，
就能在一个脚本内同时兼容两种板子，而无需改动原本的 AsyncSerial 类。

示例:
    python IMUReader.py arduino101
    或
    python IMUReader.py tdk42688
"""

import asyncio
import logging
import re
import datetime
from typing import Optional
from collections import deque

# 假设 async_serial.py 不做修改，与之前相同
from async_serial import AsyncSerial

logger = logging.getLogger("IMUReader")
logger.setLevel(logging.INFO)
handler = logging.StreamHandler()
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)


class IMUReader:
    """
    IMUReader类用于异步读取 "AT,yaw,pitch,roll\r\n" 格式的IMU数据，并写入文件或缓存。

    核心功能:
    1. 接收异步串口数据，并以队列的方式缓冲，避免回调中做耗时操作；
    2. 在后台协程中批量解析并写文件；
    3. 同时在内存维护一个数据缓冲区 (deque)，可在外部调用 get_buffer_data() 获取近期数据；
    4. 当行数达到 max_lines 后自动 stop()；
    5. 支持两种开发板: "arduino101" (需要DTR复位, 约450Hz) 和 "tdk42688" (无需DTR, 约500Hz)。
       - 区别主要是 auto_reset、默认波特率、以及写入文件名字等。
    """

    # 针对 "AT,yaw,pitch,roll" 的正则表达式
    IMU_PATTERN = re.compile(
        r"AT,([+-]?\d+(?:\.\d+)?),([+-]?\d+(?:\.\d+)?),([+-]?\d+(?:\.\d+)?)\r?$"
    )

    def __init__(
        self,
        port: str,
        board_type: str,
        baudrate: Optional[int] = None,        # 根据板子决定默认
        stopbits: int = 1,
        parity: str = 'N',
        output_file: Optional[str] = None,     # 若不指定则自动命名
        max_lines: Optional[int] = None,
        logger_instance: Optional[logging.Logger] = None,
        buffer_size: int = 100,
        auto_reset: Optional[bool] = None,     # 根据 board_type 可能自动赋值
        reset_delay: float = 0.2
    ):
        """
        参数说明:
        - port: 串口号, 如 COM7, /dev/ttyUSB0 等
        - board_type: "arduino101" 或 "tdk42688" 或其他
        - baudrate: 若不指定则会根据 board_type 给默认
        - stopbits, parity: 默认 1, 'N'
        - output_file: 数据保存文件, 若不传则根据 board_type 生成默认
        - max_lines: 存多少行后自动停止, 可为空
        - buffer_size: 二级缓存大小
        - auto_reset, reset_delay: 是否通过DTR进行复位, 及复位延迟
        """
        if not port:
            raise ValueError("必须指定port")
        if max_lines is not None and max_lines <= 0:
            raise ValueError("max_lines必须>0")
        if buffer_size <= 0:
            raise ValueError("buffer_size必须>0")
        self.board_type = board_type.lower().strip()

        # 针对不同板子设定默认 baudrate、auto_reset
        if self.board_type == "arduino101":
            default_baud = 460800
            default_auto_reset = True   # 需要DTR复位
            if output_file is None:
                output_file = "arduino101_imu_data.txt"
        elif self.board_type == "tdk42688":
            default_baud = 961200  # 修改为961200以匹配main.c
            default_auto_reset = False  # 不需要DTR
            if output_file is None:
                output_file = "tdk_imu_data.txt"
        else:
            logger.warning(f"未知 board_type={self.board_type}, 使用通用默认(115200, auto_reset=False).")
            default_baud = 115200
            default_auto_reset = False
            if output_file is None:
                output_file = "imu_data.txt"

        if baudrate is None:
            baudrate = default_baud
        if auto_reset is None:
            auto_reset = default_auto_reset

        self.port = port
        self.baudrate = baudrate
        self.stopbits = stopbits
        self.parity = parity
        self.output_file = output_file
        self.max_lines = max_lines
        self.line_count = 0
        self.running = True

        self.logger = logger_instance if logger_instance else logger

        # 初始化 AsyncSerial
        # 无需修改 async_serial.py 文件, 仅仅改变传参
        self.serial = AsyncSerial(
            port=self.port,
            baudrate=self.baudrate,
            stopbits=self.stopbits,
            parity=self.parity,
            on_frame_received=self.on_frame_received,
            logger=self.logger,
            auto_reset=auto_reset,
            reset_delay=reset_delay,
            use_watchdog=False,        # 可根据需要
            watchdog_interval=5.0,
            no_data_timeout=15.0
        )

        # 数据缓存：使用 deque 存储最新的一批IMU数据
        self.data_buffer = deque(maxlen=buffer_size)

        # 异步队列，用于暂存串口回调收到的原始数据帧
        self._frame_queue: asyncio.Queue = asyncio.Queue(maxsize=buffer_size * 2)
        self._processing_task: Optional[asyncio.Task] = None
        self.file = None

    async def connect(self):
        """
        连接串口、打开文件、启动后台处理协程。
        """
        # 1. 打开串口
        await self.serial.connect()

        # 2. 打开输出文件
        self.file = open(self.output_file, "w", encoding="utf-8", buffering=8*1024)
        # 写表头（可根据需要修改）
        self.file.write("Timestamp(s) Yaw Pitch Roll\n")
        self.logger.info(f"IMU数据将记录到文件: {self.output_file}")

        # 3. 启动后台协程 _process_loop
        self._processing_task = asyncio.create_task(self._process_loop())

    def on_frame_received(self, frame: bytes):
        """
        串口回调函数 (由 AsyncSerial 触发)，将帧放入队列。
        不要在此做耗时操作，以免阻塞回调。
        """
        if not self.running:
            return
        try:
            self._frame_queue.put_nowait(frame)
        except asyncio.QueueFull:
            self.logger.warning("IMUReader队列已满，丢弃帧")
        except Exception as e:
            self.logger.error(f"on_frame_received异常: {e}")

    async def _process_loop(self):
        """
        后台处理协程，从队列中批量取出帧，并解析 yaw/pitch/roll，写文件 & 存到 data_buffer。
        """
        self.logger.info("IMUReader 后台处理协程已启动")
        try:
            while self.running:
                # 每次先取一帧
                frame = await self._frame_queue.get()
                self._handle_frame(frame)

                # 额外尝试一次多取，减少协程切换次数
                i = 0
                while not self._frame_queue.empty() and i < 50:
                    f = self._frame_queue.get_nowait()
                    self._handle_frame(f)
                    i += 1

                # 给其他任务让出循环
                await asyncio.sleep(0)

        except asyncio.CancelledError:
            self.logger.info("IMUReader 后台协程被取消")
        except Exception as e:
            self.logger.error(f"IMUReader 后台协程异常: {e}")
        finally:
            self.logger.info("IMUReader 后台协程结束")

    def _handle_frame(self, frame: bytes):
        """
        处理单条帧数据:
        - 用正则匹配 "AT,yaw,pitch,roll"
        - 解析出 yaw/pitch/roll
        - 写入文件
        - 存到 data_buffer
        - 当行数到达 max_lines 时自动 stop()
        """
        line = frame.decode('utf-8', errors='replace').strip()
        match = self.IMU_PATTERN.match(line)
        if not match:
            return  # 无匹配, 可能是噪声或不完整

        try:
            yaw = float(match.group(1))
            pitch = float(match.group(2))
            roll = float(match.group(3))

            # 可选: 根据实际范围进行简单过滤
            # 如果 TDK output 的 yaw 范围与 101 并无区别, 就保持通用逻辑
            if not (0 <= yaw < 360 and -90 <= pitch <= 90 and -180 <= roll <= 180):
                return
            # 将 yaw 转换到 [-180, 180) 范围, 以便查看更直观
            if yaw >= 180:
                yaw -= 360

            timestamp = datetime.datetime.now().timestamp()

            # 如果设定了 max_lines, 检查是否到达
            if self.max_lines and self.line_count >= self.max_lines:
                asyncio.create_task(self.stop())
                return

            # 终端打印, 仅用于调试
            print(f"IMU: ts={timestamp:.3f}, Yaw={yaw:.2f}, Pitch={pitch:.2f}, Roll={roll:.2f}")

            # 写文件
            if self.file:
                self.file.write(f"{timestamp:.6f} {yaw:.2f} {pitch:.2f} {roll:.2f}\n")
                self.line_count += 1

            # 存到队列
            self.data_buffer.append({
                'timestamp': timestamp,
                'yaw': yaw,
                'pitch': pitch,
                'roll': roll
            })

        except Exception as e:
            self.logger.error(f"_handle_frame异常: {e}")

    async def stop(self):
        """
        停止后台处理与串口读写，关闭文件。
        """
        if not self.running:
            return
        self.running = False

        # 取消后台协程
        if self._processing_task:
            try:
                self._processing_task.cancel()
                await self._processing_task
            except asyncio.CancelledError:
                pass
            self._processing_task = None

        # 关闭串口
        await self.serial.close()

        # 关闭文件
        if self.file:
            self.file.close()
            self.logger.info(f"文件 {self.output_file} 已关闭")

        self.logger.info("IMUReader 已停止")

    def get_buffer_data(self):
        """
        返回当前 data_buffer 的所有数据(列表形式)。
        """
        return list(self.data_buffer)


async def main():
    """
    示例的 main() 函数，用于区分 Arduino 101 和 TDK 开发板。
    用户可以通过命令行参数指定板子类型，例如:
        python IMUReader.py arduino101
        python IMUReader.py tdk42688
    """
    import sys

    # 默认参数
    board_type = "tdk42688"  # 或 "arduino101"
    if len(sys.argv) > 1:
        board_type = sys.argv[1].lower()

    # 根据不同板子设置默认值:
    if board_type == "arduino101":
        imu_port = "COM7"  # 根据实际情况修改
        # Arduino 通常波特率 460800, 需要 DTR
        imu_baudrate = 230400
        auto_reset = True
        reset_delay = 0.2
        imu_output_file = "arduino101_imu_data.txt"
        imu_max_lines = 10000
    elif board_type == "tdk42688":
        imu_port = "COM11"  # 根据实际情况修改
        # TDK 使用 961200，无需 DTR
        imu_baudrate = 921600
        auto_reset = False
        reset_delay = 0.0
        imu_output_file = "tdk_imu_data.txt"
        imu_max_lines = 20000
    else:
        logger.warning(f"未知板子类型: {board_type}，使用通用默认")
        imu_port = "COM7"  # 根据实际情况修改
        imu_baudrate = 115200
        auto_reset = False
        reset_delay = 0.0
        imu_output_file = "imu_data.txt"
        imu_max_lines = 5000

    imu_reader = IMUReader(
        port=imu_port,
        board_type=board_type,
        baudrate=imu_baudrate,
        output_file=imu_output_file,
        max_lines=imu_max_lines,
        buffer_size=10000,
        auto_reset=auto_reset,
        reset_delay=reset_delay
    )

    try:
        await imu_reader.connect()
        logger.info(f"开始读取 {board_type} 开发板的IMU数据, 按 Ctrl+C 退出...")
        while imu_reader.running:
            await asyncio.sleep(1.0)  # 每秒空转一次
    except KeyboardInterrupt:
        logger.warning("用户中断")
    except Exception as e:
        logger.error(f"IMUReader 调试时发生异常: {e}")
    finally:
        await imu_reader.stop()


if __name__ == "__main__":
    # 直接运行时，异步执行 main()
    asyncio.run(main())
