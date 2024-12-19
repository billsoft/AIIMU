# gimbal_angle_reader.py

import asyncio
import math
import logging
from collections import deque
from typing import Tuple, Optional
import numpy as np
from scipy.interpolate import interp1d

from gimbal_controller import GimbalController  # 确保此路径正确
from cyber_gear_motor import CyberGearMotor      # 确保此路径正确

# 设置日志
logger = logging.getLogger("GimbalAngleReader")
logger.setLevel(logging.DEBUG)  # 设置为DEBUG以获取详细日志
handler = logging.StreamHandler()
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

class GimbalAngleReader:
    """
    云台角度读取器
    - 45Hz采样率读取云台实际角度
    - 维护历史数据用于记录
    - 生成与IMUReader相同格式的文本文件
    - 设置最大记录数，达到后自动停止并进行插值
    """

    def __init__(self, gimbal_controller: GimbalController, max_records: int = 225, output_file: str = "gimbal_angles.txt"):
        """
        初始化GimbalAngleReader。

        :param gimbal_controller: GimbalController实例
        :param max_records: 最大记录条数（例如5秒 * 45Hz = 225条）
        :param output_file: 输出文件名
        """
        self.gimbal_controller = gimbal_controller
        self.running = False
        self.max_records = max_records
        self.output_file = output_file

        # 历史数据缓存：(timestamp, yaw, pitch, roll)
        self.angle_history = deque(maxlen=max_records)

        # 异步任务
        self.read_task = None

        # 记录计数
        self.record_count = 0

    async def start(self):
        """启动角度读取器"""
        self.running = True
        try:
            self.read_task = asyncio.create_task(self._read_loop())
            logger.info("GimbalAngleReader 已启动。")
        except Exception as e:
            logger.error(f"启动 GimbalAngleReader 时发生错误: {e}")
            await self.stop()

    async def stop(self):
        """停止角度读取器，并确保电机停止"""
        self.running = False
        if self.read_task:
            self.read_task.cancel()
            try:
                await self.read_task
            except asyncio.CancelledError:
                logger.info("GimbalAngleReader 读取任务已取消。")
        await self.gimbal_controller.stop()
        logger.info("GimbalAngleReader 已停止。")

    async def _read_loop(self):
        """45Hz采样读取云台角度并记录"""
        interval = 1.0 / 45.0  # 45Hz
        loop = asyncio.get_event_loop()
        while self.running and self.record_count < self.max_records:
            try:
                current_time = loop.time()
                angles = await self._get_gimbal_angles()
                if angles:
                    yaw, pitch, roll = angles
                    # 记录数据
                    self.angle_history.append((current_time, yaw, pitch, roll))
                    self.record_count += 1
                    logger.debug(f"记录第 {self.record_count} 条数据: Timestamp={current_time:.6f}s, Yaw={math.degrees(yaw):.2f}°, Pitch={math.degrees(pitch):.2f}°, Roll={math.degrees(roll):.2f}°")
                await asyncio.sleep(interval)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"角度读取循环发生错误: {e}")
                await asyncio.sleep(interval)

        if self.record_count >= self.max_records:
            logger.info(f"已记录 {self.record_count} 条数据，准备进行插值并写入文件。")
            await self._process_and_write()

    async def _get_gimbal_angles(self) -> Optional[Tuple[float, float, float]]:
        """从云台控制器获取当前角度"""
        try:
            angle_left = self.gimbal_controller.motor_left.get_real_time_position()
            angle_right = self.gimbal_controller.motor_right.get_real_time_position()

            if angle_left is None or angle_right is None:
                logger.warning("获取到的电机位置数据为空。")
                return None

            # 计算欧拉角
            roll, pitch, yaw = self.gimbal_controller.compute_final(angle_left, angle_right)
            return yaw, pitch, roll
        except Exception as e:
            logger.error(f"获取云台角度时发生错误: {e}")
            return None

    async def _process_and_write(self):
        """对收集到的角度数据进行插值，并写入文件"""
        if len(self.angle_history) == 0:
            logger.warning("没有数据可供插值。")
            return

        # 提取原始数据
        timestamps, yaws, pitches, rolls = zip(*self.angle_history)

        # 转换为numpy数组
        timestamps = np.array(timestamps)
        yaws = np.array(yaws)
        pitches = np.array(pitches)
        rolls = np.array(rolls)

        # 创建插值函数
        yaw_interp = interp1d(timestamps, yaws, kind='linear', fill_value="extrapolate")
        pitch_interp = interp1d(timestamps, pitches, kind='linear', fill_value="extrapolate")
        roll_interp = interp1d(timestamps, rolls, kind='linear', fill_value="extrapolate")

        # 生成新的时间戳（10倍插值，即2250Hz）
        start_time = timestamps[0]
        end_time = timestamps[-1]
        num_samples = self.max_records * 10  # 10倍插值
        new_timestamps = np.linspace(start_time, end_time, num=num_samples)

        # 计算插值后的角度
        new_yaws = yaw_interp(new_timestamps)
        new_pitches = pitch_interp(new_timestamps)
        new_rolls = roll_interp(new_timestamps)

        # 写入文件
        try:
            with open(self.output_file, "w", encoding="utf-8") as f:
                f.write("Timestamp(s) Yaw Pitch Roll\n")
                for ts, yaw, pitch, roll in zip(new_timestamps, new_yaws, new_pitches, new_rolls):
                    f.write(f"{ts:.6f} {math.degrees(yaw):.2f} {math.degrees(pitch):.2f} {math.degrees(roll):.2f}\n")
            logger.info(f"插值后的数据已写入文件: {self.output_file}")
        except Exception as e:
            logger.error(f"写入插值数据到文件时发生错误: {e}")

    def get_history_data(self) -> list:
        """获取历史角度数据"""
        return list(self.angle_history)

# 为测试和调试添加的 main 函数
async def main():
    """
    测试GimbalAngleReader:
    1. 初始化电机和云台控制器
    2. 启动控制器和读取器
    3. 记录5秒数据（225条记录）
    4. 进行10倍插值（2250条记录）
    5. 写入文件
    """
    # 初始化电机，确保两个电机的 position_request_interval 都设置为45Hz
    motor_left = CyberGearMotor(
        can_id=127,
        serial_port='COM5',
        master_id=0x00FD,
        position_request_interval=1/45  # 设置为45Hz
    )
    motor_right = CyberGearMotor(
        can_id=127,
        serial_port='COM4',
        master_id=0x00FD,
        position_request_interval=1/45  # 设置为45Hz
    )

    # 初始化云台控制器
    gimbal_controller = GimbalController(motor_left, motor_right, mode='pitch_random')

    # 启动云台控制器
    controller_task = asyncio.create_task(gimbal_controller.start())

    # 初始化并启动云台角度读取器
    gimbal_angle_reader = GimbalAngleReader(
        gimbal_controller=gimbal_controller,
        max_records=225,  # 设置最大记录数为225条（5秒 * 45Hz）
        output_file="gimbal_angles.txt"  # 输出文件名
    )
    await gimbal_angle_reader.start()

    try:
        # 等待读取器完成记录
        while gimbal_angle_reader.record_count < gimbal_angle_reader.max_records:
            await asyncio.sleep(0.1)
    except KeyboardInterrupt:
        logger.info("程序被用户中断。")
    except Exception as e:
        logger.error(f"程序运行时发生异常：{e}")
    finally:
        await gimbal_angle_reader.stop()
        # 确保云台控制器停止
        controller_task.cancel()
        try:
            await controller_task
        except asyncio.CancelledError:
            logger.info("GimbalController 任务已取消。")

if __name__ == "__main__":
    asyncio.run(main())
