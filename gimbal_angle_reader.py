# gimbal_angle_reader.py

import asyncio
import math
import logging
from collections import deque
from typing import Tuple, Optional
import numpy as np
from scipy.interpolate import interp1d
import datetime  # 新增

from gimbal_controller import GimbalController
from cyber_gear_motor import CyberGearMotor

logger = logging.getLogger("GimbalAngleReader")
logger.setLevel(logging.DEBUG)
handler = logging.StreamHandler()
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

class GimbalAngleReader:
    """
    云台角度读取器：
    将时间戳获取方式与IMUReader统一，使用 datetime.datetime.now().timestamp()
    """

    def __init__(self, gimbal_controller: GimbalController, max_records: int = 315, output_file: str = "gimbal_angles.txt"):
        self.gimbal_controller = gimbal_controller
        self.running = False
        self.max_records = max_records
        self.output_file = output_file
        self.angle_history = deque(maxlen=max_records)
        self.read_task = None
        self.record_count = 0

    async def start(self):
        self.running = True
        try:
            self.read_task = asyncio.create_task(self._read_loop())
            logger.info("GimbalAngleReader 已启动。")
        except Exception as e:
            logger.error(f"启动 GimbalAngleReader 时发生错误: {e}")
            await self.stop()

    async def stop(self):
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
        interval = 1.0 / 45.0
        while self.running and self.record_count < self.max_records:
            try:
                # 使用与IMU相同的timestamp规则
                current_time = datetime.datetime.now().timestamp()
                angles = await self._get_gimbal_angles()
                if angles:
                    yaw, pitch, roll = angles
                    self.angle_history.append((current_time, yaw, pitch, roll))
                    self.record_count += 1
                    logger.debug(f"记录第 {self.record_count} 条数据: Timestamp={current_time:.6f}s")
                await asyncio.sleep(interval)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"角度读取循环发生错误: {e}")
                await asyncio.sleep(interval)

        if self.record_count >= self.max_records:
            await self._process_and_write()

    async def _get_gimbal_angles(self) -> Optional[Tuple[float, float, float]]:
        try:
            angle_left = self.gimbal_controller.motor_left.get_real_time_position()
            angle_right = self.gimbal_controller.motor_right.get_real_time_position()
            if angle_left is None or angle_right is None:
                logger.warning("获取到的电机位置数据为空。")
                return None
            roll, pitch, yaw = self.gimbal_controller.compute_final(angle_left, angle_right)
            return yaw, pitch, roll
        except Exception as e:
            logger.error(f"获取云台角度时发生错误: {e}")
            return None

    async def _process_and_write(self):
        if len(self.angle_history) == 0:
            logger.warning("没有数据可供插值。")
            return

        timestamps, yaws, pitches, rolls = zip(*self.angle_history)

        timestamps = np.array(timestamps)
        yaws = np.array(yaws)
        pitches = np.array(pitches)
        rolls = np.array(rolls)

        yaw_interp = interp1d(timestamps, yaws, kind='linear', fill_value="extrapolate")
        pitch_interp = interp1d(timestamps, pitches, kind='linear', fill_value="extrapolate")
        roll_interp = interp1d(timestamps, rolls, kind='linear', fill_value="extrapolate")

        new_timestamps = np.linspace(timestamps[0], timestamps[-1], num=self.max_records*10)
        new_yaws = yaw_interp(new_timestamps)
        new_pitches = pitch_interp(new_timestamps)
        new_rolls = roll_interp(new_timestamps)

        try:
            with open(self.output_file, "w", encoding="utf-8") as f:
                f.write("Timestamp(s) Yaw Pitch Roll\n")
                for ts, yaw, pitch, roll in zip(new_timestamps, new_yaws, new_pitches, new_rolls):
                    f.write(f"{ts:.6f} {math.degrees(yaw):.2f} {math.degrees(pitch):.2f} {math.degrees(roll):.2f}\n")
            logger.info(f"插值后的数据已写入文件: {self.output_file}")
        except Exception as e:
            logger.error(f"写入插值数据到文件时发生错误: {e}")

    def get_history_data(self) -> list:
        return list(self.angle_history)

# main测试函数保留
async def main():
    motor_left = CyberGearMotor(can_id=127, serial_port='COM5', master_id=0x00FD, position_request_interval=1/45)
    motor_right = CyberGearMotor(can_id=127, serial_port='COM4', master_id=0x00FD, position_request_interval=1/45)
    gimbal_controller = GimbalController(motor_left, motor_right, mode='pitch_random')

    controller_task = asyncio.create_task(gimbal_controller.start())
    gimbal_angle_reader = GimbalAngleReader(gimbal_controller, max_records=315, output_file="gimbal_angles.txt")
    await gimbal_angle_reader.start()

    try:
        while gimbal_angle_reader.record_count < gimbal_angle_reader.max_records:
            await asyncio.sleep(0.1)
    except KeyboardInterrupt:
        logger.info("程序被用户中断。")
    except Exception as e:
        logger.error(f"程序运行时发生异常：{e}")
    finally:
        await gimbal_angle_reader.stop()
        controller_task.cancel()
        try:
            await controller_task
        except asyncio.CancelledError:
            pass

if __name__ == "__main__":
    asyncio.run(main())
