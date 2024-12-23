# gimbal_angle_reader.py

import asyncio
import math
import logging
from collections import deque
from typing import Tuple, Optional
import numpy as np
from scipy.interpolate import interp1d
import datetime  # 引入datetime模块，用于与IMU统一时间戳

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
    1. 以 45Hz 的频率采样云台姿态（Roll, Pitch, Yaw）；
    2. 使用与 IMU 相同的时间戳获取方式（datetime.datetime.now().timestamp()）；
    3. 将读取到的“电机输出（弧度）”转换为“度数”后存入队列；
    4. 记录满 max_records 条后，在 _process_and_write() 中插值并写入文件：
       - 无需再做 math.degrees()，因为我们已经转换成度数存储；
       - 写入文件时，直接写度数；
    5. 提供 get_history_data() 接口给上层（DataSynchronizer），返回的也是度数，方便与 IMU 记录对比同步。
    """

    def __init__(
        self,
        gimbal_controller: GimbalController,
        max_records: int = 315,
        output_file: str = "gimbal_angles.txt"
    ):
        """
        :param gimbal_controller: 云台控制器实例，用于获得电机位置并计算姿态
        :param max_records: 记录的最大条数（例如7秒*45Hz=315）
        :param output_file: 写入插值后姿态数据的文件名
        """
        self.gimbal_controller = gimbal_controller
        self.running = False
        self.max_records = max_records
        self.output_file = output_file

        # 存储姿态数据的队列；队列内格式为: (timestamp, yaw_deg, pitch_deg, roll_deg)
        # 都是“度数”而非“弧度”
        self.angle_history = deque(maxlen=max_records)

        # 异步任务句柄
        self.read_task = None
        self.record_count = 0

    async def start(self):
        """
        启动云台角度读取器，并创建异步任务进行循环采样。
        """
        self.running = True
        try:
            # 创建读取任务
            self.read_task = asyncio.create_task(self._read_loop())
            logger.info("GimbalAngleReader 已启动。")
        except Exception as e:
            logger.error(f"启动 GimbalAngleReader 时发生错误: {e}")
            await self.stop()

    async def stop(self):
        """
        停止云台角度读取器，同时停止云台控制器。
        """
        self.running = False
        if self.read_task:
            self.read_task.cancel()
            try:
                await self.read_task
            except asyncio.CancelledError:
                logger.info("GimbalAngleReader 读取任务已取消。")

        # 停止云台控制器（内部会令电机急停并关闭串口）
        await self.gimbal_controller.stop()
        logger.info("GimbalAngleReader 已停止。")

    async def _read_loop(self):
        """
        循环读取云台角度的协程任务：
        1. 使用 datetime.datetime.now().timestamp() 获取系统时间戳；
        2. 从电机获取弧度并转换为度数；
        3. 存入 self.angle_history；
        4. 直到记录满 max_records 条后调用 _process_and_write() 写文件。
        """
        interval = 1.0 / 45.0  # 45Hz
        while self.running and self.record_count < self.max_records:
            try:
                # 统一使用与 IMU 相同的时间戳获取方式
                current_time = datetime.datetime.now().timestamp()

                # 获取云台姿态：yaw_rad, pitch_rad, roll_rad （弧度）
                angles = await self._get_gimbal_angles()
                if angles:
                    # 这里的 angles = (yaw_deg, pitch_deg, roll_deg)
                    yaw_deg, pitch_deg, roll_deg = angles

                    # 存入队列：队列里始终是度数
                    self.angle_history.append((current_time, yaw_deg, pitch_deg, roll_deg))
                    self.record_count += 1

                    logger.debug(f"记录第 {self.record_count} 条数据: Timestamp={current_time:.6f}s | "
                                 f"Yaw={yaw_deg:.2f}° | Pitch={pitch_deg:.2f}° | Roll={roll_deg:.2f}°")

                # 等待下一个采样周期
                await asyncio.sleep(interval)

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"角度读取循环发生错误: {e}")
                await asyncio.sleep(interval)

        # 如果记录已达上限，则执行插值并写文件
        if self.record_count >= self.max_records:
            await self._process_and_write()

    async def _get_gimbal_angles(self) -> Optional[Tuple[float, float, float]]:
        """
        1. 从云台控制器获取电机位置(弧度)；
        2. 调用 compute_final() 计算 (roll_rad, pitch_rad, yaw_rad)；
        3. 统一转化为度数；【此处额外将 pitch 值乘以 -1 以适配 IMU 俯仰方向】
        """
        try:
            angle_left_rad = self.gimbal_controller.motor_left.get_real_time_position()
            angle_right_rad = self.gimbal_controller.motor_right.get_real_time_position()
            if angle_left_rad is None or angle_right_rad is None:
                logger.warning("获取到的电机位置数据为空。")
                return None

            # gimbal_controller.compute_final() 返回 (roll_rad, pitch_rad, yaw_rad)
            roll_rad, pitch_rad, yaw_rad = self.gimbal_controller.compute_final(
                angle_left_rad, angle_right_rad
            )

            # 统一将弧度 -> 度
            roll_deg = math.degrees(roll_rad)
            pitch_deg = math.degrees(pitch_rad)
            yaw_deg = math.degrees(yaw_rad)

            # == 关键改动：将 pitch 取负号，以适配 IMU 俯仰方向 ==
            pitch_deg = -pitch_deg

            return (yaw_deg, pitch_deg, roll_deg)

        except Exception as e:
            logger.error(f"获取云台角度时发生错误: {e}")
            return None

    async def _process_and_write(self):
        """
        当记录达到上限时：
        1. 对 self.angle_history 中的 [time, yaw_deg, pitch_deg, roll_deg] 做插值；
        2. 写入文件 self.output_file，写入的单位仍是度数。
        """
        if len(self.angle_history) == 0:
            logger.warning("没有数据可供插值。")
            return

        # 拆分队列： timestamps, yaws, pitches, rolls (都是度数)
        timestamps, yaws, pitches, rolls = zip(*self.angle_history)

        # 转为 numpy 数组，便于插值
        timestamps = np.array(timestamps)
        yaws = np.array(yaws)
        pitches = np.array(pitches)
        rolls = np.array(rolls)

        # 定义线性插值函数
        yaw_interp   = interp1d(timestamps, yaws,    kind='linear', fill_value="extrapolate")
        pitch_interp = interp1d(timestamps, pitches, kind='linear', fill_value="extrapolate")
        roll_interp  = interp1d(timestamps, rolls,   kind='linear', fill_value="extrapolate")

        # 生成新的时间戳：10倍插值 -> self.max_records * 10
        new_timestamps = np.linspace(
            timestamps[0],
            timestamps[-1],
            num=self.max_records * 10
        )

        # 执行插值得到度数
        new_yaws    = yaw_interp(new_timestamps)
        new_pitches = pitch_interp(new_timestamps)
        new_rolls   = roll_interp(new_timestamps)

        # 写文件：这里写出的就是度数
        try:
            with open(self.output_file, "w", encoding="utf-8") as f:
                f.write("Timestamp(s) Yaw Pitch Roll\n")
                for ts, yaw_deg, pitch_deg, roll_deg in zip(new_timestamps, new_yaws, new_pitches, new_rolls):
                    f.write(f"{ts:.6f} {yaw_deg:.2f} {pitch_deg:.2f} {roll_deg:.2f}\n")
            logger.info(f"插值后的数据已写入文件: {self.output_file}")
        except Exception as e:
            logger.error(f"写入插值数据到文件时发生错误: {e}")

    def get_history_data(self) -> list:
        """
        返回队列中采集到的云台角度数据（单位：度）给上层 (main.py/DataSynchronizer) 同步。
        """
        return list(self.angle_history)


# ======================= 以下为独立测试main函数，可不改动 =======================
async def main():
    """
    单独测试 GimbalAngleReader 的主函数
    1. 初始化电机
    2. 初始化云台控制器
    3. 启动读取器
    4. 等待记录结束后停止
    """
    motor_left = CyberGearMotor(
        can_id=127,
        serial_port='COM5',
        master_id=0x00FD,
        position_request_interval=1/45
    )
    motor_right = CyberGearMotor(
        can_id=127,
        serial_port='COM4',
        master_id=0x00FD,
        position_request_interval=1/45
    )

    gimbal_controller = GimbalController(motor_left, motor_right, mode='pitch_random')

    # 启动云台控制器
    controller_task = asyncio.create_task(gimbal_controller.start())

    # 初始化并启动 GimbalAngleReader，示例：记录7秒*45Hz=315条
    gimbal_angle_reader = GimbalAngleReader(
        gimbal_controller=gimbal_controller,
        max_records=315,
        output_file="gimbal_angles.txt"
    )
    await gimbal_angle_reader.start()

    try:
        # 等待数据记录完成
        while gimbal_angle_reader.record_count < gimbal_angle_reader.max_records:
            await asyncio.sleep(0.1)
    except KeyboardInterrupt:
        logger.info("测试程序被用户中断。")
    except Exception as e:
        logger.error(f"程序运行时发生异常：{e}")
    finally:
        # 停止读取器
        await gimbal_angle_reader.stop()
        # 取消并等待云台控制器结束
        controller_task.cancel()
        try:
            await controller_task
        except asyncio.CancelledError:
            pass

if __name__ == "__main__":
    asyncio.run(main())
