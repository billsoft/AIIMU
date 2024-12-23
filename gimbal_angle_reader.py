import asyncio
import math
import logging
from collections import deque
from typing import Tuple, Optional
import numpy as np
from scipy.interpolate import interp1d
import datetime

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
    云台角度读取器:
    - 以 45Hz 读取电机弧度,计算 roll_rad, pitch_rad, yaw_rad
    - 转为度数后, 再存成 dict => {'timestamp':ts, 'roll':..., 'pitch':..., 'yaw':...}
    - 记录满 max_records 后做插值写文件
    - 提供 get_history_data() 返回 list[dict], dict格式=>{'timestamp':..., 'yaw':..., 'pitch':..., 'roll':...}
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
                current_time = datetime.datetime.now().timestamp()
                angles = await self._get_gimbal_angles()  # 返回 dict
                if angles:
                    # angles = {'roll':..., 'pitch':..., 'yaw':...}
                    roll_deg = angles['roll']
                    pitch_deg = angles['pitch']
                    yaw_deg = angles['yaw']
                    self.angle_history.append({
                        'timestamp': current_time,
                        'roll': roll_deg,
                        'pitch': pitch_deg,
                        'yaw': yaw_deg
                    })
                    self.record_count += 1

                    logger.debug(f"Gimbal第{self.record_count}条: ts={current_time:.3f}, R={roll_deg:.2f}, P={pitch_deg:.2f}, Y={yaw_deg:.2f}")
                await asyncio.sleep(interval)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"角度读取循环错误: {e}")
                await asyncio.sleep(interval)

        if self.record_count >= self.max_records:
            await self._process_and_write()

    async def _get_gimbal_angles(self) -> Optional[dict]:
        """
        从电机获取弧度 => compute_final => 转度数 (roll_deg, pitch_deg, yaw_deg),
        再将 pitch 乘以 -1(若需要与 IMU 对齐方向).
        最后返回 dict{'roll':..., 'pitch':..., 'yaw':...}
        """
        try:
            angle_left = self.gimbal_controller.motor_left.get_real_time_position()
            angle_right = self.gimbal_controller.motor_right.get_real_time_position()
            if angle_left is None or angle_right is None:
                logger.warning("电机位置数据为空")
                return None
            # compute_final 返回 (roll_rad, pitch_rad, yaw_rad)
            roll_rad, pitch_rad, yaw_rad = self.gimbal_controller.compute_final(angle_left, angle_right)

            roll_deg = math.degrees(roll_rad)
            pitch_deg = math.degrees(pitch_rad)
            yaw_deg = math.degrees(yaw_rad)
            # 如果要反向pitch
            pitch_deg = -pitch_deg

            return {
                'roll': roll_deg,
                'pitch': pitch_deg,
                'yaw': yaw_deg
            }
        except Exception as e:
            logger.error(f"获取云台角度时异常: {e}")
            return None

    async def _process_and_write(self):
        if len(self.angle_history) == 0:
            logger.warning("无数据可插值")
            return

        # 拆包: list of dict => arrays
        timestamps = np.array([x['timestamp'] for x in self.angle_history])
        rolls = np.array([x['roll'] for x in self.angle_history])
        pitches = np.array([x['pitch'] for x in self.angle_history])
        yaws = np.array([x['yaw'] for x in self.angle_history])

        # 创建插值函数
        from scipy.interpolate import interp1d
        roll_interp = interp1d(timestamps, rolls, kind='linear', fill_value="extrapolate")
        pitch_interp = interp1d(timestamps, pitches, kind='linear', fill_value="extrapolate")
        yaw_interp = interp1d(timestamps, yaws, kind='linear', fill_value="extrapolate")

        # 10倍插值
        new_ts = np.linspace(timestamps[0], timestamps[-1], self.max_records*10)
        new_rolls = roll_interp(new_ts)
        new_pitches = pitch_interp(new_ts)
        new_yaws = yaw_interp(new_ts)

        try:
            with open(self.output_file, "w", encoding="utf-8") as f:
                f.write("Timestamp(s) Yaw Pitch Roll\n")
                for t, y, p, r in zip(new_ts, new_yaws, new_pitches, new_rolls):
                    f.write(f"{t:.6f} {y:.2f} {p:.2f} {r:.2f}\n")
            logger.info(f"插值后写文件完成: {self.output_file}")
        except Exception as e:
            logger.error(f"写文件错误: {e}")

    def get_history_data(self) -> list:
        """返回list[dict], 每条dict格式: {timestamp, roll, pitch, yaw} (度数)"""
        return list(self.angle_history)


# 测试main
async def main():
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

    controller_task = asyncio.create_task(gimbal_controller.start())
    gimbal_angle_reader = GimbalAngleReader(gimbal_controller, max_records=315, output_file="gimbal_angles.txt")
    await gimbal_angle_reader.start()

    try:
        while gimbal_angle_reader.record_count < 315:
            await asyncio.sleep(0.1)
    finally:
        await gimbal_angle_reader.stop()
        controller_task.cancel()
        try:
            await controller_task
        except:
            pass

if __name__ == "__main__":
    asyncio.run(main())
