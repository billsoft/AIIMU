import asyncio
import random
import math
import logging
import datetime
import re
from dataclasses import dataclass
from async_serial import AsyncSerial
from cyber_gear_motor import CyberGearMotor

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# 正则表达式匹配 "AT,yaw,pitch,roll\r\n"
IMU_PATTERN = re.compile(r"AT,([+-]?\d+(?:\.\d+)?),([+-]?\d+(?:\.\d+)?),([+-]?\d+(?:\.\d+)?)\r?$")

@dataclass
class EulerAngles:
    yaw: float
    pitch: float


class Stabilizer:
    def __init__(self):
        self.motor_left = CyberGearMotor(
            can_id=127,
            serial_port='COM5',
            master_id=0x00FD,
            position_request_interval=1 / 50  # 修改为50Hz
        )

        self.motor_right = CyberGearMotor(
            can_id=127,
            serial_port='COM4',
            master_id=0x00FD,
            position_request_interval=1 / 50  # 修改为50Hz
        )

        self.imu_com = AsyncSerial(
            port='COM11',
            baudrate=921600,
            on_frame_received=self._on_frame_received
        )
        pass

    def _on_frame_received(self, frame: bytes):
        match = IMU_PATTERN.match(frame.decode('utf-8', errors='replace').strip())
        if match:
            try:
                yaw, pitch, roll = map(float, match.groups())
                logger.info(f"Received IMU data: yaw={yaw}, pitch={pitch}, roll={roll}")
                self._stabilize(yaw, pitch, roll)
            except ValueError:
                logger.error("Invalid IMU data format")
        else:
            logger.error("Invalid IMU data format")
            pass


    def _stabilize(self, yaw, pitch, roll):
        # 计算出欧拉角与水平平面的夹角，也就是云台要补偿安详转动的欧拉角
        yaw_compensation = -yaw
        pitch_compensation = -pitch
        roll_compensation = -roll

        # 根据补偿角计算出被增稳云台机构左右两侧电机的转动角度

        left_motor_radian , right_motor_radian = self._calculate_motor_angles(yaw_compensation, pitch_compensation, roll_compensation)

        # 控制电机发送控制指令
        self.motor_left.set_position(left_motor_radian)
        self.motor_right.set_position(right_motor_radian)
        pass

    def to_radians(self, degrees):
        """
        将角度转换为弧度，并限制在[-0.7, 0.7]范围内
        参数：角度值
        返回：限制在[-0.7, 0.7]范围内的弧度值
        """
        radians = degrees * 3.141592653589793 / 180.0
        # 限制弧度值范围
        if radians > 0.7:
            return 0.7
        elif radians < -0.7:
            return -0.7
        return radians

    def _calculate_motor_angles(self, yaw, pitch, roll):
        # 根据欧拉角计算左右两侧电机的转动角度 因为机械限位云台不能无限旋转，其运动范围是滚转和俯仰 不能超过正负0.7π弧度
        # 我们获得的欧拉角yaw, pitch, roll 都是角度，我们计算中要用角度最后返回时 要返回左电机和右电机的弧度
        left_motor_angle= 0
        right_motor_angle = 0
        left_motor_radian = self.to_radians(left_motor_angle)
        right_motor_radian = self.to_radians(right_motor_angle)
        return left_motor_radian, right_motor_radian

    pass

