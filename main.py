# main.py

import asyncio
import random
import math
import logging
from constants import Constants
from cyber_gear_motor import CyberGearMotor

# 设置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class GimbalController:
    """云台控制器类，负责控制电机和计算欧拉角"""

    # 定义云台的运行模式
    MODE_2DOF_RANDOM = '2dof_random'
    MODE_PITCH_RANDOM = 'pitch_random'
    MODE_PITCH_SINE = 'pitch_sine'
    MODE_ROLL = 'roll'
    MODE_ROLL_SINE = 'roll_sine'
    MODE_LEVEL_ROLL_SEEK_EAST = 'level_roll_seek_east'

    def __init__(self, motor_left: CyberGearMotor, motor_right: CyberGearMotor, mode='pitch_random'):
        self.motor_left = motor_left
        self.motor_right = motor_right
        self.running = True
        self.mode = mode  # 设置运行模式
        self.euler_angles = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.control_task = None
        self.print_task = None

    async def start(self):
        """启动云台控制器"""
        # 启动电机连接
        await asyncio.gather(
            self.motor_left.connect(),
            self.motor_right.connect()
        )
        await asyncio.sleep(0.5)
        # 重置圈数
        await asyncio.gather(
            self.motor_left.reset_rotation(),
            self.motor_right.reset_rotation()
        )
        await asyncio.sleep(0.1)

        # 设置电机参数 机械零位
        await asyncio.gather(
            self.moter_left.set_zero_position(),
            self.moter_right.set_zero_position()
        )
        await asyncio.sleep(0.1)

        # 启动控制和打印任务
        self.control_task = asyncio.create_task(self.control_loop())
        self.print_task = asyncio.create_task(self.print_euler_angles())
        # 等待运行状态变为 False
        try:
            while self.running:
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            pass

    async def control_loop(self):
        """控制循环，根据模式控制电机"""
        try:
            while self.running:
                if self.mode == self.MODE_2DOF_RANDOM:
                    # 1. 二自由度随机运动：随机改变俯仰和滚转角度
                    target_pitch = random.uniform(-0.6, 0.6)  # 弧度
                    target_roll = random.uniform(-0.6, 0.6)   # 弧度
                    motor_left_angle, motor_right_angle = self.calculate_motor_angles(target_pitch, target_roll)
                    await asyncio.gather(
                        self.motor_left.set_position(motor_left_angle),
                        self.motor_right.set_position(motor_right_angle)
                    )
                elif self.mode == self.MODE_PITCH_RANDOM:
                    # 2. 俯仰随机运动：随机改变俯仰角度，滚转角为0
                    target_pitch = random.uniform(-0.6, 0.6)  # 弧度
                    motor_left_angle, motor_right_angle = self.calculate_motor_angles(target_pitch, 0.0)
                    await asyncio.gather(
                        self.motor_left.set_position(motor_left_angle),
                        self.motor_right.set_position(motor_right_angle)
                    )
                elif self.mode == self.MODE_PITCH_SINE:
                    # 3. 俯仰正弦往复运动，滚转角为0
                    t = asyncio.get_event_loop().time()
                    target_pitch = 0.6 * math.sin(0.5 * t)  # 弧度
                    motor_left_angle, motor_right_angle = self.calculate_motor_angles(target_pitch, 0.0)
                    await asyncio.gather(
                        self.motor_left.set_position(motor_left_angle),
                        self.motor_right.set_position(motor_right_angle)
                    )
                    await asyncio.sleep(0.1)  # 调整睡眠时间以获得平滑运动
                    continue
                elif self.mode == self.MODE_ROLL:
                    # 4. 滚转运动：随机改变滚转角度，俯仰角为0
                    target_roll = random.uniform(-0.6, 0.6)  # 弧度
                    motor_left_angle, motor_right_angle = self.calculate_motor_angles(0.0, target_roll)
                    await asyncio.gather(
                        self.motor_left.set_position(motor_left_angle),
                        self.motor_right.set_position(motor_right_angle)
                    )
                elif self.mode == self.MODE_ROLL_SINE:
                    # 5. 滚转正弦运动，俯仰角为0
                    t = asyncio.get_event_loop().time()
                    target_roll = 0.6 * math.sin(0.5 * t)  # 弧度
                    motor_left_angle, motor_right_angle = self.calculate_motor_angles(0.0, target_roll)
                    await asyncio.gather(
                        self.motor_left.set_position(motor_left_angle),
                        self.motor_right.set_position(motor_right_angle)
                    )
                    await asyncio.sleep(0.1)
                    continue
                elif self.mode == self.MODE_LEVEL_ROLL_SEEK_EAST:
                    # 6. 水平、滚转正弦交替切换运动
                    t = asyncio.get_event_loop().time()
                    if int(t) % 10 < 5:
                        # 前5秒保持水平，俯仰角和滚转角均为0
                        motor_left_angle, motor_right_angle = self.calculate_motor_angles(0.0, 0.0)
                    else:
                        # 后5秒进行滚转正弦运动，俯仰角为0
                        target_roll = 0.6 * math.sin(0.5 * t)  # 弧度
                        motor_left_angle, motor_right_angle = self.calculate_motor_angles(0.0, target_roll)
                    await asyncio.gather(
                        self.motor_left.set_position(motor_left_angle),
                        self.motor_right.set_position(motor_right_angle)
                    )
                    await asyncio.sleep(0.1)
                    continue
                else:
                    # 默认模式，不做任何动作
                    await asyncio.sleep(0.5)
                    continue
                # 每次指令发送后等待一段时间
                await asyncio.sleep(0.5)
        except asyncio.CancelledError:
            pass

    async def print_euler_angles(self):
        """以1Hz的频率打印实时欧拉角"""
        try:
            while self.running:
                await asyncio.sleep(0.3)
                angle_left = self.motor_left.get_real_time_position()
                angle_right = self.motor_right.get_real_time_position()
                if angle_left is None or angle_right is None:
                    print("电机位置数据未就绪。")
                    continue
                self.calculate_euler_angles(angle_left, angle_right)
                print(f"[Euler Angles] Roll: {math.degrees(self.euler_angles['roll']):.2f}°, "
                      f"Pitch: {math.degrees(self.euler_angles['pitch']):.2f}°, "
                      f"Yaw: {math.degrees(self.euler_angles['yaw']):.2f}°")
        except asyncio.CancelledError:
            pass

    def calculate_motor_angles(self, pitch_angle: float, roll_angle: float) -> (float, float):
        """根据目标俯仰角和滚转角计算电机角度

        Args:
            pitch_angle (float): 目标俯仰角（单位：弧度）
            roll_angle (float): 目标滚转角（单位：弧度）

        Returns:
            (float, float): 左电机角度，右电机角度
        """
        # 根据机械结构计算电机角度
        # theta_left = pitch + roll
        # theta_right = -pitch + roll
        theta_left = pitch_angle + roll_angle
        theta_right = -pitch_angle + roll_angle
        return theta_left, theta_right

    def calculate_euler_angles(self, angle_left: float, angle_right: float):
        """根据电机角度计算云台的欧拉角

        Args:
            angle_left (float): 左电机角度（单位：弧度）
            angle_right (float): 右电机角度（单位：弧度）
        """
        # 根据机械结构，计算俯仰和滚转角度
        # pitch = (theta_left - theta_right) / 2
        # roll = (theta_left + theta_right) / 2
        self.euler_angles['pitch'] = (angle_left - angle_right) / 2
        self.euler_angles['roll'] = (angle_left + angle_right) / 2
        # 云台不具备偏航运动，设置为0
        self.euler_angles['yaw'] = 0.0

    async def stop(self):
        """停止云台控制器"""
        self.running = False
        # 取消任务
        if self.control_task:
            self.control_task.cancel()
            try:
                await self.control_task
            except asyncio.CancelledError:
                pass
        if self.print_task:
            self.print_task.cancel()
            try:
                await self.print_task
            except asyncio.CancelledError:
                pass
        # 紧急停止电机并关闭连接
        await asyncio.gather(
            self.motor_left.emergency_stop(),
            self.motor_right.emergency_stop()
        )
        await asyncio.gather(
            self.motor_left.close(),
            self.motor_right.close()
        )

def main():
    """主函数，创建云台控制器并运行"""
    # 创建电机实例，请确保 serial_port 设置为实际使用的串口端口
    motor_left = CyberGearMotor(
        can_id=127,  # 左电机的 CAN ID，请根据实际情况调整
        serial_port='COM5',  # 左电机的串口
        master_id=0x00FD,
        position_request_interval=1
    )
    motor_right = CyberGearMotor(
        can_id=127,  # 右电机的 CAN ID，请根据实际情况调整
        serial_port='COM4',  # 右电机的串口
        master_id=0x00FD,
        position_request_interval=1/30
    )

    # 提示用户选择模式
    print("请选择运行模式:")
    print("1. 二自由度随机运动")
    print("2. 俯仰随机运动")
    print("3. 俯仰正弦往复运动")
    print("4. 滚转运动")
    print("5. 滚转正弦运动")
    print("6. 水平、滚转正弦交替切换运动")
    mode_input = input("请输入模式编号 (1-6): ")

    mode_map = {
        '1': GimbalController.MODE_2DOF_RANDOM,
        '2': GimbalController.MODE_PITCH_RANDOM,
        '3': GimbalController.MODE_PITCH_SINE,
        '4': GimbalController.MODE_ROLL,
        '5': GimbalController.MODE_ROLL_SINE,
        '6': GimbalController.MODE_LEVEL_ROLL_SEEK_EAST
    }

    selected_mode = mode_map.get(mode_input, GimbalController.MODE_PITCH_RANDOM)

    gimbal_controller = GimbalController(motor_left, motor_right, mode=selected_mode)
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(gimbal_controller.start())
    except KeyboardInterrupt:
        print("程序被用户中断。")
        loop.run_until_complete(gimbal_controller.stop())
    except Exception as e:
        logger.error(f"程序运行时发生异常：{e}")
        loop.run_until_complete(gimbal_controller.stop())
    finally:
        loop.close()

if __name__ == "__main__":
    main()
