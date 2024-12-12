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
    """
    云台控制器类：
    - 模式1增加等待时间避免抖动。
    - 模式6通过一个10秒周期设计，实现俯仰正弦与滚转正弦的平滑过渡，无跳变。
    """
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
        self.mode = mode
        self.euler_angles = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.control_task = None
        self.print_task = None

        # 提高速度减少滞后
        self.default_speed = 5.0
        # 基本更新间隔
        self.base_interval = 0.05

    async def start(self):
        """启动云台控制器并初始化电机"""
        await asyncio.gather(
            self.motor_left.connect(),
            self.motor_right.connect()
        )
        await asyncio.sleep(0.01)

        # 重置圈数
        await asyncio.gather(
            self.motor_left.reset_rotation(),
            self.motor_right.reset_rotation()
        )
        await asyncio.sleep(0.01)

        # 设置零位
        await asyncio.gather(
            self.motor_left.set_zero_position(),
            self.motor_right.set_zero_position()
        )
        await asyncio.sleep(0.01)

        self.control_task = asyncio.create_task(self.control_loop())
        self.print_task = asyncio.create_task(self.print_euler_angles())

        try:
            while self.running:
                await asyncio.sleep(self.base_interval)
        except asyncio.CancelledError:
            pass

    async def control_loop(self):
        """
        对模式6进行特殊处理：
        定义一个10s周期：
        0~5s: pitch正弦，roll=0
        5s时pitch峰值约0.359，此时roll从同样的角度开始正弦，pitch不立刻变0，而是在5~5.5s间线性淡出到0，
        roll正弦自然进行，不产生跳变
        5.5~10s：仅roll正弦运动，到10s roll回归0，下个周期再从pitch开始
        """
        try:
            while self.running:
                if self.mode == self.MODE_2DOF_RANDOM:
                    # 等待时间加长，给电机完成动作余裕，减少抖动
                    pitch_final = random.uniform(-0.6, 0.6)
                    roll_final = random.uniform(-0.6, 0.6)
                    await self.one_shot_set_positions(pitch_final, roll_final)
                    await asyncio.sleep(0.3)
                    continue

                elif self.mode == self.MODE_PITCH_RANDOM:
                    pitch_final = random.uniform(-0.6, 0.6)
                    roll_final = 0.0
                    await self.one_shot_set_positions(pitch_final, roll_final)
                    await asyncio.sleep(0.3)
                    continue

                elif self.mode == self.MODE_PITCH_SINE:
                    t = asyncio.get_event_loop().time()
                    pitch_final = 0.6 * math.sin(0.5 * t)
                    roll_final = 0.0
                    await self.one_shot_set_positions(pitch_final, roll_final)
                    await asyncio.sleep(0.01)
                    continue

                elif self.mode == self.MODE_ROLL:
                    pitch_final = 0.0
                    roll_final = random.uniform(-0.6, 0.6)
                    await self.one_shot_set_positions(pitch_final, roll_final)
                    await asyncio.sleep(0.3)
                    continue

                elif self.mode == self.MODE_ROLL_SINE:
                    t = asyncio.get_event_loop().time()
                    pitch_final = 0.0
                    roll_final = 0.6 * math.sin(0.5 * t)
                    await self.one_shot_set_positions(pitch_final, roll_final)
                    await asyncio.sleep(0.01)
                    continue

                elif self.mode == self.MODE_LEVEL_ROLL_SEEK_EAST:
                    # 10s周期，分成3段
                    t = asyncio.get_event_loop().time()
                    tc = t % 10.0  # 周期内时间

                    # 在t=5秒时pitch达到峰值0.359左右，我们为roll引入相位来匹配此初始值。
                    # 计算φ满足roll(5s)=pitch(5s)
                    # pitch(5s)=0.6*sin(0.5*5)=0.6*sin(2.5)≈0.359
                    # roll(t)=0.6*sin(0.5*(t-5)+φ)，要求roll(5)=0.359
                    # =>0.359=0.6*sin(φ) => φ=arcsin(0.359/0.6)≈0.64弧度
                    φ = 0.64

                    if tc < 5:
                        # 前5秒：pitch正弦, roll=0
                        pitch_final = 0.6 * math.sin(0.5 * tc)
                        roll_final = 0.0
                    elif 5 <= tc < 5.5:
                        # 5~5.5秒：pitch从0.359线性减为0, roll从0.359继续正弦
                        # pitch(5)=0.359
                        # 在0.5秒内线性衰减到0：pitch_final=0.359*(1 - (tc-5)/0.5)
                        pitch_peak = 0.6 * math.sin(2.5) # ≈0.359
                        fade_ratio = 1 - (tc - 5)/0.5
                        pitch_final = pitch_peak * fade_ratio

                        # roll从0.359平滑继续
                        roll_final = 0.6 * math.sin(0.5*(tc - 5) + φ)
                    else:
                        # 5.5~10秒：pitch=0, roll继续正弦直到回到0
                        pitch_final = 0.0
                        roll_final = 0.6 * math.sin(0.5*(tc - 5) + φ)

                    await self.one_shot_set_positions(pitch_final, roll_final)
                    await asyncio.sleep(0.01)
                    continue
                else:
                    await asyncio.sleep(0.01)
                    continue
        except asyncio.CancelledError:
            pass

    async def one_shot_set_positions(self, pitch_final: float, roll_final: float):
        # 与此前逻辑相同，只是加了一个较大的速度参数
        L_roll = -roll_final / 2
        R_roll = -roll_final / 2
        L_pitch = pitch_final
        R_pitch = -pitch_final
        L_final = L_roll + L_pitch
        R_final = R_roll + R_pitch

        await asyncio.gather(
            self.motor_left.set_position(L_final, speed=self.default_speed),
            self.motor_right.set_position(R_final, speed=self.default_speed)
        )

    async def print_euler_angles(self):
        # 打印间隔略小一些，观测更流畅
        try:
            while self.running:
                await asyncio.sleep(0.05)
                angle_left = self.motor_left.get_real_time_position()
                angle_right = self.motor_right.get_real_time_position()
                if angle_left is None or angle_right is None:
                    print("电机位置数据未就绪。")
                    continue

                self.calculate_euler_angles(angle_left, angle_right)

                roll_deg = math.degrees(self.euler_angles['roll'])
                pitch_deg = math.degrees(self.euler_angles['pitch'])
                yaw_deg = math.degrees(self.euler_angles['yaw'])

                print(f"[Euler Angles] Roll: {roll_deg:.2f}°, "
                      f"Pitch: {pitch_deg:.2f}°, "
                      f"Yaw: {yaw_deg:.2f}°")
        except asyncio.CancelledError:
            pass

    def calculate_euler_angles(self, L: float, R: float):
        euler_angles = self.compute_final(L, R)
        self.euler_angles['roll'] = euler_angles[0]
        self.euler_angles['pitch'] = euler_angles[1]
        self.euler_angles['yaw'] = 0.0

    def compute_final(self, L_final: float, R_final: float):
        roll_final = -(L_final + R_final)
        pitch_final = (L_final - R_final) / 2.0
        return roll_final, pitch_final, 0.0

    async def stop(self):
        self.running = False
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
        await asyncio.gather(
            self.motor_left.emergency_stop(),
            self.motor_right.emergency_stop()
        )
        await asyncio.gather(
            self.motor_left.close(),
            self.motor_right.close()
        )

def main():
    motor_left = CyberGearMotor(
        can_id=127,
        serial_port='COM5',
        master_id=0x00FD,
        position_request_interval=1
    )
    motor_right = CyberGearMotor(
        can_id=127,
        serial_port='COM4',
        master_id=0x00FD,
        position_request_interval=1/30
    )

    print("请选择运行模式:")
    print("1. 二自由度随机运动")
    print("2. 俯仰随机运动")
    print("3. 俯仰正弦往复运动")
    print("4. 滚转运动")
    print("5. 滚转正弦运动")
    print("6. 水平、滚转正弦交替切换运动(平滑衔接)")
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
