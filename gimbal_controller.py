# gimbal_controller.py

import asyncio
import random
import math
import logging
import datetime
from cyber_gear_motor import CyberGearMotor  # 确保此路径正确

# 设置日志
logger = logging.getLogger("GimbalController")
logger.setLevel(logging.DEBUG)  # 设置为DEBUG以获取详细日志
handler = logging.StreamHandler()
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

class GimbalController:
    """
    云台控制器类：
    - 模式1: 二自由度随机运动
    - 模式2: 俯仰随机运动
    - 模式3: 俯仰正弦往复运动
    - 模式4: 滚转运动
    - 模式5: 滚转正弦运动
    - 模式6: 水平、滚转正弦交替切换运动(平滑衔接)
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
        self.base_interval = 0.05  # 20Hz

    async def start(self):
        """启动云台控制器并初始化电机"""
        try:
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

            logger.info("GimbalController 已启动。")

            # 等待控制任务和打印任务完成（通常不会发生，除非被取消）
            await asyncio.gather(self.control_task, self.print_task)
        except asyncio.CancelledError:
            logger.info("GimbalController 启动任务被取消。")
        except Exception as e:
            logger.error(f"GimbalController 启动时发生错误: {e}")
            await self.stop()

    async def control_loop(self):
        """控制循环，根据模式调整云台姿态"""
        try:
            while self.running:
                if self.mode == self.MODE_2DOF_RANDOM:
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

                    φ = 0.64  # 相位

                    if tc < 5:
                        # 前5秒：pitch正弦, roll=0
                        pitch_final = 0.6 * math.sin(0.5 * tc)
                        roll_final = 0.0
                    elif 5 <= tc < 5.5:
                        # 5~5.5秒：pitch从峰值线性减为0，roll从峰值开始正弦
                        pitch_peak = 0.6 * math.sin(2.5)  # ≈0.359
                        fade_ratio = 1 - (tc - 5) / 0.5
                        pitch_final = pitch_peak * fade_ratio
                        roll_final = 0.6 * math.sin(0.5 * (tc - 5) + φ)
                    else:
                        # 5.5~10秒：pitch=0, roll继续正弦
                        pitch_final = 0.0
                        roll_final = 0.6 * math.sin(0.5 * (tc - 5) + φ)

                    await self.one_shot_set_positions(pitch_final, roll_final)
                    await asyncio.sleep(0.01)
                    continue
                else:
                    await asyncio.sleep(0.01)
                    continue
        except asyncio.CancelledError:
            logger.info("GimbalController 控制循环被取消。")
        except Exception as e:
            logger.error(f"控制循环发生错误: {e}")
            await asyncio.sleep(0.1)  # 防止异常导致循环退出

    async def one_shot_set_positions(self, pitch_final: float, roll_final: float):
        """
        一次性设置电机位置
        """
        L_roll = -roll_final / 2
        R_roll = -roll_final / 2
        L_pitch = pitch_final
        R_pitch = -pitch_final
        L_final = L_roll + L_pitch
        R_final = R_roll + R_pitch

        try:
            await asyncio.gather(
                self.motor_left.set_position(L_final, speed=self.default_speed),
                self.motor_right.set_position(R_final, speed=self.default_speed)
            )
            logger.debug(f"设置电机位置: Left={L_final:.2f} rad, Right={R_final:.2f} rad")
        except Exception as e:
            logger.error(f"设置电机位置时发生错误: {e}")

    async def print_euler_angles(self):
        """
        定期打印欧拉角
        """
        try:
            while self.running:
                await asyncio.sleep(0.05)  # 20Hz 打印频率，避免过高影响性能
                angle_left = self.motor_left.get_real_time_position()
                angle_right = self.motor_right.get_real_time_position()
                if angle_left is None or angle_right is None:
                    logger.warning("电机位置数据未就绪。")
                    continue

                self.calculate_euler_angles(angle_left, angle_right)

                roll_deg = math.degrees(self.euler_angles['roll'])
                pitch_deg = math.degrees(self.euler_angles['pitch'])
                yaw_deg = math.degrees(self.euler_angles['yaw'])

                logger.info(f"[Euler Angles] Timestamp: {datetime.datetime.now().timestamp():.6f}s | "
                            f"Yaw: {yaw_deg:.2f}° | Pitch: {pitch_deg:.2f}° | Roll: {roll_deg:.2f}°")
        except asyncio.CancelledError:
            logger.info("GimbalController 打印任务被取消。")
        except Exception as e:
            logger.error(f"打印欧拉角发生错误: {e}")
            await asyncio.sleep(0.1)  # 防止异常导致循环退出

    def calculate_euler_angles(self, L: float, R: float):
        """
        计算欧拉角
        """
        euler_angles = self.compute_final(L, R)
        self.euler_angles['roll'] = euler_angles[0]
        self.euler_angles['pitch'] = euler_angles[1]
        self.euler_angles['yaw'] = 0.0  # Yaw 由 IMU 提供

    def compute_final(self, L_final: float, R_final: float):
        """
        计算最终的 Roll, Pitch, Yaw
        """
        roll_final = -(L_final + R_final)
        pitch_final = (L_final - R_final) / 2.0
        return roll_final, pitch_final, 0.0

    async def stop(self):
        """
        停止云台控制器，确保电机停止
        """
        self.running = False
        if self.control_task:
            self.control_task.cancel()
            try:
                await self.control_task
            except asyncio.CancelledError:
                logger.info("GimbalController 控制任务已取消。")
        if self.print_task:
            self.print_task.cancel()
            try:
                await self.print_task
            except asyncio.CancelledError:
                logger.info("GimbalController 打印任务已取消。")
        try:
            await asyncio.gather(
                self.motor_left.emergency_stop(),
                self.motor_right.emergency_stop()
            )
            logger.info("已发送急停命令给所有电机。")
        except Exception as e:
            logger.error(f"发送急停命令时发生错误: {e}")

        try:
            await asyncio.gather(
                self.motor_left.close(),
                self.motor_right.close()
            )
            logger.info("已关闭所有电机连接。")
        except Exception as e:
            logger.error(f"关闭电机连接时发生错误: {e}")
        logger.info("GimbalController 已停止。")

# 为测试和调试添加的 main 函数
async def test_gimbal_controller():
    """
    测试GimbalController:
    1. 初始化电机
    2. 启动控制器
    3. 运行一段时间后停止
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

    try:
        # 运行10秒后停止
        await asyncio.sleep(10)
    except KeyboardInterrupt:
        logger.info("测试程序被用户中断。")
    except Exception as e:
        logger.error(f"程序运行时发生异常：{e}")
    finally:
        await gimbal_controller.stop()
        # 确保云台控制器停止
        controller_task.cancel()
        try:
            await controller_task
        except asyncio.CancelledError:
            logger.info("GimbalController 任务已取消。")

if __name__ == "__main__":
    asyncio.run(test_gimbal_controller())
