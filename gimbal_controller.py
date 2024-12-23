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
    云台控制器类，支持以下模式：
      - MODE_2DOF_RANDOM: 二自由度随机
      - MODE_PITCH_RANDOM: 俯仰随机
      - MODE_PITCH_SINE: 俯仰正弦
      - MODE_ROLL: 滚转随机
      - MODE_ROLL_SINE: 滚转正弦
      - MODE_LEVEL_ROLL_SEEK_EAST: 水平、滚转正弦交替切换
      - MODE_LEFT_DIAGONAL_SINE: 左对角线正弦（新增）
      - MODE_RIGHT_DIAGONAL_SINE: 右对角线正弦（新增）

    优化点：
      1. 降低随机/正弦模式的指令发送频率，减少串口负载。
      2. 在 one_shot_set_positions 做“最小发送间隔”限制，避免短时间内多次发指令。
      3. 新增 left_diagonal_sine / right_diagonal_sine 两种模式。
    """

    MODE_2DOF_RANDOM        = '2dof_random'
    MODE_PITCH_RANDOM       = 'pitch_random'
    MODE_PITCH_SINE         = 'pitch_sine'
    MODE_ROLL               = 'roll'
    MODE_ROLL_SINE          = 'roll_sine'
    MODE_LEVEL_ROLL_SEEK_EAST = 'level_roll_seek_east'

    # 新增：
    MODE_LEFT_DIAGONAL_SINE  = 'left_diagonal_sine'
    MODE_RIGHT_DIAGONAL_SINE = 'right_diagonal_sine'

    def __init__(self, motor_left: CyberGearMotor, motor_right: CyberGearMotor, mode='pitch_random'):
        self.motor_left = motor_left
        self.motor_right = motor_right
        self.running = True
        self.mode = mode

        # 当前欧拉角（仅供打印展示）
        self.euler_angles = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}

        self.control_task = None
        self.print_task = None

        # 默认电机运动速度(弧度/秒)
        self.default_speed = 5.0

        # 内部循环的基本周期
        self.base_interval = 0.05  # 20Hz

        # 发送节流：最小发送间隔
        self._last_send_time = 0.0
        self._min_send_interval = 0.05  # 20Hz

    async def start(self):
        """启动云台控制器并初始化电机"""
        try:
            # 并发连接左右电机
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
            self.print_task   = asyncio.create_task(self.print_euler_angles())

            logger.info("GimbalController 已启动。")

            # 等待两个任务结束（通常不会发生，除非被cancel或异常）
            await asyncio.gather(self.control_task, self.print_task)
        except asyncio.CancelledError:
            logger.info("GimbalController 启动任务被取消。")
        except Exception as e:
            logger.error(f"GimbalController 启动时发生错误: {e}")
            await self.stop()

    async def control_loop(self):
        """模式控制循环，根据 self.mode 执行不同动作，并控制发送频率。"""
        try:
            while self.running:
                if self.mode == self.MODE_2DOF_RANDOM:
                    # 二自由度随机
                    pitch_final = random.uniform(-0.6, 0.6)
                    roll_final  = random.uniform(-0.6, 0.6)
                    await self.one_shot_set_positions(pitch_final, roll_final)
                    await asyncio.sleep(1.0)  # 加大间隔，减少负载
                    continue

                elif self.mode == self.MODE_PITCH_RANDOM:
                    # 俯仰随机
                    pitch_final = random.uniform(-0.6, 0.6)
                    roll_final  = 0.0
                    await self.one_shot_set_positions(pitch_final, roll_final)
                    await asyncio.sleep(0.8)
                    continue

                elif self.mode == self.MODE_PITCH_SINE:
                    # 俯仰正弦
                    t = asyncio.get_event_loop().time()
                    pitch_final = 0.6 * math.sin(0.5 * t)
                    roll_final  = 0.0
                    await self.one_shot_set_positions(pitch_final, roll_final)
                    await asyncio.sleep(0.05)
                    continue

                elif self.mode == self.MODE_ROLL:
                    # 滚转随机
                    pitch_final = 0.0
                    roll_final  = random.uniform(-0.6, 0.6)
                    await self.one_shot_set_positions(pitch_final, roll_final)
                    await asyncio.sleep(0.8)
                    continue

                elif self.mode == self.MODE_ROLL_SINE:
                    # 滚转正弦
                    t = asyncio.get_event_loop().time()
                    pitch_final = 0.0
                    roll_final  = 0.6 * math.sin(0.5 * t)
                    await self.one_shot_set_positions(pitch_final, roll_final)
                    await asyncio.sleep(0.05)
                    continue

                elif self.mode == self.MODE_LEVEL_ROLL_SEEK_EAST:
                    # 水平、滚转正弦交替
                    t = asyncio.get_event_loop().time()
                    tc = t % 10.0
                    φ = 0.64
                    if tc < 5:
                        pitch_final = 0.6 * math.sin(0.5 * tc)
                        roll_final  = 0.0
                    elif 5 <= tc < 5.5:
                        pitch_peak  = 0.6 * math.sin(2.5)  # ~0.359
                        fade_ratio  = 1 - (tc - 5)/0.5
                        pitch_final = pitch_peak * fade_ratio
                        roll_final  = 0.6 * math.sin(0.5*(tc-5) + φ)
                    else:
                        pitch_final = 0.0
                        roll_final  = 0.6 * math.sin(0.5*(tc-5) + φ)

                    await self.one_shot_set_positions(pitch_final, roll_final)
                    await asyncio.sleep(0.05)
                    continue

                elif self.mode == self.MODE_LEFT_DIAGONAL_SINE:
                    # 7. 左对角线正弦
                    # 思路：让左电机做sin运动，右电机保持0
                    # => Lpos = sin, Rpos=0
                    t = asyncio.get_event_loop().time()
                    left_pos = 0.6 * math.sin(0.5 * t)
                    right_pos= 0.0
                    await self.one_shot_set_positions_LR(left_pos, right_pos)
                    await asyncio.sleep(0.05)
                    continue

                elif self.mode == self.MODE_RIGHT_DIAGONAL_SINE:
                    # 8. 右对角线正弦
                    # 思路：让右电机做sin运动，左电机保持0
                    t = asyncio.get_event_loop().time()
                    left_pos = 0.0
                    right_pos= 0.6 * math.sin(0.5 * t)
                    await self.one_shot_set_positions_LR(left_pos, right_pos)
                    await asyncio.sleep(0.05)
                    continue

                else:
                    # 默认空转
                    await asyncio.sleep(self.base_interval)
                    continue

        except asyncio.CancelledError:
            logger.info("GimbalController 控制循环被取消。")
        except Exception as e:
            logger.error(f"控制循环发生错误: {e}")
            await asyncio.sleep(0.1)

    async def one_shot_set_positions(self, pitch_final: float, roll_final: float):
        """
        原模式：通过 pitch, roll 计算左右电机的目标位置(L_final, R_final)，再set_position。
        新增最小发送间隔来做节流。
        """
        now = asyncio.get_event_loop().time()
        # 若距离上次发送不足 _min_send_interval，则等待
        elapsed = now - self._last_send_time
        if elapsed < self._min_send_interval:
            await asyncio.sleep(self._min_send_interval - elapsed)

        L_roll = -roll_final / 2
        R_roll = -roll_final / 2
        L_pitch= pitch_final
        R_pitch= -pitch_final
        L_final= L_roll + L_pitch
        R_final= R_roll + R_pitch

        try:
            await asyncio.gather(
                self.motor_left.set_position(L_final, speed=self.default_speed),
                self.motor_right.set_position(R_final, speed=self.default_speed)
            )
            self._last_send_time = asyncio.get_event_loop().time()
            logger.debug(f"set_positions: L={L_final:.2f}, R={R_final:.2f}")
        except Exception as e:
            logger.error(f"set_positions错误: {e}")

    async def one_shot_set_positions_LR(self, left_pos: float, right_pos: float):
        """
        新增：专门给“对角线正弦”模式用的直接设左/右电机位置。
        同样做最小发送间隔节流。
        """
        now = asyncio.get_event_loop().time()
        elapsed = now - self._last_send_time
        if elapsed < self._min_send_interval:
            await asyncio.sleep(self._min_send_interval - elapsed)

        try:
            await asyncio.gather(
                self.motor_left.set_position(left_pos, speed=self.default_speed),
                self.motor_right.set_position(right_pos, speed=self.default_speed)
            )
            self._last_send_time = asyncio.get_event_loop().time()
            logger.debug(f"set_positions_LR: L={left_pos:.2f}, R={right_pos:.2f}")
        except Exception as e:
            logger.error(f"set_positions_LR错误: {e}")

    async def print_euler_angles(self):
        """定期打印欧拉角"""
        try:
            while self.running:
                await asyncio.sleep(0.05)  # 20Hz
                angle_left  = self.motor_left.get_real_time_position()
                angle_right = self.motor_right.get_real_time_position()
                if angle_left is None or angle_right is None:
                    logger.warning("电机位置数据未就绪。")
                    continue

                self.calculate_euler_angles(angle_left, angle_right)

                roll_deg  = math.degrees(self.euler_angles['roll'])
                pitch_deg = math.degrees(self.euler_angles['pitch'])
                yaw_deg   = math.degrees(self.euler_angles['yaw'])

                logger.info(f"[Euler] Time={datetime.datetime.now().timestamp():.3f}s "
                            f"Roll={roll_deg:.2f}, Pitch={pitch_deg:.2f}, Yaw={yaw_deg:.2f}")
        except asyncio.CancelledError:
            logger.info("GimbalController 打印任务被取消。")
        except Exception as e:
            logger.error(f"打印欧拉角时出错: {e}")
            await asyncio.sleep(0.1)

    def calculate_euler_angles(self, L: float, R: float):
        """计算欧拉角(roll,pitch,yaw)"""
        roll, pitch, yaw = self.compute_final(L, R)
        self.euler_angles['roll']  = roll
        self.euler_angles['pitch'] = pitch
        self.euler_angles['yaw']   = yaw

    def compute_final(self, L_final: float, R_final: float):
        """根据电机左右目标位置(弧度)，推算roll,pitch,yaw(弧度)。"""
        roll_final  = -(L_final + R_final)
        pitch_final = (L_final - R_final) / 2.0
        return roll_final, pitch_final, 0.0

    async def stop(self):
        """停止云台控制器、停止电机"""
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


# ----------------- 测试main函数，可保留也可单独移除 ------------------ #
async def test_gimbal_controller():
    from cyber_gear_motor import CyberGearMotor
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

    gimbal = GimbalController(motor_left, motor_right, mode=GimbalController.MODE_LEFT_DIAGONAL_SINE)
    controller_task = asyncio.create_task(gimbal.start())

    try:
        await asyncio.sleep(10)  # 运行10秒
    finally:
        await gimbal.stop()
        controller_task.cancel()
        try:
            await controller_task
        except asyncio.CancelledError:
            pass

if __name__ == "__main__":
    asyncio.run(test_gimbal_controller())
