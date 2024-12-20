# main.py

import asyncio
import logging
import math
from imu_reader import IMUReader
from gimbal_angle_reader import GimbalAngleReader
from gimbal_controller import GimbalController
from cyber_gear_motor import CyberGearMotor

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger("Main")

class DataSynchronizer:
    """
    数据同步类：
    同步后生成包含10列的数据文本文件（.txt）：
    Timestamp IMU_Roll IMU_Pitch IMU_Yaw Gimbal_Roll Gimbal_Pitch Gimbal_Yaw Diff_Roll Diff_Pitch Diff_Yaw
    """

    def __init__(self, imu_reader: IMUReader, gimbal_reader: GimbalAngleReader, output_file: str = "synchronized_output.txt"):
        self.imu_reader = imu_reader
        self.gimbal_reader = gimbal_reader
        self.output_file = output_file
        self._initialize_txt()

    def _initialize_txt(self):
        with open(self.output_file, "w", encoding='utf-8') as f:
            header = "Timestamp(s) IMU_Roll IMU_Pitch IMU_Yaw Gimbal_Roll Gimbal_Pitch Gimbal_Yaw Diff_Roll Diff_Pitch Diff_Yaw\n"
            f.write(header)

    def synchronize_and_write(self, record_duration: float):
        """
        同步IMU和云台的数据，并写入文本文件（.txt）。
        :param record_duration: 用户输入的记录时长（秒，不包含前后缓冲）
        """
        imu_data = self.imu_reader.get_buffer_data()
        gimbal_data = self.gimbal_reader.get_history_data()

        if not imu_data or not gimbal_data:
            logger.warning("IMU或云台数据为空，无法进行同步。")
            return

        gimbal_data_sorted = sorted(gimbal_data, key=lambda x: x[0])

        # 使用较大的time_window以适应系统时间差异
        time_window = 0.015  # 15ms

        # IMU的时间范围：在前后各7秒的余量中间选出record_duration
        imu_min_time = min(d['timestamp'] for d in imu_data)
        imu_start_time = imu_min_time + 7  # 前7秒作为预热，不计入正式记录
        imu_end_time = imu_start_time + record_duration

        matched_count = 0

        with open(self.output_file, "a", encoding='utf-8') as f:
            for imu in imu_data:
                imu_timestamp = imu['timestamp']
                if imu_timestamp < imu_start_time or imu_timestamp > imu_end_time:
                    continue

                # 查找时间差在time_window内的gimbal数据
                candidates = [g for g in gimbal_data_sorted if abs(g[0] - imu_timestamp) <= time_window]
                if not candidates:
                    logger.debug(f"IMU时间戳 {imu_timestamp:.6f} 没有匹配的云台数据。")
                    continue
                # 选择时间差最小的gimbal数据
                best_match = min(candidates, key=lambda x: abs(x[0] - imu_timestamp))
                g_ts, g_yaw, g_pitch, g_roll = best_match

                imu_roll = imu['roll']
                imu_pitch = imu['pitch']
                imu_yaw = imu['yaw']

                gimbal_roll = g_roll
                gimbal_pitch = g_pitch
                gimbal_yaw = g_yaw

                # 差值
                diff_roll = imu_roll - gimbal_roll
                diff_pitch = imu_pitch - gimbal_pitch
                diff_yaw = imu_yaw - gimbal_yaw

                line = f"{imu_timestamp:.6f} {imu_roll:.6f} {imu_pitch:.6f} {imu_yaw:.6f} {gimbal_roll:.6f} {gimbal_pitch:.6f} {gimbal_yaw:.6f} {diff_roll:.6f} {diff_pitch:.6f} {diff_yaw:.6f}\n"
                f.write(line)
                matched_count += 1

        logger.info(f"同步数据已写入文件: {self.output_file}，共匹配到 {matched_count} 条数据。")

        required_count = int(record_duration * 450)
        if matched_count < required_count:
            logger.warning(f"匹配数据行数 {matched_count} 少于期望的 {required_count} 行。建议延长记录时间或增加前后缓冲。")

async def main():
    """
    主函数：
    1. 用户选择运动模式
    2. 用户输入记录持续时间
    3. 初始化电机和控制器
    4. 启动预热阶段
    5. 启动正式记录阶段
    6. 等待记录完成
    7. 停止所有任务
    8. 同步数据并输出
    9. 安全退出
    """

    # 用户选择运行模式
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
    logger.info(f"选择的运动模式: {selected_mode}")

    # 用户输入记录持续时间（秒）
    while True:
        try:
            record_duration = float(input("请输入记录持续时间（秒）: "))
            if record_duration <= 0:
                print("记录时间必须为正数。")
                continue
            break
        except ValueError:
            print("请输入一个有效的数字。")

    # 增加更多余量，总共多14秒（前7秒预热 + 后7秒缓冲）
    preheat_time = 7
    buffer_time = 7
    total_duration = record_duration + preheat_time + buffer_time

    logger.info(f"总记录时长为 {total_duration} 秒（包含前 {preheat_time} 秒预热和后 {buffer_time} 秒缓冲）。")

    # 初始化电机，都设为1/45
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

    gimbal_controller = GimbalController(motor_left, motor_right, mode=selected_mode)

    # 根据total_duration计算需要的行数
    # gimbal: 45Hz * total_duration
    gimbal_records = int(45 * total_duration)
    # imu: 450Hz * total_duration
    imu_records = int(450 * total_duration)

    gimbal_angle_reader = GimbalAngleReader(
        gimbal_controller=gimbal_controller,
        max_records=gimbal_records,
        output_file="gimbal_angles.txt"
    )

    imu_port = 'COM7'
    imu_baudrate = 460800
    imu_output_file = "imu_data.txt"

    imu_reader = IMUReader(
        port=imu_port,
        baudrate=imu_baudrate,
        output_file=imu_output_file,
        max_lines=imu_records,
        buffer_size=imu_records * 2,
        auto_reset=True,
        reset_delay=0.1
    )

    # 启动云台控制器
    controller_task = asyncio.create_task(gimbal_controller.start())

    try:
        await imu_reader.connect()
    except Exception as e:
        logger.error(f"连接IMU失败: {e}")
        controller_task.cancel()
        try:
            await controller_task
        except asyncio.CancelledError:
            pass
        return

    try:
        # 启动数据记录
        await gimbal_angle_reader.start()
        await asyncio.sleep(preheat_time)

        logger.info(f"开始正式记录阶段，持续 {record_duration} 秒...")
        await asyncio.sleep(record_duration)

        logger.info(f"开始缓冲阶段，持续 {buffer_time} 秒...")
        await asyncio.sleep(buffer_time)
    except Exception as e:
        logger.error(f"运行过程中发生异常: {e}")
    finally:
        # 停止记录
        await imu_reader.stop()
        await gimbal_angle_reader.stop()

        # 停止云台控制器
        controller_task.cancel()
        try:
            await controller_task
        except asyncio.CancelledError:
            pass

        logger.info("数据记录完成，开始同步处理...")

        # 同步数据并输出10列数据文件
        synchronizer = DataSynchronizer(imu_reader, gimbal_angle_reader, "synchronized_output.txt")
        synchronizer.synchronize_and_write(record_duration)

        logger.info("程序已结束。")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        logger.error(f"程序运行时发生未捕获的异常: {e}")
