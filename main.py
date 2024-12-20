# main.py

import asyncio
import logging
import math
import datetime
from imu_reader import IMUReader
from gimbal_angle_reader import GimbalAngleReader
from gimbal_controller import GimbalController
from cyber_gear_motor import CyberGearMotor

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger("Main")

class DataSynchronizer:
    """
    数据同步类：
    同步后生成包含9列的数据文本文件（.txt）：
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
        imu_data = self.imu_reader.get_buffer_data()
        gimbal_data = self.gimbal_reader.get_history_data()

        if not imu_data or not gimbal_data:
            logger.warning("IMU或云台数据为空，无法进行同步。")
            return

        gimbal_data_sorted = sorted(gimbal_data, key=lambda x: x[0])

        # 使用较大的time_window提高匹配成功率
        time_window = 0.1

        # IMU的时间范围：在前后各2秒的余量中间选出record_duration
        # imu_data最早时间
        imu_min_time = min(d['timestamp'] for d in imu_data)
        imu_start_time = imu_min_time + 2  # 前2秒作为预热，不计入正式记录
        imu_end_time = imu_start_time + record_duration

        matched_count = 0

        with open(self.output_file, "a", encoding='utf-8') as f:
            for imu in imu_data:
                imu_timestamp = imu['timestamp']
                if imu_timestamp < imu_start_time or imu_timestamp > imu_end_time:
                    continue

                candidates = [g for g in gimbal_data_sorted if abs(g[0] - imu_timestamp) <= time_window]
                if not candidates:
                    continue
                best_match = min(candidates, key=lambda x: abs(x[0] - imu_timestamp))
                g_ts, g_yaw, g_pitch, g_roll = best_match

                imu_roll = imu['roll']
                imu_pitch = imu['pitch']
                imu_yaw = imu['yaw']

                gimbal_roll = g_roll
                gimbal_pitch = g_pitch
                gimbal_yaw = g_yaw

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

    while True:
        try:
            record_duration = float(input("请输入记录持续时间（秒）: "))
            if record_duration <= 0:
                print("记录时间必须为正数。")
                continue
            break
        except ValueError:
            print("请输入一个有效的数字。")

    # 增加更多余量，总共多4秒（前2秒预热 + 后2秒缓存）
    total_duration = record_duration + 4

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
    # gimbal: 45Hz * (record_duration+4)
    gimbal_records = int(45 * total_duration)
    # imu: 450Hz * (record_duration+4)
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

    # 开始云台记录器
    await gimbal_angle_reader.start()
    logger.info(f"开始记录总时长 {total_duration} 秒的数据（含前后余量）...")

    # 等待total_duration秒
    await asyncio.sleep(total_duration)

    # 停止IMU和云台记录
    await imu_reader.stop()
    await gimbal_angle_reader.stop()

    # 停止云台控制器
    controller_task.cancel()
    try:
        await controller_task
    except asyncio.CancelledError:
        pass

    logger.info("数据记录完成，开始同步处理...")

    synchronizer = DataSynchronizer(imu_reader, gimbal_angle_reader, "synchronized_output.txt")
    synchronizer.synchronize_and_write(record_duration)

    logger.info("程序已结束。")

if __name__ == "__main__":
    asyncio.run(main())
