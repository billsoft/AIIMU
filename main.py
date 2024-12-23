import asyncio
import logging
from imu_reader import IMUReader
from gimbal_angle_reader import GimbalAngleReader
from gimbal_controller import GimbalController
from cyber_gear_motor import CyberGearMotor

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger("Main")

class DataSynchronizer:
    """
    同步后生成包含10列的数据文本文件:
    Timestamp IMU_Roll IMU_Pitch IMU_Yaw Gimbal_Roll Gimbal_Pitch Gimbal_Yaw Diff_Roll Diff_Pitch Diff_Yaw
    """
    def __init__(self, imu_reader: IMUReader, gimbal_reader: GimbalAngleReader,
                 output_file: str = "synchronized_output.txt"):
        self.imu_reader = imu_reader
        self.gimbal_reader = gimbal_reader
        self.output_file = output_file
        with open(self.output_file, "w", encoding='utf-8') as f:
            header = ("Timestamp(s) IMU_Roll IMU_Pitch IMU_Yaw "
                      "Gimbal_Roll Gimbal_Pitch Gimbal_Yaw "
                      "Diff_Roll Diff_Pitch Diff_Yaw\n")
            f.write(header)

    def synchronize_and_write(self, record_duration: float):
        imu_data = self.imu_reader.get_buffer_data()        # list[dict], each dict: {timestamp,yaw,pitch,roll}
        gimbal_data = self.gimbal_reader.get_history_data() # list[dict], each dict: {timestamp,yaw,pitch,roll}

        if not imu_data or not gimbal_data:
            logger.warning("IMU或云台数据为空, 无法同步")
            return

        # 按 timestamp 排序云台数据
        gimbal_data_sorted = sorted(gimbal_data, key=lambda x: x['timestamp'])
        time_window = 0.015

        # IMU时间:前7秒预热+record_duration
        imu_min_time = min(d['timestamp'] for d in imu_data)
        imu_start_time = imu_min_time + 7
        imu_end_time = imu_start_time + record_duration

        matched_count = 0
        with open(self.output_file, "a", encoding='utf-8') as f:
            for imu in imu_data:
                ts_imu = imu['timestamp']
                if ts_imu < imu_start_time or ts_imu > imu_end_time:
                    continue
                # 筛选time_window内的gimbal
                candidates = [
                    g for g in gimbal_data_sorted
                    if abs(g['timestamp'] - ts_imu) <= time_window
                ]
                if not candidates:
                    continue
                best = min(candidates,
                           key=lambda x: abs(x['timestamp'] - ts_imu))

                # imu
                i_roll = imu['roll']
                i_pitch = imu['pitch']
                i_yaw = imu['yaw']
                # gimbal
                g_roll = best['roll']
                g_pitch = best['pitch']
                g_yaw = best['yaw']

                diff_roll = i_roll - g_roll
                diff_pitch = i_pitch - g_pitch
                diff_yaw = i_yaw - g_yaw

                line = (
                    f"{ts_imu:.6f} "
                    f"{i_roll:.6f} {i_pitch:.6f} {i_yaw:.6f} "
                    f"{g_roll:.6f} {g_pitch:.6f} {g_yaw:.6f} "
                    f"{diff_roll:.6f} {diff_pitch:.6f} {diff_yaw:.6f}\n"
                )
                f.write(line)
                matched_count += 1

        logger.info(f"同步数据已写入: {self.output_file}, 匹配到{matched_count}条.")

        required = int(record_duration * 450)
        if matched_count < required:
            logger.warning(
                f"匹配数 {matched_count} < 期望 {required}, "
                f"可延长记录或增大缓冲"
            )

async def main():
    """主函数: 1.选模式 2.输入时长 3.预热->记录->缓冲->stop->同步"""
    print("请选择运行模式:")
    print("1. 二自由度随机运动")
    print("2. 俯仰随机运动")
    print("3. 俯仰正弦往复运动")
    print("4. 滚转运动")
    print("5. 滚转正弦运动")
    print("6. 水平、滚转正弦交替切换运动(平滑衔接)")

    mode_input = input("请输入模式编号(1-6): ")
    mode_map = {
        '1': GimbalController.MODE_2DOF_RANDOM,
        '2': GimbalController.MODE_PITCH_RANDOM,
        '3': GimbalController.MODE_PITCH_SINE,
        '4': GimbalController.MODE_ROLL,
        '5': GimbalController.MODE_ROLL_SINE,
        '6': GimbalController.MODE_LEVEL_ROLL_SEEK_EAST
    }
    selected_mode = mode_map.get(mode_input, GimbalController.MODE_PITCH_RANDOM)
    logger.info(f"选择的模式: {selected_mode}")

    while True:
        try:
            record_duration = float(input("请输入记录持续时间(秒): "))
            if record_duration<=0:
                continue
            break
        except:
            pass

    preheat_time = 7
    buffer_time = 7
    total_duration = record_duration + preheat_time + buffer_time
    logger.info(f"总记录时长= {total_duration}秒 (含前{preheat_time}秒预热+后{buffer_time}秒缓冲)")

    motor_left = CyberGearMotor(can_id=127, serial_port='COM5', master_id=0x00FD,
                                position_request_interval=1/45)
    motor_right = CyberGearMotor(can_id=127, serial_port='COM4', master_id=0x00FD,
                                 position_request_interval=1/45)
    gimbal_controller = GimbalController(motor_left, motor_right, mode=selected_mode)

    gimbal_records = int(45*total_duration)
    imu_records = int(450*total_duration)

    gimbal_angle_reader = GimbalAngleReader(
        gimbal_controller=gimbal_controller,
        max_records=gimbal_records,
        output_file="gimbal_angles.txt"
    )

    imu_reader = IMUReader(
        port='COM7',
        baudrate=230400,
        output_file="imu_data.txt",
        max_lines=imu_records,
        buffer_size=imu_records*2,
        auto_reset=True,
        reset_delay=0.1
    )

    # 启动云台控制器
    controller_task = asyncio.create_task(gimbal_controller.start())

    try:
        # 连接IMU
        await imu_reader.connect()
    except Exception as e:
        logger.error(f"IMU连接失败: {e}")
        controller_task.cancel()
        try: await controller_task
        except: pass
        return

    try:
        await gimbal_angle_reader.start()
        logger.info(f"预热 {preheat_time}s...")
        await asyncio.sleep(preheat_time)

        logger.info(f"正式记录 {record_duration}s...")
        await asyncio.sleep(record_duration)

        logger.info(f"缓冲 {buffer_time}s...")
        await asyncio.sleep(buffer_time)

    except Exception as e:
        logger.error(f"运行中异常: {e}")

    finally:
        await imu_reader.stop()
        await gimbal_angle_reader.stop()

        controller_task.cancel()
        try: await controller_task
        except: pass

        logger.info("记录结束,开始同步...")

        syncer = DataSynchronizer(
            imu_reader, gimbal_angle_reader,
            output_file="synchronized_output.txt"
        )
        syncer.synchronize_and_write(record_duration)
        logger.info("程序结束.")


if __name__=="__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        logger.error(f"未捕获异常: {e}")
