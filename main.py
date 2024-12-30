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
    同步后生成包含10列的数据文本文件:
    Timestamp IMU_Roll IMU_Pitch IMU_Yaw
    Gimbal_Roll Gimbal_Pitch Gimbal_Yaw
    Diff_Roll Diff_Pitch Diff_Yaw
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
        imu_data = self.imu_reader.get_buffer_data()
        gimbal_data = self.gimbal_reader.get_history_data()

        if not imu_data or not gimbal_data:
            logger.warning("IMU或云台数据为空, 无法同步")
            return

        # 将 gimbal_data(列表[dict]) 转成列表，方便匹配
        gimbal_list = []
        for item in gimbal_data:
            # item: {'timestamp': ts, 'roll': ..., 'pitch': ..., 'yaw': ...}
            ts = item['timestamp']
            gimbal_list.append({
                'timestamp': ts,
                'yaw':   math.radians(item['yaw']),
                'pitch': math.radians(item['pitch']),
                'roll':  math.radians(item['roll'])
            })

        gimbal_list_sorted = sorted(gimbal_list, key=lambda x: x['timestamp'])
        time_window = 0.015

        # IMU时间范围
        imu_min_time = min(d['timestamp'] for d in imu_data)
        imu_start_time = imu_min_time + 7
        imu_end_time = imu_start_time + record_duration

        matched_count = 0
        with open(self.output_file, "a", encoding='utf-8') as f:
            for imu in imu_data:
                ts_imu = imu['timestamp']
                if ts_imu < imu_start_time or ts_imu > imu_end_time:
                    continue

                candidates = [
                    g for g in gimbal_list_sorted
                    if abs(g['timestamp'] - ts_imu) <= time_window
                ]
                if not candidates:
                    continue
                best = min(candidates, key=lambda x: abs(x['timestamp'] - ts_imu))

                # IMU: (roll, pitch, yaw) => 弧度
                i_roll  = imu['roll']
                i_pitch = imu['pitch']
                i_yaw   = imu['yaw']

                # Gimbal: (roll, pitch, yaw) => 弧度
                g_roll  = best['roll']
                g_pitch = best['pitch']
                g_yaw   = best['yaw']

                diff_r = i_roll  - g_roll
                diff_p = i_pitch - g_pitch
                diff_y = i_yaw   - g_yaw

                # 转换回度数用于记录
                i_roll_deg = math.degrees(i_roll)
                i_pitch_deg = math.degrees(i_pitch)
                i_yaw_deg = math.degrees(i_yaw)
                g_roll_deg = math.degrees(g_roll)
                g_pitch_deg = math.degrees(g_pitch)
                g_yaw_deg = math.degrees(g_yaw)
                diff_r_deg = math.degrees(diff_r)
                diff_p_deg = math.degrees(diff_p)
                diff_y_deg = math.degrees(diff_y)

                line = (f"{ts_imu:.6f} "
                        f"{i_roll_deg:.2f} {i_pitch_deg:.2f} {i_yaw_deg:.2f} "
                        f"{g_roll_deg:.2f} {g_pitch_deg:.2f} {g_yaw_deg:.2f} "
                        f"{diff_r_deg:.2f} {diff_p_deg:.2f} {diff_y_deg:.2f}\n")
                f.write(line)
                matched_count += 1

        logger.info(f"同步数据已写入 {self.output_file}, 匹配 {matched_count} 条.")
        required = int(record_duration * 500)  # 修改为500Hz
        if matched_count < required:
            logger.warning(f"匹配行数 {matched_count} < 期望 {required}, "
                           "可延长记录或增大前后缓冲.")

async def main():
    print("请选择运行模式:")
    print("1. 二自由度随机运动")
    print("2. 俯仰随机运动")
    print("3. 俯仰正弦往复运动")
    print("4. 滚转随机运动")
    print("5. 滚转正弦运动")
    print("6. 水平、滚转正弦交替(平滑)")
    print("7. 左对角线正弦(新)")
    print("8. 右对角线正弦(新)")

    mode_map = {
        '1': GimbalController.MODE_2DOF_RANDOM,
        '2': GimbalController.MODE_PITCH_RANDOM,
        '3': GimbalController.MODE_PITCH_SINE,
        '4': GimbalController.MODE_ROLL,
        '5': GimbalController.MODE_ROLL_SINE,
        '6': GimbalController.MODE_LEVEL_ROLL_SEEK_EAST,
        '7': GimbalController.MODE_LEFT_DIAGONAL_SINE,   # 新增
        '8': GimbalController.MODE_RIGHT_DIAGONAL_SINE  # 新增
    }

    mode_input = input("请输入模式编号(1-8): ")
    selected_mode = mode_map.get(mode_input, GimbalController.MODE_PITCH_RANDOM)
    logger.info(f"选择的运动模式: {selected_mode}")

    # 用户输入记录时间
    while True:
        try:
            record_duration = float(input("请输入记录持续时间(秒): "))
            if record_duration > 0:
                break
            else:
                print("请输入正数!")
        except ValueError:
            print("请输入数字!")

    preheat_time = 7
    buffer_time  = 7
    total_time   = record_duration + preheat_time + buffer_time
    logger.info(f"总记录时长= {total_time}s (含前{preheat_time}秒预热+后{buffer_time}秒缓冲).")

    # 初始化 CyberGearMotor 和 GimbalController
    motor_left = CyberGearMotor(
        can_id=127,
        serial_port='COM5',
        master_id=0x00FD,
        position_request_interval=1/50  # 修改为50Hz
    )
    motor_right = CyberGearMotor(
        can_id=127,
        serial_port='COM4',
        master_id=0x00FD,
        position_request_interval=1/50  # 修改为50Hz
    )
    gimbal_controller = GimbalController(motor_left, motor_right, mode=selected_mode)

    # 更新记录数量
    gimbal_records = int(50 * total_time)  # 修改为50Hz
    imu_records    = int(500 * total_time)  # 修改为500Hz

    # 读取器
    gimbal_angle_reader = GimbalAngleReader(
        gimbal_controller=gimbal_controller,
        max_records=gimbal_records,
        output_file="gimbal_angles.txt"
    )
    imu_reader = IMUReader(
        board_type="tdk42688",                # 添加 board_type 参数
        port='COM11',                          # 修改为TDK陀螺仪的串口号
        baudrate=921600,                       # 修改为TDK陀螺仪的波特率
        output_file="tdk_imu_data.txt",        # 修改为TDK陀螺仪的输出文件
        max_lines=imu_records,
        buffer_size=imu_records*2,
        auto_reset=False,                      # TDK无需DTR复位
        reset_delay=0.0                        # TDK无需复位延迟
    )

    # 启动云台控制
    controller_task = asyncio.create_task(gimbal_controller.start())

    try:
        # 连接 IMU
        await imu_reader.connect()
    except Exception as e:
        logger.error(f"IMU连接失败: {e}")
        controller_task.cancel()
        try:
            await controller_task
        except:
            pass
        return

    try:
        # 启动 GimbalAngleReader
        await gimbal_angle_reader.start()

        logger.info(f"预热 {preheat_time}s...")
        await asyncio.sleep(preheat_time)

        logger.info(f"正式记录 {record_duration}s...")
        await asyncio.sleep(record_duration)

        logger.info(f"缓冲 {buffer_time}s...")
        await asyncio.sleep(buffer_time)

    except Exception as e:
        logger.error(f"程序运行时发生异常: {e}")

    finally:
        # 1. 停止 IMU 读取
        await imu_reader.stop()
        # 2. 停止 GimbalAngleReader
        await gimbal_angle_reader.stop()
        # 3. 停止 云台控制器
        controller_task.cancel()
        try:
            await controller_task
        except:
            pass

        # 同步数据
        logger.info("记录结束, 开始数据同步...")
        syncer = DataSynchronizer(imu_reader, gimbal_angle_reader, "synchronized_output.txt")
        syncer.synchronize_and_write(record_duration)

        logger.info("程序已结束。")

if __name__=="__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        logger.error(f"未捕获异常: {e}")
