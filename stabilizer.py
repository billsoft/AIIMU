#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
stabilizer.py
=============
该脚本用于读取 TDK DK-42688-P 开发板通过串口发送的 IMU 数据，
持续打印原始欧拉角（Yaw、Pitch、Roll）以及卡尔曼滤波后的欧拉角。
"""

import serial
import time
import re
import numpy as np

# 串口配置
SERIAL_PORT = 'COM11'  # 根据实际情况修改
BAUDRATE = 961200
TIMEOUT = 1  # 串口超时时间（秒）

# 正则表达式匹配 "AT,yaw,pitch,roll\r\n"
IMU_PATTERN = re.compile(r"AT,([+-]?\d+(?:\.\d+)?),([+-]?\d+(?:\.\d+)?),([+-]?\d+(?:\.\d+)?)\r?$")


class KalmanFilter:
    """
    简单的一维卡尔曼滤波器
    """

    def __init__(self, process_variance, measurement_variance, initial_estimate=0.0):
        """
        参数:
        - process_variance: 过程噪声方差
        - measurement_variance: 测量噪声方差
        - initial_estimate: 初始估计值
        """
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = initial_estimate
        self.error_covariance = 1.0  # 初始误差协方差

    def update(self, measurement):
        """
        更新卡尔曼滤波器
        """
        # 预测步骤
        predicted_estimate = self.estimate
        predicted_error_covariance = self.error_covariance + self.process_variance

        # 更新步骤
        kalman_gain = predicted_error_covariance / (predicted_error_covariance + self.measurement_variance)
        self.estimate = predicted_estimate + kalman_gain * (measurement - predicted_estimate)
        self.error_covariance = (1 - kalman_gain) * predicted_error_covariance

        return self.estimate


def calculate_compensation(yaw, pitch, roll):
    """
    计算与水平面的夹角补偿值。
    补偿值为使当前欧拉角回到水平所需调整的角度。

    参数:
    - yaw: 当前航向角
    - pitch: 当前俯仰角
    - roll: 当前滚转角

    返回:
    - compensation_yaw: 补偿航向角
    - compensation_pitch: 补偿俯仰角
    - compensation_roll: 补偿滚转角
    """
    compensation_yaw = -yaw
    compensation_pitch = -pitch
    compensation_roll = -roll
    return compensation_yaw, compensation_pitch, compensation_roll


def main():
    # 初始化卡尔曼滤波器，设置过程噪声和测量噪声方差
    # 这些参数需要根据实际情况调试
    process_var = 1e-5
    measurement_var = 1e-2
    yaw_kf = KalmanFilter(process_variance=process_var, measurement_variance=measurement_var)
    pitch_kf = KalmanFilter(process_variance=process_var, measurement_variance=measurement_var)
    roll_kf = KalmanFilter(process_variance=process_var, measurement_variance=measurement_var)

    try:
        # 建立串口连接
        ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUDRATE, timeout=TIMEOUT)
        print(f"已连接到串口 {SERIAL_PORT}，波特率 {BAUDRATE}")

        while True:
            # 读取一行数据
            line = ser.readline().decode('utf-8', errors='replace').strip()
            match = IMU_PATTERN.match(line)
            if match:
                try:
                    yaw = float(match.group(1))
                    pitch = float(match.group(2))
                    roll = float(match.group(3))

                    # 计算补偿值
                    compensation_yaw, compensation_pitch, compensation_roll = calculate_compensation(yaw, pitch, roll)

                    # 应用卡尔曼滤波器
                    yaw_filtered = yaw_kf.update(yaw)
                    pitch_filtered = pitch_kf.update(pitch)
                    roll_filtered = roll_kf.update(roll)

                    # 打印原始欧拉角、补偿值和滤波后的欧拉角
                    print(f"原始: Yaw={yaw:.2f}, Pitch={pitch:.2f}, Roll={roll:.2f} | "
                          f"补偿: Yaw={compensation_yaw:.2f}, Pitch={compensation_pitch:.2f}, Roll={compensation_roll:.2f} | "
                          f"卡尔曼: Yaw={yaw_filtered:.2f}, Pitch={pitch_filtered:.2f}, Roll={roll_filtered:.2f}")

                except ValueError:
                    # 如果转换失败，忽略该行
                    continue
            else:
                # 如果不匹配，忽略该行
                continue

    except serial.SerialException as e:
        print(f"串口错误: {e}")
    except KeyboardInterrupt:
        print("\n用户中断程序")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print(f"串口 {SERIAL_PORT} 已关闭")


if __name__ == "__main__":
    main()
