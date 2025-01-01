#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
stabilizer_vpython.py
=====================
该脚本用于读取IMU数据，通过卡尔曼滤波平滑欧拉角（Yaw、Pitch、Roll），并使用VPython进行实时3D可视化。
"""

import asyncio
import serial
import re
from dataclasses import dataclass

import numpy as np
from vpython import box, vector, rate, color, scene

# 串口配置
SERIAL_PORT = 'COM11'  # 根据实际情况修改
BAUDRATE = 961200
TIMEOUT = 1  # 串口超时时间（秒）

# 正则表达式匹配 "AT,yaw,pitch,roll\r\n"
IMU_PATTERN = re.compile(r"AT,([+-]?\d+(?:\.\d+)?),([+-]?\d+(?:\.\d+)?),([+-]?\d+(?:\.\d+)?)\r?$")


@dataclass
class EulerAngles:
    yaw: float
    pitch: float
    roll: float


class KalmanFilter:
    """
    简单的一维卡尔曼滤波器
    """

    def __init__(self, process_variance, measurement_variance, initial_estimate=0.0):
        """
        参数:
        - process_variance: 过程噪声方差 (Q)
        - measurement_variance: 测量噪声方差 (R)
        - initial_estimate: 初始估计值 (x_0)
        """
        self.process_variance = process_variance  # Q
        self.measurement_variance = measurement_variance  # R
        self.estimate = initial_estimate  # \hat{x}_0
        self.error_covariance = 1.0  # 初始误差协方差 (P_0)

    def update(self, measurement):
        """
        更新卡尔曼滤波器
        参数:
        - measurement: 当前测量值 (z_k)

        返回:
        - 滤波后的估计值 (\hat{x}_k)
        """
        # 预测步骤
        predicted_estimate = self.estimate  # \hat{x}_k^- = \hat{x}_{k-1}
        predicted_error_covariance = self.error_covariance + self.process_variance  # P_k^- = P_{k-1} + Q

        # 更新步骤
        kalman_gain = predicted_error_covariance / (
            predicted_error_covariance + self.measurement_variance)  # K_k = P_k^- / (P_k^- + R)
        self.estimate = predicted_estimate + kalman_gain * (measurement - predicted_estimate)  # \hat{x}_k = \hat{x}_k^- + K_k * (z_k - \hat{x}_k^-)
        self.error_covariance = (1 - kalman_gain) * predicted_error_covariance  # P_k = (1 - K_k) * P_k^-

        return self.estimate  # 返回滤波后的估计值


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


class IMUVisualizer:
    """
    IMU实时可视化器，使用VPython显示3D长方体的姿态
    """

    def __init__(self):
        # 设置VPython场景
        scene.title = "IMU实时姿态可视化"
        scene.width = 800
        scene.height = 600
        scene.center = vector(0, 0, 0)
        scene.background = color.gray(0.2)

        # 创建长方体
        self.cube = box(length=50, height=10, width=80, color=color.white)

        # 创建坐标轴
        self.axes = {
            'x': box(length=60, height=0.5, width=0.5, color=color.red, pos=vector(30, 0, 0)),
            'y': box(length=0.5, height=90, width=0.5, color=color.green, pos=vector(0, 0, -45)),
            'z': box(length=0.5, height=0.5, width=90, color=color.blue, pos=vector(0, 0, -45))
        }

        # 初始角度
        self.euler_angles = EulerAngles(yaw=0.0, pitch=0.0, roll=0.0)

    def update_orientation(self, euler_angles: EulerAngles):
        self.euler_angles = euler_angles
        self.apply_rotation()

    def apply_rotation(self):
        # 清除之前的旋转
        self.cube.axis = vector(1, 0, 0)
        self.cube.up = vector(0, 1, 0)

        # 应用欧拉角旋转
        # 顺序为 Yaw → Pitch → Roll
        # 在VPython中，旋转是通过四元数或旋转矩阵实现的
        # 为简单起见，这里使用numpy来创建旋转矩阵

        # 转换角度为弧度
        yaw = np.radians(self.euler_angles.yaw)
        pitch = np.radians(self.euler_angles.pitch)
        roll = np.radians(self.euler_angles.roll)

        # 计算旋转矩阵
        R_yaw = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw),  np.cos(yaw), 0],
            [0,           0,            1]
        ])

        R_pitch = np.array([
            [1, 0,              0],
            [0, np.cos(pitch), -np.sin(pitch)],
            [0, np.sin(pitch),  np.cos(pitch)]
        ])

        R_roll = np.array([
            [ np.cos(roll), 0, np.sin(roll)],
            [0,             1, 0],
            [-np.sin(roll), 0, np.cos(roll)]
        ])

        # 综合旋转矩阵：R = R_yaw * R_pitch * R_roll
        R = R_yaw @ R_pitch @ R_roll

        # 更新长方体的方向
        self.cube.axis = vector(R[0, 0], R[1, 0], R[2, 0])
        self.cube.up = vector(R[0, 1], R[1, 1], R[2, 1])


async def read_imu_data(yaw_kf, pitch_kf, roll_kf, visualizer):
    """
    异步读取串口IMU数据，应用卡尔曼滤波，并更新可视化
    """
    try:
        # 建立串口连接
        ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUDRATE, timeout=TIMEOUT)
        print(f"已连接到串口 {SERIAL_PORT}，波特率 {BAUDRATE}")

        while True:
            # 异步读取一行数据
            if ser.in_waiting:
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
                        # 根据您的要求，映射如下：
                        # Roll -> Yaw, Yaw -> Pitch, Pitch -> Roll
                        yaw_filtered = roll_kf.update(compensation_roll)      # Roll -> Yaw
                        pitch_filtered = yaw_kf.update(compensation_yaw)      # Yaw -> Pitch
                        roll_filtered = pitch_kf.update(compensation_pitch)   # Pitch -> Roll

                        # 创建EulerAngles实例
                        euler_angles = EulerAngles(yaw=yaw_filtered, pitch=pitch_filtered, roll=roll_filtered)

                        # 更新3D可视化
                        visualizer.update_orientation(euler_angles)

                        # 可选：打印滤波后的角度
                        # print(f"Yaw: {yaw_filtered:.2f}, Pitch: {pitch_filtered:.2f}, Roll: {roll_filtered:.2f}")

                    except ValueError:
                        # 如果转换失败，忽略该行
                        continue
            await asyncio.sleep(0.001)  # 更高的数据读取频率

    except serial.SerialException as e:
        print(f"串口错误: {e}")
    except asyncio.CancelledError:
        print("IMU数据读取任务取消")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print(f"串口 {SERIAL_PORT} 已关闭")


async def main():
    # 初始化卡尔曼滤波器，设置过程噪声和测量噪声方差
    # 为了提高响应速度，增大 Q，减小 R
    process_var = 1e-4  # 调整过程噪声方差以提高响应速度
    measurement_var = 1e-4  # 调整测量噪声方差以提高响应速度
    yaw_kf = KalmanFilter(process_variance=process_var, measurement_variance=measurement_var)
    pitch_kf = KalmanFilter(process_variance=process_var, measurement_variance=measurement_var)
    roll_kf = KalmanFilter(process_variance=process_var, measurement_variance=measurement_var)

    # 初始化3D可视化
    visualizer = IMUVisualizer()

    # 启动IMU数据读取任务
    imu_task = asyncio.create_task(read_imu_data(yaw_kf, pitch_kf, roll_kf, visualizer))

    try:
        await imu_task  # 主协程等待IMU任务完成
    except asyncio.CancelledError:
        pass
    finally:
        # VPython没有显式的清理步骤
        pass


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n用户中断程序")
