#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
stabilizer_opengl.py
====================
该脚本用于读取IMU数据，通过卡尔曼滤波平滑欧拉角（Yaw、Pitch、Roll），并使用PyOpenGL和GLFW进行实时3D可视化。
"""

import asyncio
import serial
import time
import re
import math
import numpy as np
import glfw
from OpenGL.GL import *
from OpenGL.GLU import *
from ctypes import c_void_p
from dataclasses import dataclass

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

class IMUVisualizer:
    """
    IMU实时可视化器，使用PyOpenGL和GLFW显示3D长方体的姿态
    """
    def __init__(self):
        # 初始化GLFW
        if not glfw.init():
            raise Exception("GLFW初始化失败")

        # 创建窗口
        self.window = glfw.create_window(800, 600, "IMU实时姿态可视化", None, None)
        if not self.window:
            glfw.terminate()
            raise Exception("GLFW窗口创建失败")

        glfw.make_context_current(self.window)

        # 设置OpenGL状态
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_COLOR_MATERIAL)
        glClearColor(0.2, 0.2, 0.2, 1.0)

        # 设置视图
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, (800/600), 0.1, 50.0)
        glMatrixMode(GL_MODELVIEW)

        # 初始角度
        self.euler_angles = EulerAngles(yaw=0.0, pitch=0.0, roll=0.0)

    def update_orientation(self, euler_angles: EulerAngles):
        self.euler_angles = euler_angles

    def draw_cube(self):
        """
        绘制旋转后的长方体
        """
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        # 移动摄像机
        glTranslatef(0.0, 0.0, -10.0)
        # 应用旋转
        glRotatef(self.euler_angles.yaw, 0, 1, 0)    # Yaw: around Y-axis
        glRotatef(self.euler_angles.pitch, 1, 0, 0)  # Pitch: around X-axis
        glRotatef(self.euler_angles.roll, 0, 0, 1)   # Roll: around Z-axis

        # 绘制长方体
        glBegin(GL_QUADS)

        # 前面
        glColor3f(1, 0, 0)  # 红色
        glVertex3f(-1, -0.5,  2)
        glVertex3f( 1, -0.5,  2)
        glVertex3f( 1,  0.5,  2)
        glVertex3f(-1,  0.5,  2)

        # 后面
        glColor3f(0, 1, 0)  # 绿色
        glVertex3f(-1, -0.5, -2)
        glVertex3f( 1, -0.5, -2)
        glVertex3f( 1,  0.5, -2)
        glVertex3f(-1,  0.5, -2)

        # 左面
        glColor3f(0, 0, 1)  # 蓝色
        glVertex3f(-1, -0.5, -2)
        glVertex3f(-1, -0.5,  2)
        glVertex3f(-1,  0.5,  2)
        glVertex3f(-1,  0.5, -2)

        # 右面
        glColor3f(1, 1, 0)  # 黄色
        glVertex3f(1, -0.5, -2)
        glVertex3f(1, -0.5,  2)
        glVertex3f(1,  0.5,  2)
        glVertex3f(1,  0.5, -2)

        # 上面
        glColor3f(1, 0, 1)  # 品红色
        glVertex3f(-1, 0.5,  2)
        glVertex3f( 1, 0.5,  2)
        glVertex3f( 1, 0.5, -2)
        glVertex3f(-1, 0.5, -2)

        # 下面
        glColor3f(0, 1, 1)  # 青色
        glVertex3f(-1, -0.5,  2)
        glVertex3f( 1, -0.5,  2)
        glVertex3f( 1, -0.5, -2)
        glVertex3f(-1, -0.5, -2)

        glEnd()

        glfw.swap_buffers(self.window)

    def should_close(self):
        return glfw.window_should_close(self.window)

    def poll_events(self):
        glfw.poll_events()

    def cleanup(self):
        glfw.terminate()

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
                        yaw_filtered = yaw_kf.update(yaw)
                        pitch_filtered = pitch_kf.update(pitch)
                        roll_filtered = roll_kf.update(roll)

                        # 创建EulerAngles实例
                        euler_angles = EulerAngles(yaw=yaw_filtered, pitch=pitch_filtered, roll=roll_filtered)

                        # 更新3D可视化
                        visualizer.update_orientation(euler_angles)

                        # 打印原始欧拉角、补偿值和滤波后的欧拉角
                        print(f"原始: Yaw={yaw:.2f}, Pitch={pitch:.2f}, Roll={roll:.2f} | "
                              f"补偿: Yaw={compensation_yaw:.2f}, Pitch={compensation_pitch:.2f}, Roll={compensation_roll:.2f} | "
                              f"卡尔曼: Yaw={yaw_filtered:.2f}, Pitch={pitch_filtered:.2f}, Roll={roll_filtered:.2f}")

                    except ValueError:
                        # 如果转换失败，忽略该行
                        continue
            await asyncio.sleep(0.001)  # Yield control to the event loop

    except serial.SerialException as e:
        print(f"串口错误: {e}")
    except asyncio.CancelledError:
        print("IMU数据读取任务取消")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print(f"串口 {SERIAL_PORT} 已关闭")

async def render_loop(visualizer):
    """
    异步渲染循环
    """
    while True:
        if not visualizer.should_close():
            visualizer.draw_cube()
            visualizer.poll_events()
        else:
            break
        await asyncio.sleep(0.016)  # ~60 FPS

async def main():
    # 初始化卡尔曼滤波器，设置过程噪声和测量噪声方差
    process_var = 1e-5
    measurement_var = 1e-2
    yaw_kf = KalmanFilter(process_variance=process_var, measurement_variance=measurement_var)
    pitch_kf = KalmanFilter(process_variance=process_var, measurement_variance=measurement_var)
    roll_kf = KalmanFilter(process_variance=process_var, measurement_variance=measurement_var)

    # 初始化3D可视化
    visualizer = IMUVisualizer()

    # 启动IMU数据读取任务
    imu_task = asyncio.create_task(read_imu_data(yaw_kf, pitch_kf, roll_kf, visualizer))

    # 启动渲染循环任务
    render_task = asyncio.create_task(render_loop(visualizer))

    try:
        await asyncio.gather(imu_task, render_task)
    except asyncio.CancelledError:
        pass
    finally:
        visualizer.cleanup()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n用户中断程序")
