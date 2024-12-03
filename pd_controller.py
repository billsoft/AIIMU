# pd_controller.py

"""
实现改进的 PD 控制器，用于精确控制电机位置。
"""

import time

class PDController:
    """
    改进的 PD 控制器类，用于计算控制输出（力矩）。
    """

    def __init__(self, kp: float, kd: float, target_position: float):
        """
        初始化 PD 控制器。

        :param kp: 比例增益系数
        :param kd: 微分增益系数
        :param target_position: 目标位置（单位：弧度）
        """
        self.kp = kp                          # 比例增益系数
        self.kd = kd                          # 微分增益系数
        self.target_position = target_position  # 目标位置
        self.previous_error = 0.0             # 上一次的误差
        self.previous_time = time.time()      # 上一次的时间戳

    def compute_control(self, current_position: float, current_velocity: float) -> float:
        """
        计算控制输出（力矩）。

        :param current_position: 当前电机位置（单位：弧度）
        :param current_velocity: 当前电机速度（单位：弧度/秒）
        :return: 控制力矩（单位：牛米）
        """
        current_time = time.time()
        delta_time = current_time - self.previous_time

        # 计算位置误差
        error = self.target_position - current_position

        # 计算误差的变化率（这里使用电机的实际速度）
        error_rate = -current_velocity  # 误差变化率为目标速度（0）减去当前速度

        # PD 控制器输出
        control_output = self.kp * error + self.kd * error_rate

        # 更新状态
        self.previous_error = error
        self.previous_time = current_time

        return control_output
