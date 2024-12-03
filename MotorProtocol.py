import asyncio
import serial_asyncio
import struct
from typing import Optional, Dict, Any
from datetime import datetime

class MotorProtocol:
    def __init__(self, master_id: int, target_id: int):
        """
        初始化 MotorProtocol 类
        :param master_id: 主机 CAN ID（8位）
        :param target_id: 目标电机 CAN ID（8位）
        """
        self.master_id = master_id  # 主机CAN ID（8位）
        self.target_id = target_id  # 目标电机CAN ID（8位）

    def float_to_uint(self, x: float, x_min: float, x_max: float, bits: int) -> int:
        """
        将浮点数转换为无符号整数，确保在最小值和最大值范围内
        :param x: 浮点数值
        :param x_min: 最小值
        :param x_max: 最大值
        :param bits: 位数
        :return: 无符号整数
        """
        span = x_max - x_min  # 计算范围跨度
        if x > x_max:
            x = x_max  # 限制最大值
        elif x < x_min:
            x = x_min  # 限制最小值
        return int((x - x_min) * ((1 << bits) - 1) / span)  # 归一化并转换为整数

    def create_can_id(self, comm_type: int) -> int:
        """
        根据通信类型、主机CAN ID和目标电机CAN ID生成29位CAN ID
        :param comm_type: 通信类型
        :return: 29位CAN ID
        """
        can_id = (comm_type & 0x1F) << 24  # Bit28~Bit24: 通信类型
        can_id |= (self.master_id & 0xFF) << 16  # Bit23~Bit16: 主机CAN ID
        can_id |= (self.target_id & 0xFF) << 8   # Bit15~Bit8: 目标电机CAN ID
        # Bit7~Bit0 保留为0
        return can_id

    def construct_can_frame(self, can_id: int, data: bytes) -> bytes:
        """
        构建CAN帧
        :param can_id: 29位CAN ID
        :param data: 8字节数据
        :return: 完整的CAN帧（12字节：4字节CAN ID + 8字节数据）
        """
        return struct.pack('>I8s', can_id, data)  # 大端序

    def enable_motor(self) -> bytes:
        """
        发送电机使能运行帧（通信类型3）
        :return: 构建好的CAN帧
        """
        comm_type = 3  # 通信类型3：电机使能运行
        can_id = self.create_can_id(comm_type)  # 创建CAN ID
        data = bytearray([0] * 8)  # 初始化数据域
        data[0] = self.target_id  # 设置目标CAN ID
        return self.construct_can_frame(can_id, bytes(data))  # 构建CAN帧

    def disable_motor(self) -> bytes:
        """
        发送电机停止运行帧（通信类型4）
        :return: 构建好的CAN帧
        """
        comm_type = 4  # 通信类型4：电机停止运行
        can_id = self.create_can_id(comm_type)  # 创建CAN ID
        data = bytearray([0] * 8)  # 初始化数据域
        data[0] = 1  # Byte0=1 清故障
        return self.construct_can_frame(can_id, bytes(data))  # 构建CAN帧

    def switch_mode(self, mode: int) -> bytes:
        """
        切换电机模式（通信类型18）
        :param mode: 目标模式（1:位置模式, 2:速度模式, 3:电流模式, 0:运控模式）
        :return: 构建好的CAN帧
        """
        comm_type = 18  # 通信类型18：单个参数写入
        can_id = self.create_can_id(comm_type)  # 创建CAN ID
        data = bytearray([0] * 8)  # 初始化数据域
        index = 0x7005  # 参数索引：run_mode
        data[0:2] = struct.pack('>H', index)  # 设置参数索引（大端）
        data[4] = mode  # 设置目标模式
        return self.construct_can_frame(can_id, bytes(data))  # 构建CAN帧

    def set_speed(self, speed: float) -> bytes:
        """
        设置电机速度（通信类型18）
        :param speed: 速度值（-30~30 rad/s）
        :return: 构建好的CAN帧
        """
        comm_type = 18  # 通信类型18：单个参数写入
        can_id = self.create_can_id(comm_type)  # 创建CAN ID
        data = bytearray([0] * 8)  # 初始化数据域
        index = 0x700A  # 参数索引：spd_ref
        data[0:2] = struct.pack('>H', index)  # 设置参数索引（大端）
        data[4:8] = struct.pack('>f', speed)  # 设置速度值（float转字节，大端）
        return self.construct_can_frame(can_id, bytes(data))  # 构建CAN帧

    def set_position(self, position: float) -> bytes:
        """
        设置电机位置（通信类型18）
        :param position: 位置值（单位rad）
        :return: 构建好的CAN帧
        """
        comm_type = 18  # 通信类型18：单个参数写入
        can_id = self.create_can_id(comm_type)  # 创建CAN ID
        data = bytearray([0] * 8)  # 初始化数据域
        index = 0x7016  # 参数索引：loc_ref
        data[0:2] = struct.pack('>H', index)  # 设置参数索引（大端）
        data[4:8] = struct.pack('>f', position)  # 设置位置值（float转字节，大端）
        return self.construct_can_frame(can_id, bytes(data))  # 构建CAN帧

    def set_current(self, iq_ref: float) -> bytes:
        """
        设置电机电流（通信类型18）
        :param iq_ref: 电流值（-23~23 A）
        :return: 构建好的CAN帧
        """
        comm_type = 18  # 通信类型18：单个参数写入
        can_id = self.create_can_id(comm_type)  # 创建CAN ID
        data = bytearray([0] * 8)  # 初始化数据域
        index = 0x7006  # 参数索引：iq_ref
        data[0:2] = struct.pack('>H', index)  # 设置参数索引（大端）
        data[4:8] = struct.pack('>f', iq_ref)  # 设置电流值（float转字节，大端）
        return self.construct_can_frame(can_id, bytes(data))  # 构建CAN帧

    def set_run_control(self, torque: float, mech_position: float, speed: float, kp: float, kd: float) -> bytes:
        """
        发送运控模式电机控制指令（通信类型1）
        :param torque: 力矩 (-12~12 Nm)
        :param mech_position: 目标机械位置 (-4π~4π rad)
        :param speed: 目标角速度 (-30~30 rad/s)
        :param kp: Kp (0.0~500.0)
        :param kd: Kd (0.0~5.0)
        :return: 构建好的CAN帧
        """
        comm_type = 1  # 通信类型1：运控模式控制指令
        can_id = self.create_can_id(comm_type)  # 创建CAN ID
        data = bytearray([0] * 8)  # 初始化数据域

        # 力矩转换并设置
        torque_uint = self.float_to_uint(torque, -12.0, 12.0, 16)
        data[2:4] = struct.pack('>H', torque_uint)  # Byte2~3: 力矩

        # 目标角度转换并设置
        angle_uint = self.float_to_uint(mech_position, -4 * 3.14159265359, 4 * 3.14159265359, 16)
        data[0:2] = struct.pack('>H', angle_uint)  # Byte0~1: 目标角度

        # 目标角速度转换并设置
        speed_uint = self.float_to_uint(speed, -30.0, 30.0, 16)
        data[2:4] = struct.pack('>H', speed_uint)  # Byte2~3: 目标角速度

        # Kp 转换并设置
        kp_uint = self.float_to_uint(kp, 0.0, 500.0, 16)
        data[4:6] = struct.pack('>H', kp_uint)  # Byte4~5: Kp

        # Kd 转换并设置
        kd_uint = self.float_to_uint(kd, 0.0, 5.0, 16)
        data[6:8] = struct.pack('>H', kd_uint)  # Byte6~7: Kd

        return self.construct_can_frame(can_id, bytes(data))  # 构建CAN帧

    def read_parameter(self, index: int) -> bytes:
        """
        读取单个参数（通信类型17）
        :param index: 参数索引
        :return: 构建好的CAN帧
        """
        comm_type = 17  # 通信类型17：单个参数读取
        can_id = self.create_can_id(comm_type)  # 创建CAN ID
        data = bytearray([0] * 8)  # 初始化数据域
        data[0:2] = struct.pack('>H', index)  # 设置参数索引（大端）
        # Byte2~7 保留为0
        return self.construct_can_frame(can_id, bytes(data))  # 构建CAN帧
