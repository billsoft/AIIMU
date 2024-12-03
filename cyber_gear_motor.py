# cyber_gear_motor.py

import asyncio
import struct
import logging
import math  # 确保math模块被导入
from typing import Optional
from constants import Constants
from protocol_handler import ProtocolHandler, MotorEvent, EventType, MotorMessage
from async_serial import AsyncSerial

# 设置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    """将浮点数转换为无符号整数。"""
    span = x_max - x_min
    offset = x_min
    x_clipped = min(max(x, x_min), x_max)
    uint_value = int((x_clipped - offset) * ((1 << bits) - 1) / span)
    return uint_value

def uint_to_float(uint_val: int, x_min: float, x_max: float, bits: int) -> float:
    """将无符号整数转换为浮点数。"""
    span = x_max - x_min
    max_uint = (1 << bits) - 1
    uint_val_clipped = min(max(uint_val, 0), max_uint)
    x = (float(uint_val_clipped) * span) / max_uint + x_min
    return x

class CyberGearMotor:
    """电机控制类"""

    def __init__(self, can_id: int, serial_port: str, master_id: int = 0x00FD,
                 position_request_interval: float = 1 / 120):
        self.can_id = can_id & 0xFF
        self.master_id = master_id & 0xFF
        self.cache = {}
        self.protocol_handler = ProtocolHandler()
        self.serial_port = serial_port
        self.serial = AsyncSerial(
            port=serial_port,
            on_frame_received=self._on_frame_received,
        )
        self.control_task = None
        self.position_task = None
        self.position_request_interval = position_request_interval
        self.is_connected = False
        self.current_mode = Constants.RunMode['UNKNOWN']

    async def connect(self):
        """连接串口并启动后台任务"""
        if self.is_connected:
            logger.info(f"电机 {self.can_id} 已经连接。")
            return

        try:
            await self.serial.connect()
            self.is_connected = True
            logger.info(f"电机 {self.can_id} 连接到串口 {self.serial_port} 成功。")
            # 启动后台任务，定期发送写参数命令以触发电机反馈
            self.position_task = asyncio.create_task(self._real_time_feedback_loop())
        except Exception as e:
            self.is_connected = False
            logger.error(f"电机 {self.can_id} 无法连接串口 {self.serial_port}：{e}")
            raise e

    async def close(self):
        """关闭电机控制"""
        if not self.is_connected:
            logger.warning(f"电机 {self.can_id} 未连接。")
            return
        if self.control_task:
            self.control_task.cancel()
            try:
                await self.control_task
            except asyncio.CancelledError:
                pass
        if self.position_task:
            self.position_task.cancel()
            try:
                await self.position_task
            except asyncio.CancelledError:
                pass
        await self.stop_motor()
        await asyncio.sleep(0.1)
        await self.serial.close()
        logger.info(f"串口 {self.serial_port} 已关闭")
        self.is_connected = False
        logger.info(f"电机 {self.can_id} 已关闭。")

    async def send_message(self, message: MotorMessage, direction: int = Constants.DIRECTION_TO_MOTOR):
        """发送消息"""
        frame = self.protocol_handler.encode_message(message, direction)
        await self.serial.send_frame(frame)
        logger.debug(f"电机 {self.can_id} 发送消息: {frame.hex()}")

    async def set_zero_position(self):
        """设置电机机械零位"""
        data = bytearray(8)
        data[0] = 0x01
        message = MotorMessage(
            can_id=self.can_id,
            master_id=self.master_id,
            communication_type=Constants.CommunicationType['SET_ZERO_POSITION'],
            data_field=self.master_id,
            data=data
        )
        await self.send_message(message)
        await asyncio.sleep(0.1)
        logger.info("已发送设置机械零位命令。")

    async def set_run_mode(self, run_mode: int):
        """设置运行模式并使能电机"""
        index = Constants.ParamIndex['RUN_MODE']['index']
        data = bytearray(8)
        # 数据区使用小端字节序 '<'
        param_type = Constants.ParamIndex['RUN_MODE']['type']
        if param_type == 'uint8':
            struct.pack_into('<H2xB3x', data, 0, index, run_mode)
        elif param_type == 'uint16':
            struct.pack_into('<H2xH2x', data, 0, index, run_mode)
        elif param_type == 'uint32':
            struct.pack_into('<H2xI', data, 0, index, run_mode)
        else:
            logger.error(f"Unsupported parameter type for RUN_MODE: {param_type}")
            return
        message = MotorMessage(
            can_id=self.can_id,
            master_id=self.master_id,
            communication_type=Constants.CommunicationType['WRITE_PARAM'],
            data_field=self.master_id,
            data=data
        )
        await self.send_message(message)
        await asyncio.sleep(0.1)
        self.current_mode = run_mode
        await self.enable_motor()

    def _on_frame_received(self, frame: bytes):
        """处理接收到的帧"""
        logger.debug(f"电机 {self.can_id} 接收到帧：{frame.hex().upper()}")
        result = self.protocol_handler.decode_frame(frame)
        if not result:
            return
        message, event = result
        if message.can_id != self.can_id:
            return
        if event:
            logger.debug(f"电机 {self.can_id} 收到事件: {event.event_type}")
            self._handle_event(event)
        else:
            logger.warning(f"电机 {self.can_id} 收到未知类型的消息。")

    def _handle_event(self, event: Optional[MotorEvent]):
        """根据事件处理相关数据"""
        if not event:
            return
        if event.event_type == EventType.MOTOR_FEEDBACK:
            self._handle_motor_feedback(event.data)
        elif event.event_type == EventType.PARAM_FEEDBACK:
            self._handle_param_feedback(event.data)
        elif event.event_type == EventType.FAULT_FEEDBACK:
            self._handle_fault_feedback(event.data)

    def _handle_motor_feedback(self, data: dict):
        """处理电机反馈数据"""
        try:
            self.cache.update(data)
            logger.debug(f"电机 {self.can_id} 电机反馈数据更新：{data}")
        except Exception as e:
            logger.error(f"解析电机反馈数据错误: {e}")

    def _handle_param_feedback(self, data: dict):
        """处理参数反馈"""
        try:
            self.cache.update({
                f"param_{data['param_index']}": data['param_value'],
            })
            logger.info(f"电机 {self.can_id} 参数反馈更新：{data}")
        except Exception as e:
            logger.error(f"解析参数反馈错误: {e}")

    def _handle_fault_feedback(self, data: dict):
        """处理故障反馈"""
        try:
            self.cache.update(data)
            logger.warning(f"电机 {self.can_id} 故障信息: {data}")
        except Exception as e:
            logger.error(f"解析故障反馈错误: {e}")

    async def set_limit_spd(self, speed: float):
        """设置电机最大速度"""
        index = Constants.ParamIndex['LIMIT_SPD']['index']
        data = bytearray(8)
        struct.pack_into('<H2xf', data, 0, index, speed)
        message = MotorMessage(
            can_id=self.can_id,
            master_id=self.master_id,
            communication_type=Constants.CommunicationType['WRITE_PARAM'],
            data_field=self.master_id,
            data=data
        )
        await self.send_message(message)
        logger.info(f"已发送设置最大速度：{speed} rad/s")

    async def set_loc_ref(self, position: float):
        """设置位置参考"""
        index = Constants.ParamIndex['LOC_REF']['index']
        data = bytearray(8)
        struct.pack_into('<H2xf', data, 0, index, position)
        message = MotorMessage(
            can_id=self.can_id,
            master_id=self.master_id,
            communication_type=Constants.CommunicationType['WRITE_PARAM'],
            data_field=self.master_id,
            data=data
        )
        await self.send_message(message)
        logger.info(f"已发送设置位置参考：{position} rad")

    async def set_position(self, position: float, speed: float = 2.0):
        """设置电机位置"""
        if self.current_mode != Constants.RunMode['POSITION_MODE']:
            await self.stop_motor()
            await asyncio.sleep(0.01)
            await self.set_run_mode(Constants.RunMode['POSITION_MODE'])
            await asyncio.sleep(0.01)
        await self.set_limit_spd(speed)
        await asyncio.sleep(0.01)
        await self.set_loc_ref(position)
        await asyncio.sleep(0.01)
        logger.info(f"已发送位置指令：{position} rad")

    async def enable_motor(self):
        """使能电机"""
        data = bytearray(8)
        message = MotorMessage(
            can_id=self.can_id,
            master_id=self.master_id,
            communication_type=Constants.CommunicationType['MOTOR_ENABLE'],
            data_field=self.master_id,
            data=data
        )
        await self.send_message(message)
        logger.info("电机已使能。")

    async def stop_motor(self):
        """禁用电机"""
        data = bytearray(8)
        message = MotorMessage(
            can_id=self.can_id,
            master_id=self.master_id,
            communication_type=Constants.CommunicationType['MOTOR_STOP'],
            data_field=self.master_id,
            data=data
        )
        await self.send_message(message)
        logger.info("电机已停止。")

    async def emergency_stop(self):
        """紧急停止电机"""
        data = bytearray(8)
        data[0] = 0x01
        message = MotorMessage(
            can_id=self.can_id,
            master_id=self.master_id,
            communication_type=Constants.CommunicationType['EMERGENCY_STOP'],
            data_field=self.master_id,
            data=data
        )
        await self.send_message(message)
        logger.info("已发送紧急停止命令。")

    async def _real_time_feedback_loop(self):
        """后台任务，定期发送写参数命令以触发电机反馈，获取实时状态"""
        try:
            while True:
                await self.send_write_param_command()
                await asyncio.sleep(self.position_request_interval)
        except asyncio.CancelledError:
            pass

    async def send_write_param_command(self):
        """发送写参数命令，触发电机返回通信类型 2 的反馈"""
        index = Constants.ParamIndex['CUR_KP']['index']  # 假设使用 CUR_KP 参数
        data = bytearray(8)
        cur_kp_value = self.cache.get(f"param_{index}", 1.0)  # 默认值为 1.0
        struct.pack_into('<H2xf', data, 0, index, cur_kp_value)
        message = MotorMessage(
            can_id=self.can_id,
            master_id=self.master_id,
            communication_type=Constants.CommunicationType['WRITE_PARAM'],
            data_field=self.master_id,
            data=data
        )
        await self.send_message(message)
        logger.debug(f"电机 {self.can_id} 发送写参数命令以触发反馈。")

    def get_real_time_position(self) -> Optional[float]:
        """获取当前的实时位置（弧度）并归一化到 [-pi, pi]"""
        angle = self.cache.get('angle', None)
        if angle is not None:
            # 归一化角度到 [-pi, pi]
            angle_normalized = ((angle + math.pi) % (2 * math.pi)) - math.pi
            logger.info(f"电机 {self.can_id} 实时位置：{angle_normalized} 弧度")
            return angle_normalized
        else:
            logger.warning(f"电机 {self.can_id} 实时位置未就绪。")
            return None

    async def reset_rotation(self):
        """重置圈数为 0"""
        index = Constants.ParamIndex['ROTATION']['index']
        data = bytearray(8)
        struct.pack_into('<H2xh', data, 0, index, 0)
        message = MotorMessage(
            can_id=self.can_id,
            master_id=self.master_id,
            communication_type=Constants.CommunicationType['WRITE_PARAM'],
            data_field=self.master_id,
            data=data
        )
        await self.send_message(message)
        logger.info(f"电机 {self.can_id} 圈数已重置为 0。")

    async def send_control_command(self, angle: float = 0.0, speed: float = 0.0,
                                   kp: float = 0.0, kd: float = 0.0):
        """发送运动控制命令（通信类型 1）

        Args:
            angle (float): 目标角度（弧度），范围 [-4π, 4π]
            speed (float): 目标速度（弧度/秒），范围 [-30, 30]
            kp (float): 比例增益，范围 [0, 500]
            kd (float): 微分增益，范围 [0, 5]
        """
        data = bytearray(8)
        # 将物理量转换为无符号整数
        angle_raw = float_to_uint(angle, -4 * math.pi, 4 * math.pi, 16)
        speed_raw = float_to_uint(speed, -30.0, 30.0, 16)
        kp_raw = float_to_uint(kp, 0.0, 500.0, 16)
        kd_raw = float_to_uint(kd, 0.0, 5.0, 16)
        # 组装数据
        # 由于所有字段均为 2 字节，直接拼接
        struct.pack_into('<HHHH', data, 0, angle_raw, speed_raw, kp_raw, kd_raw)
        message = MotorMessage(
            can_id=self.can_id,
            master_id=self.master_id,
            communication_type=Constants.CommunicationType['MOTOR_CONTROL'],
            data_field=self.master_id,
            data=data
        )
        await self.send_message(message)
        logger.debug(f"电机 {self.can_id} 发送运动控制命令。")
