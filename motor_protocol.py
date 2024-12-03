# protocol_handler.py

import struct
import logging
from typing import Optional, Tuple, Any, Dict
from dataclasses import dataclass
from enum import Enum
from constants import Constants

# 设置日志
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    """将浮点数转换为无符号整数。"""
    span = x_max - x_min
    offset = x_min
    x = min(max(x, x_min), x_max)
    uint_value = int((x - offset) * ((1 << bits) - 1) / span)
    return uint_value

def uint_to_float(uint_val: int, x_min: float, x_max: float, bits: int) -> float:
    """将无符号整数转换为浮点数。"""
    span = x_max - x_min
    max_uint = (1 << bits) - 1
    uint_val = min(max(uint_val, 0), max_uint)
    x = (float(uint_val) * span) / max_uint + x_min
    return x

class EventType(Enum):
    MOTOR_FEEDBACK = "motor_feedback"
    FAULT_FEEDBACK = "fault_feedback"
    PARAM_FEEDBACK = "param_feedback"

@dataclass
class MotorEvent:
    event_type: EventType
    motor_id: int
    data: Dict[str, Any]
    raw_frame: Optional[bytes] = None

@dataclass
class MotorMessage:
    can_id: int
    master_id: int
    communication_type: int
    data_field: int
    data: bytes
    raw_frame: Optional[bytes] = None

class ProtocolHandler:
    FRAME_HEADER = b"AT"
    FRAME_TAIL = b"\r\n"

    def encode_message(self, message: MotorMessage,
                       direction: int = Constants.DIRECTION_TO_MOTOR) -> bytes:
        """编码消息为二进制帧。"""
        can_id = self._build_can_id(
            message.communication_type,
            message.master_id,
            message.can_id,
            direction
        )
        shifted_can_id = (can_id << 3) | 0x04
        frame = (self.FRAME_HEADER +
                 struct.pack('>I B', shifted_can_id, len(message.data)) +
                 message.data + self.FRAME_TAIL)
        logger.debug(f"[Encode] Original CAN ID: {hex(can_id)}, "
                     f"Shifted CAN ID: {hex(shifted_can_id)}, "
                     f"Data Length: {len(message.data)}")
        return frame

    def decode_frame(self, frame: bytes) -> Optional[Tuple[MotorMessage,
                                                           Optional[MotorEvent]]]:
        """解码二进制帧为消息和事件。"""
        try:
            if not frame.startswith(self.FRAME_HEADER) or \
               not frame.endswith(self.FRAME_TAIL):
                raise ValueError("帧头或帧尾不匹配")

            shifted_can_id_bytes = frame[2:6]
            data_length = frame[6]
            data = frame[7:-2]

            if len(data) != data_length:
                raise ValueError(f"数据长度不匹配：期望 {data_length}，实际 {len(data)}")

            shifted_can_id = int.from_bytes(shifted_can_id_bytes, 'big')
            restored_can_id = shifted_can_id >> 3
            extended_id = restored_can_id  # 保存29位ID

            # 解析通信类型、方向、源ID和目标ID
            comm_type = (restored_can_id >> 24) & 0x1F
            direction = (restored_can_id >> 16) & 0x01
            source_id = (restored_can_id >> 8) & 0xFF
            target_id = restored_can_id & 0xFF

            logger.debug(f"[Decode] Shifted CAN ID: {hex(shifted_can_id)}, "
                         f"Restored CAN ID: {hex(restored_can_id)}")
            logger.debug(f"[Decode] Comm Type: {comm_type}, Direction: {direction}, "
                         f"Source ID: {hex(source_id)}, Target ID: {hex(target_id)}")

            if direction == Constants.DIRECTION_TO_MOTOR:
                master_id = source_id
                motor_id = target_id
            else:
                master_id = target_id
                motor_id = source_id

            message = MotorMessage(
                can_id=motor_id,
                master_id=master_id,
                communication_type=comm_type,
                data_field=source_id,
                data=data,
                raw_frame=frame
            )

            event = self._create_event(message, extended_id)
            if event:
                logger.info(f"[Received Event] Type: {event.event_type}, Motor ID: {event.motor_id}")
            else:
                logger.debug(f"[Received Message] Communication Type: {message.communication_type}, No Event Created")

            return message, event

        except Exception as e:
            logger.error(f"[Decode Error] {e}")
            return None

    def _build_can_id(self, communication_type: int, source_id: int,
                      target_id: int, direction: int) -> int:
        """构建 CAN ID。"""
        can_id = (
            ((communication_type & 0x1F) << 24) |
            ((direction & 0x01) << 16) |
            ((source_id & 0xFF) << 8) |
            (target_id & 0xFF)
        )
        logger.debug(f"[Build CAN ID] Comm Type: {communication_type}, "
                     f"Direction: {direction}, Source ID: {source_id}, "
                     f"Target ID: {target_id}, CAN ID: {hex(can_id)}")
        return can_id

    def _create_event(self, message: MotorMessage, extended_id: int) -> Optional[MotorEvent]:
        """根据消息创建事件。"""
        if message.communication_type == Constants.CommunicationType.MOTOR_FEEDBACK.value:
            feedback_data = self._decode_motor_feedback(message.data, extended_id)
            if feedback_data:
                return MotorEvent(EventType.MOTOR_FEEDBACK, message.can_id,
                                  feedback_data, message.raw_frame)
        elif message.communication_type == Constants.CommunicationType.FAULT_FEEDBACK.value:
            fault_data = self._decode_fault_feedback(message.data)
            if fault_data:
                return MotorEvent(EventType.FAULT_FEEDBACK, message.can_id,
                                  fault_data, message.raw_frame)
        elif message.communication_type in [
            Constants.CommunicationType.READ_PARAM.value,
            Constants.CommunicationType.WRITE_PARAM.value
        ]:
            param_data = self._decode_param_feedback(message.data)
            if param_data:
                return MotorEvent(EventType.PARAM_FEEDBACK, message.can_id,
                                  param_data, message.raw_frame)
        else:
            logger.warning(f"未处理的通信类型: {message.communication_type}")
        return None

    def _decode_motor_feedback(self, data: bytes, extended_id: int) -> dict:
        """解码电机反馈数据。"""
        try:
            if len(data) < 8:
                raise ValueError("数据长度不足")

            # 解析29位ID中的故障信息和模式状态
            status_bits = (extended_id >> 8) & 0xFFFF  # bit23~8
            motor_can_id = status_bits & 0xFF  # bit8~15 当前电机CAN ID

            # 故障信息 bit16~21
            fault_bits = (status_bits >> 8) & 0x3F  # 6位故障信息
            faults = {
                'uncalibrated': bool(fault_bits & (1 << 5)),    # bit21 未标定
                'hall_error': bool(fault_bits & (1 << 4)),      # bit20 HALL编码故障
                'magnetic_error': bool(fault_bits & (1 << 3)),  # bit19 磁编码故障
                'overheat': bool(fault_bits & (1 << 2)),        # bit18 过温
                'overcurrent': bool(fault_bits & (1 << 1)),     # bit17 过流
                'undervoltage': bool(fault_bits & (1 << 0))     # bit16 欠压故障
            }

            # 模式状态 bit22~23
            mode_bits = (status_bits >> 14) & 0x03  # 2位模式状态
            mode_names = {0: 'Reset', 1: 'Cali', 2: 'Motor'}
            mode_name = mode_names.get(mode_bits, 'Unknown')

            # 使用大端模式解包数据
            angle_raw = int.from_bytes(data[0:2], byteorder='big', signed=False)
            speed_raw = int.from_bytes(data[2:4], byteorder='big', signed=False)
            torque_raw = int.from_bytes(data[4:6], byteorder='big', signed=False)
            temp_raw = int.from_bytes(data[6:8], byteorder='big', signed=False)

            # 将原始数据转换为物理量
            angle = uint_to_float(angle_raw, -4 * 3.14159, 4 * 3.14159, 16)
            speed = uint_to_float(speed_raw, -30, 30, 16)
            torque = uint_to_float(torque_raw, -12, 12, 16)
            temperature = temp_raw / 10.0  # 温度值需要除以10

            decoded_data = {
                "motor_can_id": motor_can_id,
                "mode_status": mode_bits,
                "mode_name": mode_name,
                "faults": faults,
                "angle": angle,
                "speed": speed,
                "torque": torque,
                "temperature": temperature
            }

            logger.debug(f"[Decode Motor Feedback] {decoded_data}")
            return decoded_data
        except Exception as e:
            logger.error(f"[Decode Motor Feedback Error] {e}")
            return {}

    def _decode_param_feedback(self, data: bytes) -> dict:
        """解码参数反馈数据。"""
        try:
            if len(data) < 8:
                raise ValueError("数据长度不足")
            # 提取参数索引（大端模式）
            param_index = int.from_bytes(data[0:2], byteorder='big', signed=False)
            # Byte2~3 为保留字节，跳过
            # 提取参数值，位于 Byte4~7
            param_value_bytes = data[4:8]
            logger.debug(f"[Param Feedback] Param Index: {hex(param_index)}, Param Value Bytes: {param_value_bytes.hex()}")

            # 根据参数类型解码
            param_info = self._get_param_info(param_index)
            param_type = param_info.get('type', 'unknown')

            if param_type == 'float':
                param_value = struct.unpack('>f', param_value_bytes)[0]
            elif param_type == 'uint16':
                param_value = int.from_bytes(param_value_bytes[:2], byteorder='big', signed=False)
            elif param_type == 'uint8':
                param_value = param_value_bytes[0]
            elif param_type == 'int16':
                param_value = int.from_bytes(param_value_bytes[:2], byteorder='big', signed=True)
            elif param_type == 'int32':
                param_value = int.from_bytes(param_value_bytes, byteorder='big', signed=True)
            elif param_type == 'uint32':
                param_value = int.from_bytes(param_value_bytes, byteorder='big', signed=False)
            elif param_type == 'string':
                # 假设字符串为 ASCII 编码，长度可变
                param_value = data[4:].decode('ascii', errors='ignore').strip('\x00')
            else:
                param_value = None
            logger.debug(f"[Param Feedback] {hex(param_index)}: {param_value}")

            return {"param_index": param_index, "param_value": param_value}
        except Exception as e:
            logger.error(f"[Decode Param Feedback Error] {e}, Data: {data.hex()}")
            return {}

    def _decode_fault_feedback(self, data: bytes) -> dict:
        """解码故障反馈数据。"""
        try:
            if len(data) < 4:
                raise ValueError("数据长度不足")
            fault_code = int.from_bytes(data[:4], byteorder='big', signed=False)
            logger.debug(f"[Decode Fault Feedback] Fault Code: {fault_code}")
            return {"fault_code": fault_code}
        except Exception as e:
            logger.error(f"[Decode Fault Feedback Error] {e}")
            return {}

    def _get_param_info(self, param_index: int) -> Dict[str, Any]:
        """根据参数索引返回参数信息，包括类型。"""
        # 从 constants.py 中的 ParamIndex 获取参数信息
        for key, value in Constants.ParamIndex.items():
            if value['index'] == param_index:
                return value
        return {}
