import asyncio
import struct
import logging
import math
from typing import Optional, Any
from constants import Constants
from protocol_handler import ProtocolHandler, MotorEvent, EventType, MotorMessage
from async_serial import AsyncSerial

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
    span = x_max - x_min
    offset = x_min
    x_clipped = min(max(x, x_min), x_max)
    uint_value = int((x_clipped - offset) * ((1 << bits) - 1) / span)
    return uint_value

def uint_to_float(uint_val: int, x_min: float, x_max: float, bits: int) -> float:
    span = x_max - x_min
    max_uint = (1 << bits) - 1
    uint_val_clipped = min(max(uint_val, 0), max_uint)
    x = (float(uint_val_clipped) * span) / max_uint + x_min
    return x

class CyberGearMotor:
    """电机控制类，使用新增的线程安全方法提高同步性。"""

    def __init__(self, can_id: int, serial_port: str, master_id: int = 0x00FD,
                 position_request_interval: float = 1/120):
        self.can_id = can_id & 0xFF
        self.master_id = master_id & 0xFF
        self.cache = {}
        self.protocol_handler = ProtocolHandler()
        self.serial_port = serial_port
        self.serial = AsyncSerial(
            port=serial_port,
            baudrate=921600,
            on_frame_received=self._on_frame_received,
        )
        self.control_task = None
        self.position_task = None
        self.position_request_interval = position_request_interval
        self.is_connected = False
        self.current_mode = Constants.RunMode['UNKNOWN']
        self.cache_lock = asyncio.Lock()

    async def connect(self):
        if self.is_connected:
            logger.info(f"电机 {self.can_id} 已经连接。")
            return

        try:
            await self.serial.connect()
            self.is_connected = True
            logger.info(f"电机 {self.can_id} 连接到串口 {self.serial_port} 成功。")
            await self.enable_motor()
            await asyncio.sleep(0.001)
            logger.info(f"电机 {self.can_id} 链接有已经使能。")
            self.current_mode = Constants.RunMode['UNKNOWN']
            self.position_task = asyncio.create_task(self._real_time_feedback_loop())
        except Exception as e:
            self.is_connected = False
            logger.error(f"电机 {self.can_id} 无法连接串口 {self.serial_port}：{e}")
            raise e

    async def close(self):
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
        await asyncio.sleep(0.001)
        await self.serial.close()
        self.is_connected = False
        logger.info(f"电机 {self.can_id} 已关闭。")

    async def send_message(self, message: MotorMessage, direction: int = Constants.DIRECTION_TO_MOTOR):
        """使用线程安全的方法进行发送"""
        frame = self.protocol_handler.encode_message(message, direction)
        # 使用新增的线程安全发送方式
        await self.serial.send_frame(frame)
        logger.debug(f"电机 {self.can_id} 发送消息: {frame.hex()}")

    async def set_zero_position(self):
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
        await asyncio.sleep(0.001)
        logger.info("已发送设置机械零位命令。")

    async def set_param(self, param_name: str, value: Any):
        try:
            index = Constants.ParamIndex[param_name]['index']
            param_type = Constants.ParamIndex[param_name]['type']
            data = bytearray(8)

            if param_type == 'uint8':
                struct.pack_into('<H2xB3x', data, 0, index, value)
            elif param_type == 'uint16':
                struct.pack_into('<H2xH2x', data, 0, index, value)
            elif param_type == 'uint32':
                struct.pack_into('<H2xI', data, 0, index, value)
            elif param_type == 'float':
                struct.pack_into('<H2xf', data, 0, index, value)
            elif param_type == 'int16':
                struct.pack_into('<H2xh', data, 0, index, value)
            else:
                logger.error(f"Unsupported parameter type for {param_name}: {param_type}")
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

            if param_name == 'RUN_MODE':
                self.current_mode = value
                await self.enable_motor()

            logger.info(f"已发送设置参数 {param_name}：{value}")
        except KeyError:
            logger.error(f"参数名称 {param_name} 未在 Constants.ParamIndex 中定义。")
        except Exception as e:
            logger.error(f"设置参数 {param_name} 时发生错误：{e}")

    async def set_run_mode(self, run_mode: int):
        await self.set_param('RUN_MODE', run_mode)

    async def set_limit_spd(self, speed: float):
        await self.set_param('LIMIT_SPD', speed)

    async def set_loc_ref(self, position: float):
        await self.set_param('LOC_REF', position)

    async def send_write_param_command(self):
        await self.set_param('SPD_KP', 1.0)

    async def reset_rotation(self):
        await self.set_param('ROTATION', 0)
        logger.info(f"电机 {self.can_id} 圈数已重置为 0。")

    async def set_position(self, position: float, speed: float = 2.0):
        if self.current_mode != Constants.RunMode['POSITION_MODE']:
            await self.stop_motor()
            await asyncio.sleep(0.001)
            await self.set_run_mode(Constants.RunMode['POSITION_MODE'])
            await asyncio.sleep(0.001)
            self.current_mode = Constants.RunMode['POSITION_MODE']
            await self.enable_motor()

        await self.set_limit_spd(speed)
        await asyncio.sleep(0.001)
        await self.set_loc_ref(position)
        await asyncio.sleep(0.001)
        logger.info(f"已发送位置指令：{position} rad")

    def degrees_to_radians(self, degrees: float):
        return degrees * (math.pi / 180.0)

    def radians_to_degrees(self, radians: float):
        return radians / (180.0 / math.pi)

    async def set_position_angle(self, angle: float, speed: float = 2.0):
        await self.set_position(self.degrees_to_radians(angle), speed)

    async def enable_motor(self):
        data = bytearray(8)
        message = MotorMessage(
            can_id=self.can_id,
            master_id=self.master_id,
            communication_type=Constants.CommunicationType['MOTOR_ENABLE'],
            data_field=self.master_id,
            data=data
        )
        await self.send_message(message)
        await asyncio.sleep(0.001)
        logger.info("电机已使能。")

    async def stop_motor(self):
        data = bytearray(8)
        message = MotorMessage(
            can_id=self.can_id,
            master_id=self.master_id,
            communication_type=Constants.CommunicationType['MOTOR_STOP'],
            data_field=self.master_id,
            data=data
        )
        await self.send_message(message)
        await asyncio.sleep(0.001)
        logger.info("电机已停止。")

    async def emergency_stop(self):
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
        try:
            while True:
                await self.send_write_param_command()
                await asyncio.sleep(self.position_request_interval)
        except asyncio.CancelledError:
            pass

    def get_real_time_position(self) -> Optional[float]:
        angle = self.cache.get('angle', None)
        if angle is not None:
            angle_normalized = ((angle + math.pi) % (2 * math.pi)) - math.pi
            logger.info(f"电机 {self.can_id} 实时位置：{angle_normalized} 弧度")
            return angle_normalized
        else:
            logger.warning(f"电机 {self.can_id} 实时位置未就绪。")
            return None

    def _on_frame_received(self, frame: bytes):
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
        if not event:
            return
        if event.event_type == EventType.MOTOR_FEEDBACK:
            self._schedule_cache_update(event.data)
        elif event.event_type == EventType.PARAM_FEEDBACK:
            param_data = {f"param_{event.data['param_index']}": event.data['param_value']}
            self._schedule_cache_update(param_data)
        elif event.event_type == EventType.FAULT_FEEDBACK:
            self._schedule_cache_update(event.data)

    def _schedule_cache_update(self, new_data: dict):
        asyncio.get_running_loop().create_task(self._async_update_cache(new_data))

    async def _async_update_cache(self, new_data: dict):
        async with self.cache_lock:
            self.cache.update(new_data)
            logger.debug(f"电机 {self.can_id} cache更新：{new_data}")
