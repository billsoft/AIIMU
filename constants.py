# constants.py

import json
import os

class Constants:
    """
    包含电机参数范围、通信类型、运行模式和参数索引的常量定义。
    """

    # 从 constants.json 文件中加载常量数据
    constants_file = os.path.join(os.path.dirname(__file__), 'constants.json')
    with open(constants_file, 'r', encoding='utf-8') as f:
        constants_data = json.load(f)

    # 电机参数范围
    P_MIN = constants_data["P_MIN"]
    P_MAX = constants_data["P_MAX"]
    V_MIN = constants_data["V_MIN"]
    V_MAX = constants_data["V_MAX"]
    T_MIN = constants_data["T_MIN"]
    T_MAX = constants_data["T_MAX"]
    KP_MIN = constants_data["KP_MIN"]
    KP_MAX = constants_data["KP_MAX"]
    KD_MIN = constants_data["KD_MIN"]
    KD_MAX = constants_data["KD_MAX"]

    # 数据位数
    POSITION_BITS = constants_data["POSITION_BITS"]
    ROTATION_BITS = constants_data["ROTATION_BITS"]

    # 通信方向
    DIRECTION_TO_MOTOR = constants_data["DIRECTION_TO_MOTOR"]
    DIRECTION_FROM_MOTOR = constants_data["DIRECTION_FROM_MOTOR"]

    # 通信类型
    CommunicationType = constants_data["CommunicationType"]

    # 运行模式
    RunMode = constants_data["RunMode"]

    # 参数索引
    ParamIndex = constants_data["ParamIndex"]
