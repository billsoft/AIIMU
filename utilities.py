# utilities.py

"""
提供工具函数，包括数据类型转换和其他常用功能。
"""

class Utilities:
    """
    工具类，包含常用的静态方法。
    """

    @staticmethod
    def float_to_uint(x, x_min, x_max, bits):
        """
        将浮点数转换为无符号整数，用于数据打包。

        :param x: 待转换的浮点数
        :param x_min: 浮点数最小值
        :param x_max: 浮点数最大值
        :param bits: 转换后的整数位数
        :return: 转换后的无符号整数
        """
        span = x_max - x_min
        offset = x_min
        # 限制 x 的范围
        if x > x_max:
            x = x_max
        elif x < x_min:
            x = x_min
        return int((x - offset) * ((1 << bits) - 1) / span)

    @staticmethod
    def uint_to_float(x_int, x_min, x_max, bits):
        """
        将无符号整数转换为浮点数，用于数据解析。

        :param x_int: 待转换的无符号整数
        :param x_min: 浮点数最小值
        :param x_max: 浮点数最大值
        :param bits: 整数位数
        :return: 转换后的浮点数
        """
        span = x_max - x_min
        offset = x_min
        return (x_int * span) / ((1 << bits) - 1) + offset
