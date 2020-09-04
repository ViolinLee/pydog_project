# -*- coding: utf-8 -*-

import math
from pydog.hal.pca9685 import PCA9685

class Servos:
    def __init__(self, i2c, address=0x40, freq=50, min_us=500, max_us=2500,  #根据舵机参数自行设置
                 degrees=180):
        self.period = 1000000 / freq  # 周期/T(s) = 1/freq，freq=50Hz-->周期/T = 20000(us) = 20(ms)
        self.min_duty = self._us2duty(min_us)
        self.max_duty = self._us2duty(max_us)
        self.degrees = degrees
        self.freq = freq
        self.pca9685 = PCA9685(i2c, address)
        self.pca9685.freq(freq)

    def _us2duty(self, value):
        # 一周期(20000us)用12位精度来表示。value(us)转换为占用周期数（分数/小数，即不到一个周期，故 value < 20000us = 20ms）
        return int(4095 * value / self.period)

    def position(self, index, degrees=None, radians=None, us=None, duty=None):
        span = self.max_duty - self.min_duty
        if degrees is not None:
            duty = self.min_duty + span * degrees / self.degrees
        elif radians is not None:
            duty = self.min_duty + span * radians / math.radians(self.degrees)
        elif us is not None:
            duty = self._us2duty(us)
        elif duty is not None:
            pass
        else:
            return self.pca9685.duty(index)
        duty = min(self.max_duty, max(self.min_duty, int(duty)))
        self.pca9685.duty(index, duty)

    def release(self, index):
        self.pca9685.duty(index, 0)

    def position_duty(self, index, degrees=None, radians=None, us=None, duty=None):
        int_dutu=int(duty)
        self.pca9685.duty(index, int_dutu)
