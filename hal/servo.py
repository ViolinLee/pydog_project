"""Adapted from https://bitbucket.org/thesheep/micropython-servo/src/f562a6abeaf0e83b752838df7cd31d88ea10b2c7/servo.py?at=default&fileviewer=file-view-default
"""
from machine import PWM, Pin
import math


class DServo:
    """
    A simple class for controlling hobby servos.

    Args:
        pin (machine.Pin): The pin where servo is connected. Must support PWM.
        freq (int): The frequency of the signal, in hertz.
        min_us (int): The minimum signal length supported by the servo.
        max_us (int): The maximum signal length supported by the servo.
        angle (int): The angle between the minimum and maximum positions.

    """
    def __init__(self, pin, freq=50, min_us=600, max_us=2400, degrees=180):
        self.min_us = min_us
        self.max_us = max_us
        self.us = 0
        self.freq = freq
        self.degrees = degrees
        self.pwm = PWM(pin, freq=freq, duty=0)

    def write_us(self, us):
        """Set the signal to be ``us`` microseconds long. Zero disables it."""
        if us == 0:
            self.pwm.duty(0)
            return
        us = min(self.max_us, max(self.min_us, int(us)))
        duty = us * 1024 * self.freq // 1000000
        self.pwm.duty(duty)

    def write_angle(self, degrees=None, radians=None):
        """Move to the specified angle in ``degrees`` or ``radians``."""
        if degrees is None:
            degrees = math.degrees(radians)
        degrees = degrees % 360
        
        total_range = self.max_us - self.min_us
        us = self.min_us + total_range * degrees / self.degrees
        self.write_us(us)


class Servos:
    def __init__(self, pins_list, freq=50, min_us=600, max_us=2400, degrees=180):
        self.direct_servos = [DServo(Pin(pin_num), freq, min_us, max_us, degrees) for pin_num in pins_list]
        self.pin_nums = pins_list

    def position(self, servo_id, degrees=None, radians=None):
        self.direct_servos[servo_id].write_angle(degrees=degrees, radians=radians)



