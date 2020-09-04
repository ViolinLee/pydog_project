# -*- coding: utf-8

from pydog.config import *
from pydog.hal.servo import Servos
from math import pi, acos, atan, sqrt


class PaDog(object):
    def __init__(self, femur1, femur2, femur3, femur4, tieba1, tieba2, tieba3, tieba4):
        # initialize legs angles
        pca_i2c = I2C(pca_i2c_scl, pca_i2c_ada, pca_i2c_freq)

        self.servos = Servos(pca_i2c, address=pca_i2c_adr)
        self.legs_angles = (femur1, tieba1, femur2, tieba2, femur3, tieba3, femur4, tieba4)
        self.legs_ids = (4, 5, 6, 7, 8, 9, 10, 11)
        self.move_leg()

    def ik_calculate(self, fr_coord, br_coord, bl_coord, fl_coord):
        angles_list = []
        for coord in [fr_coord, br_coord, bl_coord, fl_coord]:
            femur, tieba = ik_engine(coord)
            angles_list += [femur, tieba]
            self.legs_angles = tuple(angles_list)

    def move_leg(self):
        for i, leg_id in enumerate(self.legs_ids):
            self.servos.position(leg_id, degrees=self.legs_angles[i])


def ik_engine(x, y):
    x = -x
    tieba = pi - acos((x * x + y * y - l1 * l1 - l2 * l2) / (-2 * l1 * l2))
    fai = acos((l1 * l1 + x * x + y * y - l2 * l2) / (2 * l1 * sqrt(x * x + y * y)))
    if x > 0:
        femur = abs(atan(y / x)) - fai
    elif x < 0:
        femur = pi - abs(atan(y / x)) - fai
    else:
        femur = pi - 1.5707 - fai
    tieba = 180 * tieba / pi
    femur = 180 * femur / pi

    return femur, tieba
