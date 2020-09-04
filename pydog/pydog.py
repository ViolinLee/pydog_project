# -*- coding: utf-8

from pydog.config import *
from pydog.hal.servo import Servos
from math import pi, acos, atan, sqrt


class PyDog(object):
    def __init__(self, femur1, femur2, femur3, femur4, tibia1, tibia2, tibia3, tibia4):
        # initialize legs angles
        pca_i2c = I2C(pca_i2c_scl, pca_i2c_sda, pca_i2c_freq)

        self.servos = Servos(pca_i2c, address=pca_i2c_adr)
        self.legs_angles = (femur1, tibia1, femur2, tibia2, femur3, tibia3, femur4, tibia4)
        self.legs_ids = (4, 5, 6, 7, 8, 9, 10, 11)
        self.move_leg()

    def ik_calculate(self, fr_coord, br_coord, bl_coord, fl_coord):
        angles_list = []
        for coord in [fr_coord, br_coord, bl_coord, fl_coord]:
            femur, tibia = ik_engine(coord)
            angles_list += [femur, tibia]
            self.legs_angles = tuple(angles_list)

    def move_leg(self):
        for i, leg_id in enumerate(self.legs_ids):
            self.servos.position(leg_id, degrees=self.legs_angles[i])


def ik_engine(xy_coord):
    x = -xy_coord[0]
    y = xy_coord[1]

    tibia = pi - acos((x * x + y * y - l1 * l1 - l2 * l2) / (-2 * l1 * l2))
    fai = acos((l1 * l1 + x * x + y * y - l2 * l2) / (2 * l1 * sqrt(x * x + y * y)))
    if x > 0:
        femur = abs(atan(y / x)) - fai
    elif x < 0:
        femur = pi - abs(atan(y / x)) - fai
    else:
        femur = pi - 1.5707 - fai
    tibia = 180 * tibia / pi
    femur = 180 * femur / pi

    return femur, tibia
