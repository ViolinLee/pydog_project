# -*- coding: utf-8 -*-

from math import pi, cos, sin, acos, atan2, sqrt


def ik_engine(l1, l2, xz_coord):
    """Note the upside-down legs"""
    x = -xz_coord[0]  # 推导逆运动学的坐标系与实际坐标系有相反的X轴
    z = xz_coord[1]

    tibia = pi - acos((x ** 2 + z ** 2 - l1 ** 2 - l2 ** 2) / (-2 * l1 * l2))
    fai = acos((l1 ** 2 + x ** 2 + z ** 2 - l2 ** 2) / (2 * l1 * sqrt(x ** 2 + z ** 2)))
    alpha = atan2(z, x)
    femur = alpha - fai

    tibia = 180 * tibia / pi
    femur = 180 * femur / pi

    return femur, tibia


def fk_engine(l1, l2, femur_tibia_angles):
    femur = femur_tibia_angles[0] * pi / 180
    tibia = femur_tibia_angles[1] * pi / 180

    x = -(l1 * cos(femur) - l2 * cos(pi - tibia - femur))  # 推导逆运动学的坐标系与实际坐标系有相反方向的X轴
    z = l1 * sin(femur) + l2 * sin(pi - tibia - femur)
    xz_coord = [x, z]

    return xz_coord
