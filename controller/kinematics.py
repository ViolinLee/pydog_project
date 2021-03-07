# -*- coding: utf-8 -*-

from math import pi, cos, sin, acos, atan2, sqrt


def ik_engine(l1, l2, xz_coord):
    """Note the upside-down legs"""
    x = xz_coord[0]
    z = xz_coord[1]

    tibia = pi - acos((l1 ** 2 + l2 ** 2 - x ** 2 - z ** 2) / (2 * l1 * l2))  # acos变量取值范围为[0, pi]
    fai = acos((l1 ** 2 + x ** 2 + z ** 2 - l2 ** 2) / (2 * l1 * sqrt(x ** 2 + z ** 2)))
    alpha = atan2(z, x)
    femur = alpha - fai

    tibia = 180 * tibia / pi
    femur = 180 * femur / pi

    return femur, tibia


def fk_engine(l1, l2, femur_tibia_angles):
    femur = femur_tibia_angles[0] * pi / 180
    tibia = femur_tibia_angles[1] * pi / 180

    x = l1 * cos(femur) - l2 * cos(pi - tibia - femur)
    z = l1 * sin(femur) + l2 * sin(pi - tibia - femur)
    xz_coord = [x, z]

    return xz_coord


def translate_coord(axis, offset, current_coord):
    if axis == 0:
        fr_coord, br_coord, bl_coord, fl_coord = [[current_coord[i][0] + offset, current_coord[i][1]]
                                                  for i in range(4)]
    else:
        fr_coord, br_coord, bl_coord, fl_coord = [[current_coord[i][0], current_coord[i][1] + offset]
                                                  for i in range(4)]

    return fr_coord, br_coord, fl_coord, bl_coord


def rotate_coord(height, half_bl, half_bw, roll, pitch):
    # Frontal Left
    fl_x = half_bl - half_bl * cos(pitch)
    fl_z = ((height + half_bw * sin(roll) / 2) + half_bl * sin(pitch)) / cos(roll)
    # Frontal Right
    fr_x = fl_x
    fr_z = ((height - half_bw * sin(roll) / 2) + half_bl * sin(pitch)) / cos(roll)
    # Behind Left
    bl_x = half_bl * cos(pitch) - half_bl
    bl_z = ((height + half_bw * sin(roll) / 2) - half_bl * sin(pitch)) / cos(roll)
    # Behind Right
    br_x = bl_x
    br_z = ((height - half_bw * sin(roll) / 2) - half_bl * sin(pitch)) / cos(roll)

    fr_coord = [fr_x, fr_z]
    br_coord = [br_x, br_z]
    fl_coord = [fl_x, fl_z]
    bl_coord = [bl_x, bl_z]

    return fr_coord, br_coord, fl_coord, bl_coord


if __name__ == '__main__':
    coord = [0.0050354, 145.9822]  # [0.0050354, 145.9822] 报错
    l1 = 80
    l2 = 62.5
    print(ik_engine(l1, l2, coord))
