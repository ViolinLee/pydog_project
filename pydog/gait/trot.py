# -*- coding: utf-8 -*-

from math import pi, sin, cos
from pydog.config import *
from collections import deque


def gen_toe_path(cycle_length, duty_ratio, inc, front, back, height, reverse):
    xz_coord = []
    t = 0
    span = back - front
    first_duration = cycle_length * duty_ratio
    second_duration = cycle_length * (1 - duty_ratio)
    switch_position = int(first_duration / inc) + 1
    while True:
        if t <= first_duration:
            sigma = 2 * pi * t / first_duration
            x_tmp = back - span / (2 * pi) * (sigma - sin(sigma))
            y_tmp = height / 2 * (1 - cos(sigma))

        elif first_duration < t < cycle_length:
            sigma = 2 * pi * (t - first_duration) / second_duration
            x_tmp = front + span / (2 * pi) * (sigma - sin(sigma))
            y_tmp = 0
        else:
            break

        xz_coord.append([x_tmp, y_tmp])
        t += inc

    if reverse:
        xz_coord.reverse()
        switch_position = int(cycle_length / inc - switch_position)

    return xz_coord, switch_position


def trot_gait(reverse=False):
    fl_xz_path, switch_pos = gen_toe_path(trot_cycle_length, trot_duty_ratio, time_increment, x_front, x_back,
                                            lift_height, reverse)
    fr_xz_path = deque(fl_xz_path)
    fr_xz_path.rotate(-switch_pos)

    br_xz_path = fl_xz_path
    bl_xz_path = fr_xz_path

    return fl_xz_path, fr_xz_path, br_xz_path, bl_xz_path


def turn_gait(left=True):
    fl_xz_path, switch_pos = gen_toe_path(trot_cycle_length, trot_duty_ratio, time_increment, x_front, x_back,
                                           lift_height, reverse=left)
    bl_xz_path = deque(fl_xz_path)
    bl_xz_path.rotate(-switch_pos)

    fr_xz_path, switch_pos = gen_toe_path(trot_cycle_length, trot_duty_ratio, time_increment, x_front, x_back,
                                           lift_height, reverse=not left)
    br_xz_path = deque(fr_xz_path)
    br_xz_path.rotate(-switch_pos)

    return fl_xz_path, fr_xz_path, br_xz_path, bl_xz_path


def jump_gait():
    return


if __name__ == '__main__':
    import matplotlib.pyplot as plt

    fl_xz_path, fr_xz_path, br_xz_path, bl_xz_path = trot_gait()

    fl_x_path = [coord[0] for coord in fl_xz_path]
    fl_z_path = [coord[1] for coord in fl_xz_path]

    ax1 = plt.subplot(4, 2, 1)
    plt.plot(fl_x_path, 'r--')
    plt.plot(fl_z_path, 'g--')

    ax2 = plt.subplot(4, 2, 2)
    plt.plot(fl_x_path, fl_z_path, 'r--')

    fr_x_path = [coord[0] for coord in fr_xz_path]
    fr_z_path = [coord[1] for coord in fr_xz_path]

    ax3 = plt.subplot(4, 2, 3)
    plt.plot(fr_x_path, 'r--')
    plt.plot(fr_z_path, 'g--')

    ax4 = plt.subplot(4, 2, 4)
    plt.plot(fr_x_path, fr_z_path, 'r--')

    fl_x_path = [coord[0] for coord in fl_xz_path]
    fl_z_path = [coord[1] for coord in fl_xz_path]

    ax5 = plt.subplot(4, 2, 5)
    plt.plot(fl_x_path, 'r--')
    plt.plot(fl_z_path, 'g--')

    ax6 = plt.subplot(4, 2, 6)
    plt.plot(fl_x_path, fl_z_path, 'r--')

    fr_x_path = [coord[0] for coord in fr_xz_path]
    fr_z_path = [coord[1] for coord in fr_xz_path]

    ax7 = plt.subplot(4, 2, 7)
    plt.plot(fr_x_path, 'r--')
    plt.plot(fr_z_path, 'g--')

    ax8 = plt.subplot(4, 2, 8)
    plt.plot(fr_x_path, fr_z_path, 'r--')

    plt.show()