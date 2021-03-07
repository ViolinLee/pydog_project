# -*- coding: utf-8 -*-

from math import pi, sin, cos


def gen_toe_path(cycle_length, duty_ratio, inc, front, back, height, reverse):
    """
    Note: The toe's z coord is in reversed direction and ignore the body height.
    """
    toe_xz_coord = []
    t = 0
    span = front - back
    first_duration = cycle_length * duty_ratio
    second_duration = cycle_length * (1 - duty_ratio)
    switch_position = int(first_duration / inc) + 1
    while True:
        if t <= first_duration:
            sigma = 2 * pi * t / first_duration
            x_tmp = back + span / (2 * pi) * (sigma - sin(sigma))
            z_tmp = height / 2 * (1 - cos(sigma))

        elif first_duration < t < cycle_length:
            sigma = 2 * pi * (t - first_duration) / second_duration
            x_tmp = front - span / (2 * pi) * (sigma - sin(sigma))
            z_tmp = 0
        else:
            break

        toe_xz_coord.append([-x_tmp, z_tmp])  # Note The IK Model's x dir is in reverse direction.
        t += inc

    if reverse:
        toe_xz_coord.reverse()
        switch_position = int(cycle_length / inc - switch_position)

    return toe_xz_coord, switch_position


def toe_path_rectify(body_height, fr_path, br_path, bl_path, fl_path):
    for i, fr_coord in enumerate(fr_path):
        fr_coord[1] = body_height - fr_path[i][1]
        br_path[i][1] = body_height - br_path[i][1]
        bl_path[i][1] = body_height - bl_path[i][1]
        fl_path[i][1] = body_height - fl_path[i][1]

    print(fr_path)


def trot_gait(body_height, trot_cycle_length, trot_duty_ratio, path_increment, x_front, x_back, lift_height, reverse=False):
    print('In trot')
    fl_path, switch_pos = gen_toe_path(trot_cycle_length, trot_duty_ratio, path_increment, x_front, x_back,
                                          lift_height, reverse)
    # fr_path = deque(fl_path)
    # fr_path.rotate(-switch_pos)
    fr_path = [list(tmp) for tmp in fl_path[switch_pos:]] + [list(tmp) for tmp in fl_path[:switch_pos]]
    br_path = [list(tmp) for tmp in fl_path]
    bl_path = [list(tmp) for tmp in fr_path]

    toe_path_rectify(body_height, fr_path, br_path, bl_path, fl_path)

    return fr_path, br_path, bl_path, fl_path


def gallup_gait(body_height, trot_cycle_length, trot_duty_ratio, path_increment, x_front, x_back, lift_height, reverse=False):
    print('In gallup')
    fl_path, switch_pos = gen_toe_path(trot_cycle_length, trot_duty_ratio, path_increment, x_front, x_back,
                                          lift_height, reverse)
    fr_path = [list(tmp) for tmp in fl_path]
    bl_path = [list(tmp) for tmp in fl_path[switch_pos:]] + [list(tmp) for tmp in fl_path[:switch_pos]]
    br_path = [list(tmp) for tmp in bl_path]

    toe_path_rectify(body_height, fr_path, br_path, bl_path, fl_path)

    return fr_path, br_path, bl_path, fl_path


def turn_gait(body_height, trot_cycle_length, trot_duty_ratio, path_increment, x_front, x_back, lift_height, reverse=True):
    fl_path, switch_pos = gen_toe_path(trot_cycle_length, trot_duty_ratio, path_increment, x_front, x_back,
                                          lift_height, not reverse)
    bl_path = [list(tmp) for tmp in fl_path[switch_pos:]] + [list(tmp) for tmp in fl_path[:switch_pos]]
    fr_path, switch_pos = gen_toe_path(trot_cycle_length, trot_duty_ratio, path_increment, x_front, x_back,
                                          lift_height, reverse)
    br_path = [list(tmp) for tmp in fr_path[switch_pos:]] + [list(tmp) for tmp in fr_path[:switch_pos]]

    toe_path_rectify(body_height, fr_path, br_path, bl_path, fl_path)

    return fr_path, br_path, bl_path, fl_path


def jump_gait():
    return


if __name__ == '__main__':
    import matplotlib.pyplot as plt

    fr_xz_path, br_xz_path, bl_xz_path, fl_xz_path = gallup_gait(100)

    """Frontal Left"""
    fl_x_path = [coord[0] for coord in fl_xz_path]
    fl_z_path = [coord[1] for coord in fl_xz_path]

    ax1 = plt.subplot(4, 2, 1)
    plt.plot(fl_x_path, 'r--')
    plt.plot(fl_z_path, 'g--')

    ax2 = plt.subplot(4, 2, 2)
    plt.plot(fl_x_path, fl_z_path, 'r--')

    """Frontal right"""
    fr_x_path = [coord[0] for coord in fr_xz_path]
    fr_z_path = [coord[1] for coord in fr_xz_path]

    ax3 = plt.subplot(4, 2, 3)
    plt.plot(fr_x_path, 'r--')
    plt.plot(fr_z_path, 'g--')

    ax4 = plt.subplot(4, 2, 4)
    plt.plot(fr_x_path, fr_z_path, 'r--')

    """Behind Left"""
    bl_x_path = [coord[0] for coord in bl_xz_path]
    bl_z_path = [coord[1] for coord in bl_xz_path]

    ax5 = plt.subplot(4, 2, 5)
    plt.plot(bl_x_path, 'r--')
    plt.plot(bl_z_path, 'g--')

    ax6 = plt.subplot(4, 2, 6)
    plt.plot(bl_x_path, bl_z_path, 'r--')

    """Behind right"""
    br_x_path = [coord[0] for coord in br_xz_path]
    br_z_path = [coord[1] for coord in br_xz_path]

    ax7 = plt.subplot(4, 2, 7)
    plt.plot(br_x_path, 'r--')
    plt.plot(br_z_path, 'g--')

    ax8 = plt.subplot(4, 2, 8)
    plt.plot(br_x_path, br_z_path, 'r--')

    plt.show()
