# -*- coding: utf-8

from pydog.config import *
from pydog.hal.servo import Servos
from math import pi, acos, atan2, sqrt


class Pydog(object):
    def __init__(self, l1, l2, joints_ids, init_joints_angles, init_servos_pos, calibration_txt,
                 pca_i2c_scl, pca_i2c_sda, pca_i2c_freq, pca_i2c_adr):
        # initialize legs angles
        pca_i2c = I2C(pca_i2c_scl, pca_i2c_sda, pca_i2c_freq)

        self.l1 = l1
        self.l2 = l2
        self.servos = Servos(pca_i2c, address=pca_i2c_adr)
        self.init_servos_pos = list(init_servos_pos)
        self.joints_angles = list(init_joints_angles)
        self.joints_ids = joints_ids
        self.calibration_txt = calibration_txt
        self.calibrate()
        self.move_leg()

    def calibrate(self):
        self.move_leg()
        print("Welcome to PyDog's Legs Calibration System!")
        mode = input("Press Enter to start the calibration, "
                     "or input any other keys then press Enter to load the last calibration data>>")
        servos_codes = ['fr0', 'fr1', 'br0', 'br1', 'bl0', 'bl1', 'fl0', 'fl1']
        if mode == "":
            # init_servo_pos_dict = dict(zip(servos_codes, self.init_servos_pos))
            inc_servo_pos_dict = dict(zip(servos_codes, [0]*8))
            while True:
                servo_code = input("Please input the servo code to be calibrated, "
                                   "or 'q' to quit, 'p' to view prompts>>")
                if servo_code in servos_codes:
                    servo_id = servos_codes.index(servo_code)
                    servo_inc = input("Input '+' or '-' then press Enter to set up the servo position>>")
                    if servo_inc == '+':
                        inc_servo_pos_dict[servo_code] += 1
                        self.init_servos_pos[servo_id] += 1
                        self.move_leg()
                    elif servo_inc == '-':
                        inc_servo_pos_dict[servo_code] -= 1
                        self.init_servos_pos[servo_id] -= 1
                        self.move_leg()
                    elif servo_inc == 'q':
                        break
                    else:
                        print("Error input! Please try again>>")
                elif servo_code == 'q':
                    print("Calibration Result (Initial Servos Position):\n", self.init_servos_pos)
                    break
                elif servo_code == 'p':
                    print("fr0: frontal right femur\n"
                          "fr1: frontal right tibia")
                else:
                    print("Error input! Please try again.")
                    continue
                if sum([abs(val) for val in inc_servo_pos_dict.values()]) > 0:
                    save_calibration_data(self.calibration_txt, inc_servo_pos_dict)
                else:
                    inc_servo_pos_dict = load_calibration_data(self.calibration_txt)
                    self.init_servos_pos = [self.init_servos_pos[i] + inc_servo_pos_dict[servo_code]
                                            for i, servo_code in enumerate(servos_codes)]

    def joint2servo(self, joints_angles):
        upper_operators = [1, 1, -1, -1]
        lower_operators = [[1, -1], [1, -1], [-1, 1], [-1, 1]]

        servos_pos = []
        for i, joint_angle in enumerate(joints_angles):
            if i % 2 == 0:
                operator = upper_operators[int(i / 2)]
                upper_servo_pos = round(self.init_servos_pos[i] + operator * joint_angle, 4)
                servos_pos.append(upper_servo_pos)
            else:
                operators = lower_operators[int(i / 2)]
                lower_servo_pos = round(self.init_servos_pos[i] + operators[0] * 90 + operators[1] * joint_angle, 4)
                servos_pos.append(lower_servo_pos)

        return servos_pos

    def ik_calculate(self, fr_coord, br_coord, bl_coord, fl_coord):
        angles_list = []
        for i, coord in enumerate([fr_coord, br_coord, bl_coord, fl_coord]):
            femur, tieba = ik_engine(self.l1, self.l2, coord)
            angles_list.append(femur)
            angles_list.append(tieba)

        self.joints_angles = angles_list
        print("Inverse Kinematic Calculation Result\n:", self.joints_angles)

    def move_leg(self):
        servos_pos = self.joint2servo(self.joints_angles)
        print("Move Leg by Driving Servos to\n:", servos_pos)
        for i, leg_id in enumerate(self.joints_ids):
            self.servos.position(leg_id, degrees=servos_pos[i])


def ik_engine(l1, l2, xy_coord):
    x = -xy_coord[0]
    y = xy_coord[1]

    tibia = pi - acos((x ** 2 + y ** 2 - l1 ** 2 - l2 ** 2) / (-2 * l1 * l2))
    fai = acos((l1 ** 2 + x ** 2 + y ** 2 - l2 ** 2) / (2 * l1 * sqrt(x ** 2 + y ** 2)))
    alpha = atan2(y, x)
    femur = alpha - fai

    tibia = 180 * tibia / pi
    femur = 180 * femur / pi

    return femur, tibia


def load_calibration_data(calibration_txt):
    data_dict = {}
    with open(calibration_txt, 'r') as f:
        for line in f.readlines():
            (key, val) = line.split()
            data_dict[key] = int(val)
    print("Successfully Loaded Calibration Data: ", data_dict)
    return data_dict


def save_calibration_data(calibration_txt, data_dict):
    with open(calibration_txt, 'w') as f:
        for key, values in data_dict.items():
            f.write(key + '\t' + str(values) + '\n')
    print("Successfully Saved Calibration Data: ", data_dict)


