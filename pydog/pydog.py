# -*- coding: utf-8

from math import pi, cos, sin
from pydog.hal.servo import Servos
from machine import I2C
from pydog.controller.kinematics import fk_engine, ik_engine
from pydog.utils import load_calibration_data, save_calibration_data
from pydog.gait import *


class PyDog(object):
    def __init__(self, body_l, body_w, l1, l2,
                 joints_ids, init_joints_angles, init_servos_pos, calibration_txt,
                 pca_i2c_scl, pca_i2c_sda, pca_i2c_freq, pca_i2c_adr):
        # initialize legs angles
        pca_i2c = I2C(scl=pca_i2c_scl, sda=pca_i2c_sda, freq=pca_i2c_freq)

        self.body_l = body_l
        self.body_w = body_w
        self.l1 = l1
        self.l2 = l2
        self.servos = Servos(pca_i2c, address=pca_i2c_adr)
        self.joints_ids = joints_ids
        self.init_servos_pos = list(init_servos_pos)
        self.init_joints_angles = list(init_joints_angles)
        """The coordinates of the leg tip relative to the its first joint (X-Front Z-Down)"""
        self.init_coord = self.fk_calculate(init_joints_angles)

        self.expected_joints_angles = self.init_joints_angles
        self.expected_coord = self.init_coord

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
        lower_operators = [-1, -1, 1, 1]

        servos_pos = []
        for i, joint_angle in enumerate(joints_angles):
            if i % 2 == 0:
                operator = upper_operators[int(i / 2)]
                upper_servo_pos = round(self.init_servos_pos[i] + operator*(joint_angle-self.init_joints_angles[i]), 4)
                servos_pos.append(upper_servo_pos)
            else:
                operator = lower_operators[int(i / 2)]
                lower_servo_pos = round(self.init_servos_pos[i] + operator*(joint_angle-self.init_joints_angles[i]), 4)
                servos_pos.append(lower_servo_pos)

        return servos_pos

    def ik_calculate(self, fr_coord, br_coord, bl_coord, fl_coord):
        angles_list = []
        for i, coord in enumerate([fr_coord, br_coord, bl_coord, fl_coord]):
            femur, tieba = ik_engine(self.l1, self.l2, coord)
            angles_list.append(femur)
            angles_list.append(tieba)

        self.expected_joints_angles = angles_list
        self.expected_coord = [fr_coord, br_coord, bl_coord, fl_coord]
        print("Inverse Kinematic Calculation Result\n:", self.expected_joints_angles)

    def fk_calculate(self, joints_angles):
        coords = []
        for i in range(0, len(joints_angles), 2):
            xy_coord = fk_engine(self.l1, self.l2, joints_angles[i:i + 2])
            coords.append(xy_coord)
        return coords

    def move_leg(self):
        servos_pos = self.joint2servo(self.expected_joints_angles)
        print("Move Leg by Driving Servos to\n:", servos_pos)

        for i, leg_id in enumerate(self.joints_ids):
            self.servos.position(leg_id, degrees=servos_pos[i])

    def move2start(self):
        self.expected_joints_angles = self.init_joints_angles
        self.expected_coord = self.init_coord
        self.move_leg()

    def process_trans(self, axis, offset, elapsed=50, duration=1000):
        remain_time = duration
        if axis == 0:
            fr_coord, br_coord, bl_coord, fl_coord = [[self.expected_coord[i][0] + offset, self.expected_coord[i][1]]
                                                      for i in range(4)]
        else:
            fr_coord, br_coord, bl_coord, fl_coord = [[self.expected_coord[i][0], self.expected_coord[i][1] + offset]
                                                      for i in range(4)]

        inter_fr_coord, inter_br_coord, inter_bl_coord, inter_fl_coord = self.expected_coord
        while elapsed <= remain_time:
            ratio = elapsed / remain_time
            for i in range(2):
                inter_fr_coord[i] += (fr_coord[i] - inter_fr_coord[i]) * ratio
                inter_br_coord[i] += (br_coord[i] - inter_br_coord[i]) * ratio
                inter_bl_coord[i] += (bl_coord[i] - inter_bl_coord[i]) * ratio
                inter_fl_coord[i] += (fl_coord[i] - inter_fl_coord[i]) * ratio

            self.ik_calculate(inter_fr_coord, inter_br_coord, inter_bl_coord, inter_fl_coord)
            self.move_leg()

            remain_time -= elapsed

    def process_rotate(self, roll_pitch_angles, elapsed=50, duration=1000):
        half_bl = self.body_l/2
        half_bw = self.body_w/2
        bh = self.init_coord[0][1]

        remain_time = duration
        roll, pitch = [angle * pi / 180 for angle in roll_pitch_angles]  # 8DOF dog has yaw angle equal 0.

        # Frontal Left
        fl_x = half_bl - half_bl * cos(pitch)
        fl_z = bh + half_bw*sin(roll)/2 + half_bl*cos(roll)*sin(pitch)
        # Frontal Right
        fr_x = fl_x
        fr_z = bh - half_bw*sin(roll)/2 + half_bl*cos(roll)*sin(pitch)
        # Behind Left
        bl_x = half_bl * cos(pitch) - half_bl
        bl_z = bh + half_bw*sin(roll)/2 - half_bl*cos(roll)*sin(pitch)
        # Behind Right
        br_x = bl_x
        br_z = bh - half_bw*sin(roll)/2 - half_bl*cos(roll)*sin(pitch)

        fr_coord = [fr_x, fr_z]
        br_coord = [br_x, br_z]
        fl_coord = [fl_x, fl_z]
        bl_coord = [bl_x, bl_z]
        inter_fr_coord, inter_br_coord, inter_bl_coord, inter_fl_coord = self.expected_coord
        while elapsed <= remain_time:
            ratio = elapsed / remain_time
            for i in range(2):
                inter_fr_coord[i] += (fr_coord[i] - inter_fr_coord[i]) * ratio
                inter_br_coord[i] += (br_coord[i] - inter_br_coord[i]) * ratio
                inter_bl_coord[i] += (bl_coord[i] - inter_bl_coord[i]) * ratio
                inter_fl_coord[i] += (fl_coord[i] - inter_fl_coord[i]) * ratio
            self.ik_calculate(inter_fr_coord, inter_br_coord, inter_bl_coord, inter_fl_coord)
            self.move_leg()
            remain_time -= elapsed

    def process_demo_gait(self, mode, elapsed):

        return

    def update_state(self):
        """主程序里控制器每调节一次之前需要确保状态（读取和处理传感器数据）已更新"""
        return
