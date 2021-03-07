# -*- coding: utf-8

from math import pi, cos, sin, atan
from utime import sleep_us, sleep_ms
from pydog.hal.servo import Servos
from pydog.controller.kinematics import fk_engine, ik_engine, rotate_coord, translate_coord
from pydog.utils import load_calibration_data, save_calibration_data
from pydog.gait.gait import trot_gait, turn_gait, gallup_gait, jump_gait


class PyDog(object):
    def __init__(self, body_l, body_w, l1, l2,
                 joints_ids, calibrate_joints_angles, calibrate_servos_pos, calibration_txt, init_joints_angles, html_str):

        self.mode = 0
        self.gait = 0
        self.forward_reverse = False
        self.left_reverse = False
        self.step_reset = True
        self.up_down_tag = False
        self.left_right_tag = False
        self.set_height = 100
        self.walking_times = 5
        self.body_l = body_l
        self.body_w = body_w
        self.l1 = l1
        self.l2 = l2
        self.servos = Servos(joints_ids)
        self.joints_ids = joints_ids
        self.init_joints_angles = init_joints_angles[:]
        self.cali_joints_angles = calibrate_joints_angles[:]
        self.cali_servos_pos = list(calibrate_servos_pos[:])
        self.calibration_txt = calibration_txt
        self.calibrate(load=True)
        """The coordinates of the leg tip relative to the its first joint (X-Front Z-Down)"""
        self.init_coord = self.fk_calculate(init_joints_angles)
        self.expected_joints_angles = init_joints_angles[:]  # Avoid shallow copy
        self.expected_coord = self.init_coord[:]
        self.panel_html = html_str

        self.move_leg(None)

    def calibrate(self, load):
        print("Welcome to PyDog's Legs Calibration System!")
        servos_codes = ['fr0', 'fr1', 'br0', 'br1', 'bl0', 'bl1', 'fl0', 'fl1']

        if load:
            inc_servo_pos_dict = load_calibration_data(self.calibration_txt)
            self.cali_servos_pos = [self.cali_servos_pos[i] + inc_servo_pos_dict[servo_code]
                                    for i, servo_code in enumerate(servos_codes)]
        else:
            self.expected_joints_angles = self.cali_joints_angles[:]
            self.move_leg(self.cali_servos_pos)
            inc_servo_pos_dict = dict(zip(servos_codes, [0]*8))
            while True:
                servo_code = input("Please input the servo code to be calibrated, "
                                   "or 'q' to quit, 'p' to view prompts>>")
                if servo_code in servos_codes:
                    servo_id = servos_codes.index(servo_code)
                    servo_inc = input("Input '+' or '-' then press Enter to set up the servo position>>")
                    if servo_inc == '+':
                        inc_servo_pos_dict[servo_code] += 1
                        self.cali_servos_pos[servo_id] += 1
                        self.move_leg(self.cali_servos_pos)
                    elif servo_inc == '-':
                        inc_servo_pos_dict[servo_code] -= 1
                        self.cali_servos_pos[servo_id] -= 1
                        self.move_leg(self.cali_servos_pos)
                    elif servo_inc == 'q':
                        break
                    else:
                        print("Error input! Please try again>>")
                elif servo_code == 'q':
                    print("Calibration Result (Initial Servos Position):\n", self.cali_servos_pos)
                    break
                elif servo_code == 'p':
                    print("fr0: frontal right femur\n"
                          "fr1: frontal right tibia")
                else:
                    print("Error input! Please try again.")
                    continue

            if sum([abs(val) for val in inc_servo_pos_dict.values()]) > 0:
                save_calibration_data(self.calibration_txt, inc_servo_pos_dict)

    def joint2servo(self, joints_angles):
        upper_operators = [1, 1, -1, -1]
        lower_operators = [-1, -1, 1, 1]

        servos_pos = []
        for i, joint_angle in enumerate(joints_angles):
            if i % 2 == 0:
                operator = upper_operators[int(i / 2)]
                upper_servo_pos = round(self.cali_servos_pos[i] + operator*(joint_angle-self.cali_joints_angles[i]), 4)
                servos_pos.append(upper_servo_pos)
            else:
                operator = lower_operators[int(i / 2)]
                lower_servo_pos = round(self.cali_servos_pos[i] + operator*(joint_angle-self.cali_joints_angles[i]), 4)
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
        # print("Inverse Kinematic Calculation Result\n:", self.expected_joints_angles)

    def fk_calculate(self, joints_angles):
        coords = []
        for i in range(0, len(joints_angles), 2):
            xy_coord = fk_engine(self.l1, self.l2, joints_angles[i:i + 2])
            coords.append(xy_coord)
        return coords

    def move_leg(self, servos_poses):
        # When not given servos poses, calculate according to expected joints angles and reference servos poses
        if servos_poses is None:
            servos_poses = self.joint2servo(self.expected_joints_angles)
        
        # print("Move Leg by Driving Servos to\n:", servos_poses)
        for i, leg_id in enumerate(self.joints_ids):            
            self.servos.position(i, degrees=servos_poses[i])
            sleep_us(500)  # Avoid seervo jitter due to the disadvantage of software PWM

    def move2start(self):
        self.expected_joints_angles = self.init_joints_angles[:]  # self.expected_joints_angles = self.init_joints_angles
        self.expected_coord = self.fk_calculate(self.init_joints_angles)  # self.expected_coord = self.init_coord
        self.move_leg(None)

    def process_trans(self, axis, offset, elapsed=50, duration=1000):
        fr_coord, br_coord, fl_coord, bl_coord = translate_coord(axis, offset, self.expected_coord)

        inter_fr_coord, inter_br_coord, inter_bl_coord, inter_fl_coord = self.expected_coord
        remain_time = duration
        while elapsed <= remain_time:
            ratio = elapsed / remain_time
            for i in range(2):
                inter_fr_coord[i] += (fr_coord[i] - inter_fr_coord[i]) * ratio
                inter_br_coord[i] += (br_coord[i] - inter_br_coord[i]) * ratio
                inter_bl_coord[i] += (bl_coord[i] - inter_bl_coord[i]) * ratio
                inter_fl_coord[i] += (fl_coord[i] - inter_fl_coord[i]) * ratio

            self.ik_calculate(inter_fr_coord, inter_br_coord, inter_bl_coord, inter_fl_coord)
            self.move_leg(None)

            remain_time -= elapsed

    def process_rotate(self, height, roll_pitch_angles, elapsed=50, duration=1000):
        half_bl = self.body_l/2  # Body Length
        half_bw = self.body_w/2  # Body Width
        roll, pitch = [angle * pi / 180 for angle in roll_pitch_angles]  # 8DOF dog has yaw angle equal 0.
        fr_coord, br_coord, fl_coord, bl_coord = rotate_coord(height, half_bl, half_bw, roll, pitch)

        inter_fr_coord, inter_br_coord, inter_bl_coord, inter_fl_coord = self.expected_coord
        remain_time = duration
        while elapsed <= remain_time:
            ratio = elapsed / remain_time
            for i in range(2):
                inter_fr_coord[i] += (fr_coord[i] - inter_fr_coord[i]) * ratio
                inter_br_coord[i] += (br_coord[i] - inter_br_coord[i]) * ratio
                inter_bl_coord[i] += (bl_coord[i] - inter_bl_coord[i]) * ratio
                inter_fl_coord[i] += (fl_coord[i] - inter_fl_coord[i]) * ratio
            self.ik_calculate(inter_fr_coord, inter_br_coord, inter_bl_coord, inter_fl_coord)
            self.move_leg(None)

            remain_time -= elapsed

    def process_cone_rotate(self, height, h=5, r=1, step=0.12, home=True):
        theta = 0
        while theta <= 2*pi:
            theta += step
            roll_angle = atan(r * cos(theta) / h) * 180 / pi
            pitch_angle = atan(r * sin(theta) / h) * 180 / pi
            self.process_rotate(height, [roll_angle, pitch_angle], elapsed=12, duration=12)
        if home:
            self.process_rotate(height, [0, 0])

    def process_demo_gait(self,  height, trot_cycle_length, trot_duty_ratio, path_increment, x_front, x_back, lift_height, reverse, elapsed=10, duration=20):
        mode_gait_dict = {'trot': trot_gait, 'turn': turn_gait, 'gallup': gallup_gait, 'jump': jump_gait}
        gait_func = 'turn'  # If mode != 1 (please call process_demo_gait only when mode==2 which represent 'turn' mode)
        if self.mode == 1:
            if self.gait == 0:
                gait_func = 'trot'
            elif self.gait == 1:
                gait_func = 'gallup'

        try:
            fr_xz_path, br_xz_path, bl_xz_path, fl_xz_path = mode_gait_dict[gait_func](height, trot_cycle_length, trot_duty_ratio, path_increment, x_front, x_back, lift_height, reverse)
            cnt = 0
            while cnt < self.walking_times:
                for tmp_id, fr_coord in enumerate(fr_xz_path):
                    br_coord = br_xz_path[tmp_id]
                    bl_coord = bl_xz_path[tmp_id]
                    fl_coord = fl_xz_path[tmp_id]

                    remain_time = duration
                    inter_fr_coord, inter_br_coord, inter_bl_coord, inter_fl_coord = self.expected_coord
                    while elapsed <= remain_time:
                        ratio = elapsed / remain_time
                        for i in range(2):
                            inter_fr_coord[i] += (fr_coord[i] - inter_fr_coord[i]) * ratio
                            inter_br_coord[i] += (br_coord[i] - inter_br_coord[i]) * ratio
                            inter_bl_coord[i] += (bl_coord[i] - inter_bl_coord[i]) * ratio
                            inter_fl_coord[i] += (fl_coord[i] - inter_fl_coord[i]) * ratio
                        self.ik_calculate(inter_fr_coord, inter_br_coord, inter_bl_coord, inter_fl_coord)
                        self.move_leg(None)

                        remain_time -= elapsed
                cnt += 1

        except Exception as e:
            print(repr(e))
            raise

    def process_panel(self, panel_req):
        changing_tag = False
        if 'GET /?Gait=trot' in panel_req:
            self.mode = 0
            self.gait = 0
            self.move2start()
        elif 'GET /?Gait=gallup' in panel_req:
            self.mode = 0
            self.gait = 1
            self.move2start()
        elif 'GET /?MODE=stand' in panel_req:
            self.mode = 0
            self.move2start()
        elif 'GET /?MODE=move' in panel_req:
            self.mode = 1
            self.move2start()
        elif 'GET /?MODE=turn' in panel_req:
            self.mode = 2
            self.move2start()
        elif 'GET /?MODE=translate' in panel_req:
            self.mode = 3
            self.move2start()
        elif 'GET /?MODE=rotate' in panel_req:
            self.mode = 4
            self.move2start()
        elif 'GET /?Key=forward' in panel_req:
            self.up_down_tag = True
            self.forward_reverse = False
            self.step_reset = False
            changing_tag = True
        elif 'GET /?Key=backward' in panel_req:
            self.up_down_tag = True
            self.forward_reverse = True
            self.step_reset = False
            changing_tag = True
        elif 'GET /?Key=left' in panel_req:
            self.left_right_tag = True
            self.left_reverse = False
            self.step_reset = False
            changing_tag = True
        elif 'GET /?Key=right' in panel_req:
            self.left_right_tag = True
            self.left_reverse = True
            self.step_reset = False
            changing_tag = True
        elif 'GET /?Key=step_reset' in panel_req:
            self.step_reset = True
            changing_tag = True
        elif 'GET /height=' in panel_req:
            index_r = panel_req.find('height=')
            set_height = int(panel_req[index_r + 7:index_r + 10].lstrip().rstrip())
            current_height = sum(self.expected_coord[i][1] for i in range(4)) / 4
            set_height = max(80, min(110, set_height))
            self.set_height = set_height
            height_offset = set_height - current_height
            self.process_trans(axis=1, offset=height_offset, elapsed=50, duration=1000)
            changing_tag = False
        elif 'GET /times=' in panel_req:
            index_r = panel_req.find('times=')
            self.walking_times = int(panel_req[index_r + 6:index_r + 7].lstrip().rstrip())
        return changing_tag

    def remote_control(self):
        try:
            # Process Mode (mode 0 is default 'home idle' do-nothing mode)
            height = self.set_height  # 4 represent 4 legs
            if self.mode in [1, 2]:  # Gait (include stepping)
                reverse_tag = self.forward_reverse if self.mode == 1 else self.left_reverse
                if (self.mode == 1 and self.up_down_tag) or (self.mode == 2 and self.left_right_tag):
                    self.process_demo_gait(height=height, trot_cycle_length=1.0, trot_duty_ratio=0.5, path_increment=0.05,
                                          x_front=15, x_back=-15, lift_height=20, reverse=reverse_tag, elapsed=1,
                                          duration=1)
            elif self.mode == 3:  # Translate
                if self.left_right_tag:  # Forward - Back
                    self.process_trans(axis=0, offset=(-1)**int(self.left_reverse)*30, elapsed=50, duration=1000)
                    sleep_ms(200)
                    self.process_trans(axis=0, offset=(-1)**int(not self.left_reverse)*30, elapsed=50, duration=1000)
                    self.left_right_tag = False
                elif self.up_down_tag:  # Up - Down
                    self.process_trans(axis=1, offset=(-1) ** int(self.forward_reverse) * 30, elapsed=50, duration=1000)
                    sleep_ms(200)
                    self.process_trans(axis=1, offset=(-1) ** int(not self.forward_reverse) * 30, elapsed=50, duration=1000)
                    self.up_down_tag = False
            elif self.mode == 4:  # Rotates
                if self.left_right_tag:  # Roll
                    self.process_rotate(height=height, roll_pitch_angles=[(-1)**int(not self.left_reverse)*30, 0], elapsed=50, duration=800)
                    sleep_ms(200)
                    self.process_rotate(height=height, roll_pitch_angles=[0, 0], elapsed=50, duration=800)
                    self.left_right_tag = False
                elif self.up_down_tag:  # Pitch
                    self.process_rotate(height=height, roll_pitch_angles=[0, (-1) ** int(not self.forward_reverse)*20], elapsed=50, duration=800)
                    sleep_ms(200)
                    self.process_rotate(height=height, roll_pitch_angles=[0, 0], elapsed=50, duration=800)
                    self.up_down_tag = False
            elif self.mode == 5:  # Cone Rotate
                self.process_cone_rotate(height, home=True)
        except ValueError:
            print('Invalide Operation, Please Try Again!')
