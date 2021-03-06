# -*- coding: utf-8 -*-

# Physical Attributes
l1 = 80
l2 = 62.5
body_l = 202
body_w = 105

init_servos_pos = (154, 110, 149, 104, 23, 70, 18, 73)  # Assembly Features
init_joints_angles = (52, 88, 52, 88, 52, 88, 52, 88)  # Kinematics Model
calibration_txt = "pydog/calibration.txt"

# IOs Mappings
joints_ids = (4, 5, 6, 7, 8, 9, 10, 11)

# Gait Setting
trot_cycle_length = 1.0
trot_duty_ratio = 0.5
time_increment = 0.01
elapsed = 0.01
x_front = 20
x_back = -30
lift_height = 50
kp_height = 0.03
kp_weight = 0.04

# PCA9685 Setting
pca_i2c_adr = 0x40
pca_i2c_scl = "Y9"
pca_i2c_sda = "Y10"
pca_i2c_freq = 100000