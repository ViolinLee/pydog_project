# -*- coding: utf-8 -*-

# -*- coding: utf-8 -*-

from pydog.pydog import PyDog
from pydog.config import *
from time import sleep

dog = PyDog(body_l, body_w, l1, l2, joints_ids, init_joints_angles, init_servos_pos, calibration_txt, pca_i2c_scl,
            pca_i2c_sda, pca_i2c_freq, pca_i2c_adr)
sleep(3)

for i in range(6):
    dog.process_trans(axis=0, offset=(-1) ** i * 50, duration=800)
sleep(0.5)

dog.move2start()
for i in range(6):
    dog.process_trans(axis=1, offset=(-1) ** i * 60, duration=800)
sleep(0.5)

dog.move2start()
dog.process_trans(axis=1, offset=20, duration=400)
sleep(0.2)

height = dog.expected_coord[0][1]
roll_pitch_sequences = [[30, 0], [0, 0],
                        [-30, 0], [0, 0],
                        [30, 0], [0, 0],
                        [-30, 0], [0, 0],
                        [0, 15], [0, 0],
                        [0, -15], [0, 0],
                        [0, 15], [0, 0],
                        [0, -15], [0, 0]]
for roll_pitch in roll_pitch_sequences:
    dog.process_rotate(height, roll_pitch, elapsed=50, duration=800)
sleep(0.5)

for i in range(4):
    dog.process_cone_rotate(height, home=False)
dog.process_cone_rotate(height, home=True)
