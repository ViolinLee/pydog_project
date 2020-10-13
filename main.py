# -*- coding: utf-8 -*-

# -*- coding: utf-8 -*-

from pydog.pydog import PyDog
from pydog.config import *
from time import sleep

dog = PyDog(body_l, body_w, l1, l2, joints_ids, init_joints_angles, init_servos_pos, calibration_txt, pca_i2c_scl, pca_i2c_sda, pca_i2c_freq, pca_i2c_adr)
sleep(3)


dog.process_trans(axis=1, offset=-60, duration=1200)
dog.process_trans(axis=1, offset=80, duration=1600)
dog.process_trans(axis=1, offset=-20, duration=400)

dog.process_trans(axis=0, offset=50, duration=1000)
dog.process_trans(axis=0, offset=-60, duration=1200)
dog.process_trans(axis=0, offset=10, duration=200)