#!/usr/bin/env python3

import math

wheel_radius = 0.04 # 80 mm wheel diameter
motor_rpm_max = 200.0 # 200 rpm max for jgb37-520 motor 12V 200 rpm

max_linear_velocity = (motor_rpm_max / 60) * 2 * math.pi * wheel_radius # in m/s
max_angular_velocity = (motor_rpm_max / 60) * 2 * math.pi # in rad/s

tof_frame_id = "range_finder"