#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import time
import os

from control_classes import PhysicalCar
from control_classes import PIDController
from control_classes import SensorData
from control_classes import ImageData

from utils import write_serial_byte_string


def imu_callback(data, sensor_data):
    sensor_data.imu_data = data


# Takes the callback data and PhysicalCar,
# and makes a call to the PhysicalCar motor if is int-like string in range
# otherwise, if it hears something other than a valid int, just writes 1500
def manual_speed_callback(data, param_car):
    try:
        sanitized_data = int(data)
        if 1000 <= sanitized_data <= 2000:
            param_car.motor = sanitized_data
        else:
            print("error: manual speed was int, but not in range of 1000-2000: ", sanitized_data)
            param_car.motor = 1500
    except ValueError:
        print("error: manual speed not an int")


# Takes callback data and PIDController, and sets the PID properties for p, i, d values
# then also resets the integral and last error and time to 0, so reinitializes
# Assumes the user sends PID as "1.0,0.01,0.001" for P,I,D vals
def pid_param_callback(data, param_controller):
    kpid_arr = data.split(',')
    try:
        param_controller.kp = float(kpid_arr[0])
        param_controller.ki = float(kpid_arr[1])
        param_controller.kd = float(kpid_arr[2])
        param_controller.reset()
    except ValueError:
        print("error: one or more of the floats passed to PID were invalid")


def camera_callback(data, param_data):
    camera_data = param_data[0]
    bridge = param_data[1]
    real_img = None
    try:
        real_img = bridge.imgmsg_to_cv2(data, "bgr8")
        real_img = np.asarray(real_img)[:, :, ::-1]
    except CvBridgeError as e:
        print(e)

    camera_data.image_data = real_img
    camera_data.width = data.width
    camera_data.height = data.height


def depth_callback(data, param_data):
    depth_data = param_data[0]
    bridge = param_data[1]
    real_img = None
    try:
        real_img = bridge.imgmsg_to_cv2(data)
        real_img = np.asarray(real_img)
    except CvBridgeError as e:
        print(e)

    depth_data.image_data = real_img
    depth_data.width = data.width
    depth_data.height = data.height
