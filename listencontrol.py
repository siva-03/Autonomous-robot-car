#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time


class SensorData:
    def __init__(self):
        self.imu_data = None


class ImageData:
    def __init__(self):
        self.image_data = None
        self.width = None
        self.height = None


def imu_callback(data, sensor_data):
    sensor_data.imu_data = data


def camera_callback(data, param_data):
    camera_data = param_data[0]
    bridge = param_data[1]
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
    try:
        real_img = bridge.imgmsg_to_cv2(data)
        real_img = np.asarray(real_img)
    except CvBridgeError as e:
        print(e)

    depth_data.image_data = real_img
    depth_data.width = data.width
    depth_data.height = data.height


def min_max_scale(value):
    min_value = -10000
    max_value = 10000
    new_min_value = -1
    new_max_value = 1
    return (value - min_value) / (max_value - min_value) * (new_max_value - new_min_value) + new_min_value


def get_difference_with_threshold(orig_arr, thresh):
    depth_thresh = np.where(orig_arr > thresh, thresh, orig_arr)
    left_side = depth_thresh[:, :320]
    right_side = depth_thresh[:, 320:]
    return min_max_scale(np.mean(left_side) - np.mean(right_side))


def control_loop():
    rospy.init_node('mylisten', anonymous=True)
    sensor_data = SensorData()
    camera_data = ImageData()
    depth_data = ImageData()
    
    bridge = CvBridge()

    rospy.Subscriber("chatter", String, imu_callback, callback_args=sensor_data)
    # rospy.Subscriber("camera/color/image_raw", Image, camera_callback, callback_args=(camera_data, bridge))
    rospy.Subscriber("camera/depth/image_rect_raw", Image, depth_callback, callback_args=(depth_data, bridge))
    flag = False

    # pid initialization
    kp = 1.0
    ki = 0.001
    kd = 0.01
    set_point = 0.0
    error = 0.0
    last_error = 0.0
    integral = 0.0
    max_output = 0.1
    start_time = time.time()
    last_time = start_time

    while not rospy.is_shutdown():
        # Main Loop
        print("main loop imu: ", sensor_data.imu_data)

        if depth_data.image_data is not None and not flag:

            position = get_difference_with_threshold(depth_data.image_data)
            # Get current time
            current_time = time.time()

            # Calculate time elapsed since last iteration
            delta_time = current_time - last_time

            # Calculate error
            error = set_point - position

            # Calculate derivative of error
            if delta_time > 0:
                error_derivative = (error - last_error) / delta_time
            else:
                error_derivative = 0

            # Calculate integral term
            integral += error * delta_time

            # Calculate output value
            output = kp * error + ki * integral + kd * error_derivative

            # Limit output to maximum value
            output = min(max_output, max(-max_output, output))
            print(output)

        rospy.sleep(0.5)


if __name__ == '__main__':
    control_loop()
