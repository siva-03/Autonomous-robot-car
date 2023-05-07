#!/usr/bin/env python
# Robotics script

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import time
import os
# import cv2
# import matplotlib.pyplot as plt

from car_control.control_classes import PhysicalCar
from car_control.control_classes import PIDController
from car_control.control_classes import SensorData
from car_control.control_classes import ImageData

from car_control.control_callbacks import imu_callback
from car_control.control_callbacks import manual_speed_callback
from car_control.control_callbacks import pid_param_callback
from car_control.control_callbacks import camera_callback
from car_control.control_callbacks import depth_callback

from car_control.utils import min_max_scale
from car_control.utils import get_difference_with_threshold
from car_control.utils import write_serial_byte_string


# Main control loop for the listener script
def control_loop():
    # Initialization:
    print("Entering control loop. Car immediately initializing and centering")
    # Initialize Car Physically (this will set the wheels and motor to 1500...!)
    car = PhysicalCar(steering_channel=2, motor_channel=1)
    # so we might want to sleep and chill out for a second
    rospy.sleep(1)
    print("car successfully initialized, continuing now...")

    # initialize data objects
    sensor_data = SensorData()
    camera_data = ImageData()
    depth_data = ImageData()

    # PID initialization
    pid_controller = PIDController(1.0, 0.0, 0.01)
    # could use to say "go left until: a lot of depth in front of us, or for x seconds, then back up?"
    # if we have a lot of shallow, call that ^
    set_point = 0.0  # We could change this to -10000 if we wanted it to go left, or 10000 to go right!
    threshold = 10000  # for depth
    discretization_amount = 10
    max_output = threshold / discretization_amount  # how much pi / x radians to move at once

    # Initialize subscribers with callbacks, and the CvBridge for handling images in callback
    bridge = CvBridge()
    rospy.Subscriber("chatter", String, imu_callback, callback_args=sensor_data)
    rospy.Subscriber("manual_speed", String, manual_speed_callback, callback_args=car)
    rospy.Subscriber("manual_pid", String, pid_param_callback, callback_args=pid_controller)
    rospy.Subscriber("camera/color/image_raw", Image, camera_callback, callback_args=(camera_data, bridge))
    rospy.Subscriber("camera/depth/image_rect_raw", Image, depth_callback, callback_args=(depth_data, bridge))

    # just set for testing, will delete. This is our first test, to make sure ctrl+c works
    # hash out things in control loop that do anything, so it just spins and prints or something
    # then we test ctrl+c
    # Do not forget to add all these files to CMake :)
    car.steering = 1800
    rospy.sleep(1)

    while not rospy.is_shutdown():
        if depth_data.image_data is not None:
            position = get_difference_with_threshold(depth_data.image_data, threshold)
            print("im currently at camera diff position: ", position)

            # Instruct PID Controller to take step, based on position and desired set point
            # and get back the controller's output
            output = pid_controller.step(position, set_point)
            print("output before min/max: ", output)

            # Limit output to maximum value, because we don't want to over-react
            output = min(max_output, max(-max_output, output))
            print("output after min/max: ", output)

            # Then scale it for our Maestro controller, which has a range of 1000
            maestro_output = min_max_scale(output,
                                           -max_output,
                                           max_output,
                                           (-1000/discretization_amount),
                                           (1000/discretization_amount))
            print("maestro output: ", maestro_output)

            # Update current wheel position based on Maestro output, clipped between 1000 and 2000
            print("trying to set steering: ", str(min(2000, max((car.steering + maestro_output), 1000))))
            # car.steering = min(2000, max((car.steering + maestro_output), 1000))

        rospy.sleep(0.05)


if __name__ == '__main__':
    # https://stackoverflow.com/questions/60035686/how-multiple-run-ros-init-node-in-one-python-script
    # "You should centralize the initialization of the node at the beginning of the main script"
    rospy.init_node('listen_control', anonymous=True)
    try:
        control_loop()
    except rospy.ROSInterruptException:
        # Ctrl+C or rosnode kill ... was detected
        print("terminating...")
        write_serial_byte_string(channel=1, target=1500)
        write_serial_byte_string(channel=2, target=1500)
