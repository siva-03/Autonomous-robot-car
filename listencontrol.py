#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
import os


class StopStartData:
    def __init__(self):
        self.is_started = False


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


def stop_start_callback(data, stop_start_data):
    stop_start_data.is_started = not stop_start_data.is_started


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


# Takes a value, assuming it came from a domain with min_value and max_value,
# and proportionally shifts it to the new_min_value and new_max_value range.
# In this case, we use -10000 and 10000 as the domain because we are thresholding
# at 10 meters, which is 10000 in the depth camera, and we assume one side having full
# depth (all 10000's) and other side having no depth (all 0's, i.e. wall there) is min/max.
# Then the range of 1000 to 2000 is the Pololu Maestro control range. So say
# diff=10000, meaning all wall on left and no wall right (10000-0), that will map
# to the new max of 2000, which on the Pololu will map to steering wheel fully right
# But actually probably better to do from 0 to 4000 since -10000 and 10000 neven happen, but we
# still sometimes want the full turn, and we will clip between 1000 and 2000
def min_max_scale(value, min_value, max_value, new_min_value, new_max_value):
    return (value - min_value) / (max_value - min_value) * (new_max_value - new_min_value) + new_min_value


# takes an array, and replaces all the values greater than the threshold with
# the threshold itself.
# Returns the difference between the mean of the left side
# and the right side (columns, second dim)
def get_difference_with_threshold(orig_arr, thresh):
    depth_thresh = np.where(orig_arr > thresh, thresh, orig_arr)
    left_side = depth_thresh[:, :320]
    right_side = depth_thresh[:, 320:]
    return np.mean(left_side) - np.mean(right_side)


def write_serial_byte_string(channel=1, target=1500):
    print("received cmd to write to channel 1 target: ", String(target))
    # create serial bytes array
    # initialize with command byte, maestro, always this
    serial_bytes = ["x84"]

    # channel bytes
    channel = int(channel)
    channel_hex = hex(channel)[1:]
    serial_bytes.append(channel_hex)

    # target bytes
    target = int(target) * 4
    # second and third bytes need to be defined specifically like this:
    # https://www.pololu.com/docs/0J40/5.e
    second_byte = target & 0x7F
    third_byte = (target >> 7) & 0x7F
    second_hex = hex(second_byte)[1:]
    third_hex = hex(third_byte)[1:]
    serial_bytes.append(second_hex)
    serial_bytes.append(third_hex)

    echo_string = r'sudo echo -n -e "\\' + serial_bytes[0] + r'\\' + serial_bytes[1] + r'\\' + serial_bytes[2] + r'\\' + serial_bytes[3] + r'" > /dev/ttyACM0'
    # \x84\x01\x70\x2e" > /dev/ttyACM0'
    print("trying to turn with bytes: ")
    # print(echo_string)
    os.system(echo_string)


# Main control loop for the listener script
def control_loop():
    try:
        rospy.init_node('robotcontrol', anonymous=True)
        sensor_data = SensorData()
        start_stop_data = StopStartData()
        camera_data = ImageData()
        depth_data = ImageData()

        bridge = CvBridge()

        rospy.Subscriber("chatter", String, imu_callback, callback_args=sensor_data)
        # rospy.Subscriber("stopstart", String, stop_start_callback, callback_args=start_stop_data)
        rospy.Subscriber("camera/color/image_raw", Image, camera_callback, callback_args=(camera_data, bridge))
        rospy.Subscriber("camera/depth/image_rect_raw", Image, depth_callback, callback_args=(depth_data, bridge))

        # pid initialization
        kp = 1.0
        ki = 0
        kd = 0.01
        set_point = 0.0  # balanced depth on both sides (perhaps we could also weight depth toward center more)
        threshold = 10000  # for depth
        error = 0.0
        last_error = 0.0
        integral = 0.0
        discretization_amount = 10
        max_output = threshold / discretization_amount  # how much pi / x radians to move at once
        start_time = time.time()
        last_time = start_time

        #  Initialize Car

        car_current_wheel = 1500
        write_serial_byte_string(channel=2, target=1500)
        rospy.sleep(1)

        while not rospy.is_shutdown():
            if depth_data.image_data is not None:
                position = get_difference_with_threshold(depth_data.image_data, threshold)
                print("im currently at camera diff position: ", position)
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
                print("output before min/max: ", output)

                # Limit output to maximum value
                output = min(max_output, max(-max_output, output))
                # get_desired_angle(output) # if output = -0.1, returns -10degree, if output = 0.05, return 6degree
                # maybe wrong? maybe right? maybe need to keep track of maestro current position
                # and use the range to be -50 to 50, if x = 20 above, and add or subtract output
                # from current steering pos <- looks good!
                # output_to_pololu_value = min_max_scale(output)
                # convert_output_to_maestro_int # take output, get 1000-2000 value for our steering angle
                # call_servo_with_int(output) # get byte seq, and write to file
                print("output after min/max: ", output)
                maestro_output = min_max_scale(output,
                                               -max_output,
                                               max_output,
                                               (-1000/discretization_amount),
                                               (1000/discretization_amount))
                print("maestro output: ", maestro_output)
                car_current_wheel += maestro_output
                print("car current wheel after output: ", car_current_wheel)
                car_current_wheel = min(2000, max(car_current_wheel, 1000))
                print("car current wheel after CHOPPING: ", car_current_wheel)
                write_serial_byte_string(channel=2, target=car_current_wheel)

            rospy.sleep(0.05)
    except KeyboardInterrupt:
        print("INTERRUPTING!!!!")
        write_serial_byte_string(channel=2, target=1500)


if __name__ == '__main__':
    control_loop()
