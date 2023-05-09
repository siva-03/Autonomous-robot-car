#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String
import numpy as np
import os
import matplotlib.pyplot as plt
import cv2
# from scipy.ndimage import gaussian_filter
# import torch


# Talker function for publishers that just take user input and publish to channel
def user_input_talker(pub_channel):
    pub = rospy.Publisher(str(pub_channel), String, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        user_input = input()
        user_input = str(user_input)
        print('user input coming in: ', user_input)
        rospy.loginfo(user_input)
        pub.publish(user_input)
        rate.sleep()


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
    if 1000 <= target <= 2000 and 1 <= channel <= 4:
        print("received cmd to write to channel ?, target: ", str(target))
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
        print(echo_string)
        os.system(echo_string)
    else:
        print("target was not between 1000 and 2000, or channel not 1 or 2")


# def stop_sign_detector(rgb_image_np, model):
#     # Run the YOLOv5s model on the image
#     results = model(rgb_image_np, size=640)
#     # Get the results for the stop sign class (class 11)
#     stop_sign_results = results.xyxy[0][results.xyxy[0][:, 5] == 11]
#     # Print the bounding box coordinates for each detected stop sign
#     print("results: ", results)
#     is_stop_sign = False
#     for result in stop_sign_results:
#         is_stop_sign = True
#         print('Stop sign detected at:', result[0], result[1], result[2], result[3])
#     return is_stop_sign


# def calculate_speed_based_on_depth(depth_img_np):
#     # take 480x480 center
#     depth_img_np = depth_img_np[:, 80:560]
#
#     # first apply element-wise inversion, so close depths are high value and far depths are low
#     depth_img_np = 1 / (depth_img_np + 0.000001)
#
#     # apply a Gaussian filter with sigma = 5
#     depth_img_np_gauss = gaussian_filter(depth_img_np, sigma=5)


def stop_sign_detector(bgr_image_cv):
    print("entering stop sign detector")
    file_path = os.path.join(os.path.dirname(__file__), 'stop_sign.xml')

    # Load the stop sign Haar cascade classifier
    stop_cascade = cv2.CascadeClassifier(file_path)
    print("stop cascade created")

    # Detect stop signs in the image
    stop_signs = stop_cascade.detectMultiScale(bgr_image_cv, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30), flags=cv2.CASCADE_SCALE_IMAGE)
    print("stop signs: ", stop_signs)

    # Draw a rectangle around each detected stop sign
    found_stop = False
    for (x, y, w, h) in stop_signs:
        if w > 50 and h > 50:
            found_stop = True
            print("found")
            print(f"X: {x}, Y: {y}, W: {w}, hey {h}")
        else:
            print(f"TOO FAR BACK! X: {x}, Y: {y}, W: {w}, hey {h}")

    return found_stop


def check_obstacle_in_front(depth_img_np):
    # if n number of pixels less than x distance in middle third? of img, return true

    return False


def check_wall_in_prox(depth_img_np):

    return True
