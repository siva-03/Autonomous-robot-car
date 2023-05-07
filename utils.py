#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import numpy as np
import os


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
    if 1000 <= target <= 2000 and 1 <= channel <= 2:
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
        # print("trying to turn with bytes: ")
        # print(echo_string)
        os.system(echo_string)
    else:
        print("target was not between 1000 and 2000, or channel not 1 or 2")