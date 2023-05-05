#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
import cv2

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

    while not rospy.is_shutdown():
        # if sensor_data.imu_data is not None:
            # Do something with the imu_data
            # print("Received imu data: ", sensor_data.imu_data)

        # if camera_data.image_data is not None:
            # print("in loop cam: ", camera_data.image_data)

        # Do other stuff in the main loop
        print("main loop imu: ", sensor_data.imu_data)
        # print("main loop cam: ", camera_data.image_data)

        # print("width: ", camera_data.width)
        # print("ht: ", camera_data.height)
        if depth_data.image_data is not None and not flag:
            #flag = True
            # plt.imshow(camera_data.image_data)
            # plt.show()
            # (rows,cols,channels) = camera_data.image_data.shape
            # cv2.imshow("Image window", camera_data.image_data)
            # cv2.imwrite("img.jpg", camera_data.image_data)
            # cv2.waitKey(3)
            #plt.imshow(depth_data.image_data)
            #plt.show()
            #print(depth_data.image_data)
            #print("shape ", depth_data.image_data.shape)
            #print("min: ", np.min(depth_data.image_data))
            #print("max: ", np.max(depth_data.image_data))

            # print(camera_data.image_data)
            #print("shape ", camera_data.image_data.shape)
            #subtract both sides of the image
            max_val = 10000
            depth_thresh = np.where(depth_data.image_data > max_val, max_val, depth_data.image_data)
            left_side = depth_thresh[:, :320]
            right_side = depth_thresh[:, 320:]
            diff = np.mean(left_side) - np.mean(right_side)
            print(diff)

        rospy.sleep(0.5)

if __name__ == '__main__':
    control_loop()
