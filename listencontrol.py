#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

class SensorData:
    def __init__(self):
        self.imu_data = None

class ImageData:
    def __init__(self):
        self.image_data = None

def imu_callback(data, sensor_data):
    sensor_data.imu_data = data

def camera_callback(data, camera_data):
    camera_data.image_data = data

def control_loop():
    rospy.init_node('mylisten', anonymous=True)
    sensor_data = SensorData()
    camera_data = ImageData()

    rospy.Subscriber("chatter", String, imu_callback, callback_args=sensor_data)
    rospy.Subscriber("camera/color/image_raw", Image, camera_callback, callback_args=camera_data)

    while not rospy.is_shutdown():
        if sensor_data.imu_data is not None:
            # Do something with the imu_data
            print("Received imu data: ", sensor_data.imu_data)

        if camera_data.image_data is not None:
            print("in loop cam: ", camera_data.image_data)

        # Do other stuff in the main loop
        print("main loop imu: ", sensor_data.imu_data)
        print("main loop cam: ", camera_data.image_data)

        rospy.sleep(0.5)

if __name__ == '__main__':
    control_loop()
