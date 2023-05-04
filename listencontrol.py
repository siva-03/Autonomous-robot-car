#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class SensorData:
    def __init__(self):
        self.imu_data = None

def imu_callback(data, sensor_data):
    sensor_data.imu_data = data

def control_loop():
    rospy.init_node('mylisten', anonymous=True)
    sensor_data = SensorData()

    rospy.Subscriber("chatter", String, imu_callback, callback_args=sensor_data)

    while not rospy.is_shutdown():
        if sensor_data.imu_data is not None:
            # Do something with the imu_data
            print("Received imu data: ", sensor_data.imu_data)

        # Do other stuff in the main loop
        print("main loop imu: ", sensor_data.imu_data)

        rospy.sleep(0.1)

if __name__ == '__main__':
    control_loop()
