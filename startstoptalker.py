#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String


class StopStartData:
    def __init__(self):
        self.is_started = False


def talker():
    pub = rospy.Publisher('startstop', String, queue_size=10)
    rospy.init_node('mystartstoptalker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        user_input = input()
        user_input = str(user_input)
        print('user input coming in: ', user_input)
        rospy.loginfo(user_input)
        pub.publish(user_input)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass