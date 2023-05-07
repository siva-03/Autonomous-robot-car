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

    while not rospy.is_shutdown():
        user_input = input()
        user_input = str(user_input)
        rospy.loginfo(user_input)
        pub.publish(user_input)
        rospy.spin()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass