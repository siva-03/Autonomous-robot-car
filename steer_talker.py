#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

from car_control.utils import user_input_talker

if __name__ == '__main__':
    rospy.init_node('steer_talker', anonymous=True)
    try:
        user_input_talker('manual_steer')
    except rospy.ROSInterruptException:
        pass
