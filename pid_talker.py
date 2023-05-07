#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

from utils import user_input_talker

if __name__ == '__main__':
    rospy.init_node('pid_talker', anonymous=True)
    try:
        user_input_talker('manual_pid')
    except rospy.ROSInterruptException:
        pass
