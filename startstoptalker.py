#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import keyboard


class StopStartData:
    def __init__(self):
        self.is_started = False


def on_key_press(event, obj):
    print('Key {} was pressed.'.format(event.name))
    obj.is_started = not obj.is_started


def talker():
    pub = rospy.Publisher('startstop', String, queue_size=10)
    rospy.init_node('mystartstoptalker', anonymous=True)
    start_stop_data = StopStartData()

    # initialize to false
    start_stop_data.is_started = False

    while not rospy.is_shutdown():
        keyboard.on_press(on_key_press)
        keyboard.wait()
        rospy.loginfo(str(start_stop_data.is_started))
        pub.publish(str(start_stop_data.is_started))
        rospy.spin()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass