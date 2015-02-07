#!/usr/bin/env python
#===================================================================
# Run this program to send typed messages to the recognized_speech
# topic, to which the robot A.I. will be listening. Later this
# program can be replaced by the hearing node from the Speech and
# Hearing team.
#
# Copyright 2014, IEEE ENCS Humanoid Robot Project
#===================================================================

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('recognized_speech', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    while not rospy.is_shutdown():
        str = raw_input("> ")
        rospy.loginfo(str)
        pub.publish(str)

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException: pass
