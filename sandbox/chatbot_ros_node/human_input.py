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
import time



# The following from http://blog.mathieu-leplatre.info/colored-output-in-console-with-python.html
#===================================================================
import sys
from subprocess import call

BLACK, RED, GREEN, YELLOW, BLUE, MAGENTA, CYAN, WHITE = range(8)

#following from Python cookbook, #475186
def has_colours(stream):
    if not hasattr(stream, "isatty"):
        return False
    if not stream.isatty():
        return False # auto color only on TTYs
    try:
        import curses
        curses.setupterm()
        return curses.tigetnum("colors") > 2
    except:
        # guess false in case of error
        return False
has_colours = has_colours(sys.stdout)


def printout(text, colour=WHITE):
    if has_colours:
	seq = "\x1b[1;%dm" % (30+colour) + text + "\x1b[0m"
	sys.stdout.write(seq)
    else:
	sys.stdout.write(text)
#===================================================================


LINE_LEN = 80
wait_for_response = False


def on_say(msg):
    global wait_for_response
    text = msg.data
    if len(text) > LINE_LEN:
	printout(text + '\n', GREEN)
    else:
    	printout(' ' * (LINE_LEN - len(text)) + text + ' <\n', GREEN)
    print
    wait_for_response = False


def talker():
    global wait_for_response
    pub = rospy.Publisher('recognized_speech', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    sub = rospy.Subscriber('/say', String, on_say)
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        str = raw_input("> ")
        #rospy.loginfo(str)
	if str.startswith('/stop'):
		call(["killall", "espeak"])
	else:
		wait_for_response = True
        	pub.publish(str)
	rate.sleep()
	time.sleep(1.0)


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
	pass


