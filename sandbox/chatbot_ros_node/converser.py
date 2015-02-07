#!/usr/bin/env python
#===================================================================
# This program is a stand-in for a more complete A.I. ROS node.
# It will subscribe to the heard_text topic, receive strings,
# construct response text, and publish to the speech_text topic.
# In a working robot, a hearing node will supply text heard from a
# human, while a speech node will do text-to-speech conversion of
# the speech_text strings.
#
# This version experiments with AIML instead of NLTK.
#
# Copyright 2014, IEEE ENCS Humanoid Robot Project
#===================================================================

from __future__ import print_function
import rospy
from std_msgs.msg import String
import aiml

botAttributes = {
    "name":           "Ken",
    "master":         "the IEEE ENCS Humanoid Robot Team",
    "gender":         "male",
    "location":       "Daniel's place", # change me often
    "birthplace":     "Raleigh, North Carolina",
}

class Converser(object):
    def __init__(self):
        self.pub = rospy.Publisher('say', String)
        rospy.Subscriber('recognized_speech', String, self.callback)
        rospy.init_node('converser')
        self.bot = aiml.Kernel()
        self.bot.learn("aiml-startup.xml")
        self.bot.respond("load aiml b")
        for key, value in botAttributes.iteritems():
            self.bot.setBotPredicate(key, value)

    def callback(self,msg):
        rospy.loginfo(rospy.get_caller_id() + ": I heard: %s", msg.data)
        utterance = self.bot.respond(msg.data)
        print(utterance)
        self.pub.publish(utterance)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        c = Converser()
        c.run()
    except rospy.ROSInterruptException: pass
