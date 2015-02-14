#!/usr/bin/env python
#===================================================================
# This is the AI Respond Node.
#
# Subscribes To:
# - recognized_speech
# - recognized_face
#
# Publishes To:
# - say
#
# Responses:
# 1) The node will generate text responses to recognized_speech,
#    publishing them to the say topic.
# 2) The node will attempt to match names and faces, saving the
#    pairings to an in-memory knowledge base.
# 3) If a new face is recognized, the node will generate
#    a greeting and attempt to get the person's name.
# 4) On receiving a kill signal, the node will save
#    the currently loaded knowledge base back to a file
#    for later reuse.
#
# AIML is used for the basic chat functionality.
#
# Copyright 2015, IEEE ENCS Humanoid Robot Project
#===================================================================

from __future__ import print_function
import sys
import os.path
import pickle
import rospy
from std_msgs.msg import String
import aiml
import atexit

kb_file = "kb.p"      # default name for the knowledge base

botAttributes = {
    "name":           "Ken",
    "master":         "the IEEE ENCS Humanoid Robot Team",
    "gender":         "male",
    "location":       "Touchstone 3D, Cary, North Carolina", # fetch this info from internet later
    "birthplace":     "Raleigh, North Carolina",
}

class AIRespondNode(object):
    def __init__(self):
        self.bot = aiml.Kernel()
        self.bot.learn("aiml-startup.xml")
        self.bot.respond("load aiml b")

        for key, value in botAttributes.iteritems():
            self.bot.setBotPredicate(key, value)
        self.session_id = "test1234"
        self.current_face = None

        self.load_kb()

        self.pub = rospy.Publisher('say', String)
        rospy.Subscriber('recognized_speech', String, self.on_recognized_speech)
        rospy.Subscriber('recognized_face', String, self.on_recognized_face)
        rospy.init_node('ai_respond_node')

    def load_kb(self):
        # if kb found, load file to memory
        # otherwise just create a new dict
        if os.path.exists(kb_file):
            self.kb = pickle.load( open( kb_file, "rb" ) )
            for pred,value in self.kb.items():
                self.bot.setPredicate(pred, value, self.session_id)
        else:
            self.kb = self.bot.getSessionData(self.session_id)

    def on_recognized_speech(self, msg):
        rospy.loginfo(rospy.get_caller_id() + ": I heard: %s", msg.data)
        utterance = self.bot.respond(msg.data, self.session_id)
        self.pub.publish(utterance)
        rospy.loginfo(rospy.get_caller_id() + ": I said: %s", utterance)

    def on_recognized_face(self, msg):
        # face signature still to be determined - assuming a string for now
        rospy.loginfo(rospy.get_caller_id() + ": face recognized: %s", msg.data)
        self.current_face = msg.data
        face = self.bot.getPredicate("face", self.session_id)
        if (self.current_face != face):
            self.current_face = face
            rospy.loginfo(rospy.get_caller_id() + ": setting user face to: %s", face)
            # respond as if greeted
            utterance = self.bot.respond("hi " + self.bot.getPredicate("name"), self.session_id)
            self.pub.publish(utterance)
            rospy.loginfo(rospy.get_caller_id() + ": I said: %s", utterance)

    def save_kb(self):
        rospy.loginfo(rospy.get_caller_id() + ": I received a kill signal %s")
        rospy.loginfo(rospy.get_caller_id() + ": Writing KB to %s", kb_file)
        self.kb = self.bot.getSessionData(self.session_id)
        pickle.dump(self.kb, open( kb_file, "wb" ) )
        rospy.loginfo(rospy.get_caller_id() + ": Exiting AI Respond Node")

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        ai = AIRespondNode()

        atexit.register(ai.save_kb)
        ai.run()
    except rospy.ROSInterruptException: pass
