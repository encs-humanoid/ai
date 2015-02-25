#!/usr/bin/env python
#===================================================================
# This is the AI Respond Node.
#
# Subscribes To:
# - recognized_speech
# - recognized_face
# - speech_info
#
# Publishes To:
# - say
# - target_face
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

from __future__ import division
from __future__ import print_function
from time import time
from vision.msg import RecognizedFace
from vision.msg import TargetFace
import random
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
    	global min_match
        rospy.init_node('ai_respond_node')

        self.bot = aiml.Kernel()
        self.bot.learn("aiml-startup.xml")
        self.bot.respond("load aiml b")

        for key, value in botAttributes.iteritems():
            self.bot.setBotPredicate(key, value)
        self.session_id = "test1234"
	self.is_speaking = False

	speech_topic = self.get_param('~in_speech', '/recognized_speech')
	face_topic = self.get_param('~in_face', '/recognized_face')
	speech_info_topic = self.get_param('~in_speech_info', '/speech_info')
	say_topic = self.get_param('~out_response', '/say')
	target_topic = self.get_param('~out_target', '/target_face')
	self.max_target_hold_sec = float(self.get_param('~max_target_hold_sec', '15.0'))
	min_match = int(self.get_param('~min_match', '4'))

        self.load_kb()

        self.pub = rospy.Publisher(say_topic, String, queue_size=1)
	self.target_face_pub = rospy.Publisher(target_topic, TargetFace, queue_size=1)
        rospy.Subscriber(speech_topic, String, self.on_recognized_speech)
        rospy.Subscriber(face_topic, RecognizedFace, self.on_recognized_face)
        rospy.Subscriber(speech_info_topic, String, self.on_speech_info)


    def get_param(self, param_name, param_default):
	value = rospy.get_param(param_name, param_default)
	rospy.loginfo('Parameter %s has value %s', rospy.resolve_name(param_name), value)
	return value


    def load_kb(self):
        # if kb found, load file to memory
        # otherwise just create a new dict
        if os.path.exists(kb_file):
	    with open(kb_file, "rb") as f:
		self.kb = pickle.load(f)
            for pred, value in self.kb.items():
	    	self.setkb(pred, value)
	    # clear the input stack in case the program died in the middle of processing
	    self.setkb("_inputStack", [])
	    self.setkb("target_face", None)
	    self.setkb("name", "")
        else:
            self.kb = self.bot.getSessionData(self.session_id)
	    self.setkb("faces", Faces())


    def getkb(self, key):
    	'''
	convenience function to access bot predicates
	'''
    	return self.bot.getPredicate(key, self.session_id)


    def setkb(self, key, value):
    	'''
	convenience function to modify bot predicates
	'''
    	self.bot.setPredicate(key, value, self.session_id)


    def on_recognized_speech(self, msg):
    	self.respond_to(msg.data)


    def on_speech_info(self, msg):
    	self.is_speaking = (msg.data == "start_speaking")
	if self.is_speaking:
	    rospy.loginfo(rospy.get_caller_id() + ": speaking")
	else:
	    rospy.loginfo(rospy.get_caller_id() + ": done speaking")


    def respond_to(self, heard_text):
    	if not self.is_speaking:
	    rospy.loginfo(rospy.get_caller_id() + ": I heard: %s", heard_text)
	    utterance = self.bot.respond(heard_text, self.session_id)
	    self.pub.publish(utterance)
	    rospy.loginfo(rospy.get_caller_id() + ": I said: %s", utterance)
	else:
	    rospy.loginfo(rospy.get_caller_id() + ": still speaking, can't respond to: %s", heard_text)


    def on_recognized_face(self, recognized_face):
	faces = self.getkb("faces")
	face = faces.find(recognized_face)
	if not face:
	    face = faces.add(recognized_face)
	    rospy.loginfo('Adding new recognized face %s', str(recognized_face.encounter_ids))
	    rospy.loginfo('Face count is %d', len(faces))
	else:
	    face.update(recognized_face)

	target_face, greeting = self.update_target_face(face)

	# if we have a name for both target and converser
	converser_name = self.getkb("name")
	if target_face.name:
	    # if the target is different from the current converser's name
	    if target_face.name != converser_name:
	    	# update converser name to match target face name
	    	self.setkb("name", target_face.name)
		rospy.loginfo('Recognized face of %s', str(target_face.name))
		# trigger response which includes converser's name
		self.respond_to("recognize " + target_face.name)
#	    else:
#		rospy.loginfo('got target=%s and convers=%s', str(target_face.name), converser_name)
	elif converser_name and converser_name.strip():
	    # we have a converser name, but no target name.  Assume that
	    # the converser name applies to the target, since we cleared
	    # the converser name when the target face was last changed.
	    target_face.name = converser_name
	    rospy.loginfo('Associated name %s to face %s', converser_name, str(target_face.encounter_ids))
	elif greeting:
	    # we know neither target face nor converser's name
	    # generate a generic greeting to solicit the person's name
	    rospy.loginfo('Recognized face of stranger %s', str(target_face.encounter_ids))
	    self.respond_to("hello")
#	else:
#	    rospy.loginfo('no greeting for %s', str(target_face.name))


    def update_target_face(self, face):
	target_face = self.getkb("target_face")
	#rospy.loginfo('update_target_face called: face=%s, target_face=%s', str(face.encounter_ids), str(target_face))
	greeting = False
	if not target_face:		     # no target face set yet
	    rospy.loginfo('target face is null')
	    # set target face if not set
	    target_face = self.set_target_face(face)
	    greeting = True
	elif target_face != face:  # recognized a different person than the target
	    rospy.loginfo('Target %d, recognized %d: Face of %s, %s', target_face.id, face.id, face.name, str(face.encounter_ids))
	    # get the time since the last match to the target face
	    last_seen_time = self.getkb("target_face_timestamp_s")
	    time_since_last = time() - last_seen_time
	    # change the target with a probability that increases with the time
	    # in other words, if we haven't seen the target face in a while, it
	    # becomes increasingly likely that we will switch the target to the
	    # current recognized face.
	    r = random.random()
	    p = (time_since_last / self.max_target_hold_sec)**2
	    if random.random() < time_since_last / self.max_target_hold_sec:
		rospy.loginfo('Switching target face from %d: %s to %d: %s after %g seconds with %g probability', target_face.id, str(target_face.name), face.id, str(face.name), time_since_last, p)
		if not face.name or target_face.name != face.name: # only greet if the name changes
		    greeting = True
	    	target_face = self.set_target_face(face)
	else:				     # recognized current target face
	    #rospy.loginfo('target face update timestamp')
	    # update the time that the current target was last recognized
	    target_face = self.set_target_face(face)

	return target_face, greeting


    def set_target_face(self, face):
    	target_face = face
	previous_target = self.getkb("target_face")
	self.setkb("target_face", target_face)
	self.setkb("target_face_timestamp_s", time())
	if target_face != previous_target:
	    # clear the converser name, since we changed target faces
	    self.setkb("name", "")
	    # publish new target face
	    target_face_msg = TargetFace()
	    target_face_msg.encounter_ids = list(face.encounter_ids)
	    self.target_face_pub.publish(target_face_msg)
	    rospy.loginfo('Set target face to %d: %s, %s', target_face.id, str(target_face.name), str(target_face.encounter_ids))
	return target_face


    def save_kb(self):
        rospy.loginfo(rospy.get_caller_id() + ": I received a kill signal %s")
        rospy.loginfo(rospy.get_caller_id() + ": Writing KB to %s", kb_file)
        self.kb = self.bot.getSessionData(self.session_id)
	with open(kb_file, "wb") as f:
	    pickle.dump(self.kb, f)
	faces = self.getkb("faces")
	for i, face in enumerate(faces.faces):
	    rospy.loginfo("%d: Face of %s, %s", i, face.name, str(face.encounter_ids))
        rospy.loginfo(rospy.get_caller_id() + ": Exiting AI Respond Node")

    def run(self):
        rospy.spin()


class Faces(object):
    '''
    Keep track of known faces and provide an interface for looking up recognized faces.
    A known face is recognizable and has a known name.
    A recognized, unknown face will solicit a query from the robot to determine the
    name to put with the face.
    '''
    def __init__(self):
	self.faces = []


    def __len__(self):
	return len(self.faces)


    def find(self, recognized_face):
	global min_match
	ids = set(recognized_face.encounter_ids)
	top_overlap = min_match # init to min_match to exclude low matches
	top_faces = []
	for face in self.faces:
	    overlap = len(face.encounter_ids & ids)
	    if overlap >= top_overlap:
		# track equivalent top matches
	        if overlap > top_overlap:
		    top_faces[:] = []	    # clear list if better match found
		top_faces.append(face) 
	        top_overlap = overlap
	top_face = None
	# look for a face with a name first
	for face in top_faces:
	    if face.name:
	    	top_face = face
		break
	# if no face with a name, choose the most general match
	longest = top_overlap
	if not top_face:
	    for face in top_faces:
		if len(face.encounter_ids) >= longest: # use >= to ensure at least one match
		    top_face = face
		    longest = len(face.encounter_ids)
	return top_face  # may be None if no match was > min_match threshold


    def add(self, recognized_face):
    	face = Face(len(self.faces))
	face.update(recognized_face)
	self.faces.append(face)
	return face


class Face(object):
    def __init__(self, id):
    	self.id = id
    	self.encounter_ids = set()
	self.encounter_id_hist = dict()
	self.name = None


    def update(self, recognized_face):
        for id in recognized_face.encounter_ids:
	    if id not in self.encounter_id_hist:
	    	self.encounter_id_hist[id] = 0
	    self.encounter_id_hist[id] += 1
	threshold = max([cnt for cnt in self.encounter_id_hist.values()]) / 2.0
	for id in recognized_face.encounter_ids:
	    if self.encounter_id_hist[id] >= threshold:
		self.encounter_ids.add(id)


if __name__ == "__main__":
    try:
        ai = AIRespondNode()

        atexit.register(ai.save_kb)
        ai.run()
    except rospy.ROSInterruptException: pass
