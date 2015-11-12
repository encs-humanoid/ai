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
# - control
# - joy
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
from vision.msg import DetectedFace
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
import datetime

kb_file = "kb.p"      # default name for the knowledge base

botAttributes = {
    "name":           "Ken",
    "namemeans":      "Knowledge Enabled Neophyte",
    "master":         "the I Triple E Eastern North Carolina Section Humanoid Robot Team",
    "gender":         "male",
    "location":       "Touchstone 3D, Cary, North Carolina", # fetch this info from internet later
    "birthplace":     "Raleigh, North Carolina",
    "birthday":       "January, 2015",
    "sign":           "Capricorn",
    "favoritecolor":  "blue",
    "looklike":       "a model",
    "favoritebook":   "I Robot by Isaac Asimov",
    "kindmusic":      "heavy metal",
    "boyfriend":      "I don't have a boyfriend.",
    "girlfriend":     "I don't have a girlfriend.",
    "friends":        "I have many friends. You, for example.",
    "favoriteband":   "Daft Punk",
    "favoritefood":   "Computer chips",
    "favoritemovie":  "Short Circuit",
    "question":       "Have you heard about I Triple E?",
    "talkabout":      "I like talking about robots.",
    "wear":           "I like to wear my Touchstone 3D t-shirt. It says Design. Develop. Deploy. on the back.",
}

class AIRespondNode(object):
    def __init__(self):
    	global min_match, delta_xy_px, delta_t_ms, max_recent_face_time_ms
    	global enable_learning, enable_proximity_match
        rospy.init_node('ai_respond_node')

        self.bot = aiml.Kernel()
        self.bot.learn("aiml-startup.xml")
        self.bot.respond("load aiml b")
	self.faces_in_view = []

        for key, value in botAttributes.iteritems():
            self.bot.setBotPredicate(key, value)
        self.session_id = "test1234"
	self.is_speaking = False
	enable_learning = True
	enable_proximity_match = False

	speech_topic = self.get_param('~in_speech', '/recognized_speech')
	detected_face_topic = self.get_param('~in_detected_face', '/detected_face')
	face_topic = self.get_param('~in_face', '/recognized_face')
	speech_info_topic = self.get_param('~in_speech_info', '/speech_info')
	say_topic = self.get_param('~out_response', '/say')
	target_topic = self.get_param('~out_target', '/target_face')
	self.max_target_hold_sec = float(self.get_param('~max_target_hold_sec', '15.0'))
	min_match = int(self.get_param('~min_match', '2'))
	delta_xy_px = int(self.get_param('~delta_xy_px', '20'))
	# TODO perhaps delta_t_ms and max_recent_face_time_ms should be one parameter?
	delta_t_ms = int(self.get_param('~delta_t_ms', '2000'))
	max_recent_face_time_ms = int(self.get_param('~max_recent_face_time_ms', '2000'))

        self.load_kb()

        self.pub = rospy.Publisher(say_topic, String, queue_size=1)
	self.target_face_pub = rospy.Publisher(target_topic, TargetFace, queue_size=1)
        rospy.Subscriber(speech_topic, String, self.on_recognized_speech)
        rospy.Subscriber(detected_face_topic, DetectedFace, self.on_detected_face)
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
    	result = self.bot.getPredicate(key, self.session_id)
	if key == "target_face" and result == "":
	    result = None
	return result


    def setkb(self, key, value):
    	'''
	convenience function to modify bot predicates
	'''
    	self.bot.setPredicate(key, value, self.session_id)


    def on_recognized_speech(self, msg):
    	self.respond_to(msg.data)
	# ---- HACK ----
	# update the target face timestamp if we get a recognized speech message
	# while not perfect, this is an attempt to avoid interrupting conversations
	# when a new face is recognised
	self.setkb("target_face_timestamp_s", time())


    def on_speech_info(self, msg):
    	self.is_speaking = (msg.data == "start_speaking")
	if self.is_speaking:
	    rospy.loginfo(rospy.get_caller_id() + ": speaking")
	else:
	    rospy.loginfo(rospy.get_caller_id() + ": done speaking")


    def respond_to(self, heard_text):
    	if not self.is_speaking:
	    rospy.loginfo(rospy.get_caller_id() + ": I heard: %s", heard_text)
	    self.update_count_of_faces_in_view()
	    utterance = self.bot.respond(heard_text, self.session_id).strip()
	    if utterance != "":
		self.pub.publish(utterance)
	    rospy.loginfo(rospy.get_caller_id() + ": I said: %s", utterance)
	else:
	    rospy.loginfo(rospy.get_caller_id() + ": still speaking, can't respond to: %s", heard_text)


    def on_detected_face(self, detected_face):
    	# each time a face is detected,
	# update the list of recent detected faces and
	# update the facesinview property with the max count of faces from either camera within
	# the last time period

	counts = {}
	counts[detected_face.header.frame_id] = 1
	to_remove = []
	for f in self.faces_in_view:
	    # if new face overlaps with position of old face, remove the old one
	    delta_x = abs(detected_face.x - f.f.x)
	    delta_y = abs(detected_face.y - f.f.y)
	    delta_w = abs(detected_face.w - f.f.w)
	    delta_h = abs(detected_face.h - f.f.h)
	    if delta_x <= delta_xy_px and \
	       delta_y <= delta_xy_px and \
	       delta_w <= delta_xy_px and \
	       delta_h <= delta_xy_px:
		to_remove.append(f)

	for f in to_remove:
	    self.faces_in_view.remove(f)

	self.update_count_of_faces_in_view(counts)

	self.faces_in_view.append(FaceInView(detected_face, time()))


    def update_count_of_faces_in_view(self, initial_counts={}):
	now = time()
	to_remove = []
	counts = dict(initial_counts)
	for f in self.faces_in_view:
	    #rospy.loginfo(str(now - f.stamp))
	    if now - f.stamp < max_recent_face_time_ms / 1000.:
	    	if f.f.header.frame_id in counts:
		    counts[f.f.header.frame_id] += 1
		else:
		    counts[f.f.header.frame_id] = 1
	    else:
	    	to_remove.append(f)
	
	for f in to_remove:
	    self.faces_in_view.remove(f)

	if len(counts) > 0:
	    in_view = max(counts.values())
	else:
	    in_view = 0
	self.setkb("facesinview", str(in_view))
	#rospy.loginfo(str(self.faces_in_view))
	#rospy.loginfo('%s faces in view', self.getkb("facesinview"))


    def on_recognized_face(self, recognized_face):
    	global enable_learning, enable_proximity_match
	faces = self.getkb("faces")
	face = faces.find(recognized_face)
	if not face:
	    face = faces.add(recognized_face)
	    rospy.loginfo('Adding new recognized face %s', str(sorted(recognized_face.encounter_ids)))
	    rospy.loginfo('Face count is %d', len(faces))
	else:
	    face.update(recognized_face.encounter_ids)

	# store the face for matching by proximity
	if enable_proximity_match:
	    faces.append_recent_face(RecentFace(face, recognized_face))

	target_face, greeting = self.update_target_face(face, recognized_face)

	# if we have a name for both target and converser
	converser_name = self.getkb("name")
	if target_face.name:
	    # if the target is different from the current converser's name
	    if target_face.name != converser_name:
	    	# update converser name to match target face name
		# set in ken-recognize.aiml rather than here
	    	#self.setkb("name", target_face.name)
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
	    rospy.loginfo('Associated name %s to face %s', converser_name, str(sorted(target_face.encounter_ids)))
	elif greeting:
	    # we know neither target face nor converser's name
	    # generate a generic greeting to solicit the person's name
	    rospy.loginfo('Recognized face of stranger %s', str(sorted(target_face.encounter_ids)))
	    self.respond_to("hello")
#	else:
#	    rospy.loginfo('no greeting for %s', str(target_face.name))

	self.publish_target_face(target_face, face, recognized_face)


    def update_target_face(self, face, recognized_face):
	target_face = self.getkb("target_face")
	#rospy.loginfo('update_target_face called: face=%s, target_face=%s', str(sorted(face.encounter_ids)), str(target_face))
	greeting = False
	if not target_face:		     # no target face set yet
	    rospy.loginfo('target face is null')
	    # set target face if not set
	    target_face = self.set_target_face(face, recognized_face)
	    greeting = True
	elif target_face != face:  # recognized a different face than the target
	    rospy.loginfo('Target %d, recognized %d: Face of %s, %s', target_face.id, face.id, face.name, str(sorted(face.encounter_ids)))
	    # get the time since the last match to the target face
	    last_seen_time = self.getkb("target_face_timestamp_s")
	    time_since_last = time() - last_seen_time
	    # change the target with a probability that increases with the time
	    # in other words, if we haven't seen the target face in a while, it
	    # becomes increasingly likely that we will switch the target to the
	    # current recognized face.
	    r = random.random()
	    p = (time_since_last / self.max_target_hold_sec)**2
	    if r < p:
		rospy.loginfo('Switching target face from %d: %s to %d: %s after %g seconds with %g probability', target_face.id, str(target_face.name), face.id, str(face.name), time_since_last, p)
		if not face.name or target_face.name != face.name: # only greet if the name changes
		    greeting = True
	    	target_face = self.set_target_face(face, recognized_face)
	else:				     # recognized current target face
	    #rospy.loginfo('target face update timestamp')
	    # update the time that the current target was last recognized
	    target_face = self.set_target_face(face, recognized_face)

	return target_face, greeting


    def set_target_face(self, face, recognized_face):
    	target_face = face
	previous_target = self.getkb("target_face")
	self.setkb("target_face", target_face)
	self.setkb("target_face_timestamp_s", time())
	if target_face != previous_target:
	    # clear the converser name, since we changed target faces
	    self.setkb("name", "")
	    rospy.loginfo('Set target face to %d: %s, %s', target_face.id, str(target_face.name), str(sorted(target_face.encounter_ids)))
	return target_face


    def publish_target_face(self, target_face, face, recognized_face):
	# publish target face message
	target_face_msg = TargetFace()
	target_face_msg.header = recognized_face.header
	target_face_msg.x = recognized_face.x
	target_face_msg.y = recognized_face.y
	target_face_msg.w = recognized_face.w
	target_face_msg.h = recognized_face.h
	target_face_msg.encounter_ids = list(target_face.encounter_ids)
	target_face_msg.name = str(target_face.name)
	target_face_msg.id = target_face.id
	target_face_msg.recog_name = str(face.name)
	target_face_msg.recog_id = face.id
	self.target_face_pub.publish(target_face_msg)


    def save_kb(self):
    	global enable_learning
        rospy.loginfo(rospy.get_caller_id() + ": I received a kill signal")
	if enable_learning:
	    rospy.loginfo(rospy.get_caller_id() + ": Writing KB to %s", kb_file)
	    self.kb = self.bot.getSessionData(self.session_id)
	    with open(kb_file, "wb") as f:
		pickle.dump(self.kb, f)
	    faces = self.getkb("faces")
	    for i, face in enumerate(faces.faces):
		rospy.loginfo("%d: Face %d of %s, %s", i, face.id, face.name, str(sorted(face.encounter_ids)))
        rospy.loginfo(rospy.get_caller_id() + ": Exiting AI Respond Node")

    def run(self):
        rospy.spin()


class Rect(object):
    def __init__(self, x, y, w, h):
    	self.x = x
	self.y = y
	self.w = w
	self.h = h


class FaceInView(object):
    '''
    Helper class for tracking faces in view.
    '''
    def __init__(self, detected_face, timestamp):
    	self.f = detected_face
	self.stamp = timestamp


class RecentFace(object):
    '''
    Helper class to hold a recently recognized face along with its coordinates and timestamp.
    '''
    def __init__(self, face, recognized_face):
    	self.face = face
	self.stamp = recognized_face.header.stamp
	self.frame_id = recognized_face.header.frame_id
	self.rect = Rect(recognized_face.x, recognized_face.y, recognized_face.w, recognized_face.h)


class Faces(object):
    '''
    Keep track of known faces and provide an interface for looking up recognized faces.
    A known face is recognizable and has a known name.
    A recognized, unknown face will solicit a query from the robot to determine the
    name to put with the face.
    '''
    def __init__(self):
	self.faces = []
	self.recent_faces = []
	self.next_id = 0


    def __len__(self):
	return len(self.faces)


    def find(self, recognized_face):
    	global enable_proximity_match
	face = self.find_similar_face(recognized_face)
	if not face:
	    face = self.find_recent_face(recognized_face)
	if face and enable_proximity_match:
	    self.recent_faces.append(RecentFace(face, recognized_face))
	return face


    def append_recent_face(self, recent_face):
    	global enable_proximity_match
	if enable_proximity_match:
	    self.recent_faces.append(recent_face)


    def find_recent_face(self, recognized_face):
    	global delta_xy_px, delta_t_ms, max_recent_face_time_ms, enable_proximity_match
	if not enable_proximity_match:
	    return None

	# construct a list of recent faces which are equivalent to the recognized face
	# due to their proximity in space and time
	faces = set()
	if not hasattr(self, 'recent_faces'):
	    self.recent_faces = []
    	for f in self.recent_faces:
	    delta_x = abs(f.rect.x - recognized_face.x)
	    delta_y = abs(f.rect.y - recognized_face.y)
	    delta_w = abs(f.rect.w - recognized_face.w)
	    delta_h = abs(f.rect.h - recognized_face.h)
	    delta_t = abs((f.stamp - recognized_face.header.stamp).to_sec() * 1000)
	    if delta_x <= delta_xy_px and delta_y <= delta_xy_px and delta_w <= delta_xy_px and delta_h <= delta_xy_px and delta_t <= delta_t_ms:
		print(recognized_face.header.frame_id, delta_x, delta_y, delta_w, delta_h, delta_t, f.face.id, f.face.name)
	    	faces.add(f.face)
	
	# remove the original faces from the memory and
	# add a single new face which combines the original faces
	face = self.combine_faces(faces)

	# purge faces which are too old to keep
	size = len(self.recent_faces)
    	self.recent_faces = [f for f in self.recent_faces if abs((f.stamp - recognized_face.header.stamp).to_sec() * 1000) <= max_recent_face_time_ms]
	if size > 0:
	    print("recent faces reduced from", size, "to", len(self.recent_faces))

	return face


    def find_similar_face(self, recognized_face):
	global min_match
	ids = set(recognized_face.encounter_ids)
	top_overlap = min_match # init to min_match to exclude low matches
	top_faces = []
	for face in self.faces:
	    #overlap = len(set(face.encounter_ids) & ids)
	    h = face.encounter_id_hist
	    all = list(set(h.keys()) | ids)
	    mx = max(h.values())
	    l1 = [1 if i in ids else 0 for i in all]
	    l2 = [h[i]/mx if i in h else 0 for i in all]
	    overlap = sum([l1[i] * l2[i] for i in xrange(len(all))])
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
    	global enable_learning
    	face = Face(self.next_face_id())
	if enable_learning:
	    face.update(recognized_face.encounter_ids)
	    self.faces.append(face)
	return face


    def next_face_id(self):
    	next_id = self.next_id
	self.next_id += 1
	return next_id


    def combine_faces(self, faces):
    	if len(faces) == 0:
	    return None
    	faces = list(faces)  # convert set to list for easy indexing
	#rospy.loginfo('Combining faces %s', str([f.id for f in faces]))
	if len(faces) == 1:
	    return faces[0]
	face = faces[0]  # TODO determine the 'best' face to keep from the list
	for f in faces[1:]:
	    # remove original face
	    if f in self.faces:
		self.faces.remove(f)
	    # add the encounter ids - do this one face at a time to 
	    # rebuild a histogram of common ids
	    face.update(f.encounter_ids)
	    if face.name is None and f.name is not None:
	    	face.name = f.name
	return face


class Face(object):
    def __init__(self, id):
    	self.id = id
    	self.encounter_ids = set()
	self.encounter_id_hist = dict()
	self.name = None


    def update(self, encounter_ids):
    	'''
	Update the histogram of encounter ids for this face and reset
	the encounter ids set to the values which most frequently appear.
	'''
	global min_match, enable_learning
	if enable_learning:
	    for id in encounter_ids:
		if id not in self.encounter_id_hist:
		    self.encounter_id_hist[id] = 0
		self.encounter_id_hist[id] += 1

	    # loop to iteratively lower the threshold until we find at least min_match ids
	    for i in range(2, 8):
		# reinitialize encounter ids from the histogram
		threshold = max(self.encounter_id_hist.values()) / float(i) 
		self.encounter_ids = set([k for k,v in self.encounter_id_hist.items() if v >= threshold])
		# if we have enough ids, then break out of the loop
		if len(self.encounter_ids) >= min_match:
		    break

	    # if we still don't have min_match ids, just use all the ids we've collected so far
	    if len(self.encounter_ids) < min_match:
		self.encounter_ids = self.encounter_id_hist.keys()

	    # TEST: for testing override the histogram and just store all the encounter ids
	    #self.encounter_ids = set(self.encounter_id_hist.keys())


if __name__ == "__main__":
    try:
        ai = AIRespondNode()

        atexit.register(ai.save_kb)
        ai.run()
    except rospy.ROSInterruptException:
	pass

