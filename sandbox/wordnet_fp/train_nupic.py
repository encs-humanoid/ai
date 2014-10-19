#!/usr/bin/python
#===================================================================
# Train the NuPIC spatial pooler to recognize a set of fingerprints
# Copyright 2014, IEEE ENCS Humanoid Robot Project
#===================================================================
from __future__ import division

import os.path
import pickle
from random import randrange, random, shuffle
import sys
import time

import numpy as np

#from nupic.research.spatial_pooler import SpatialPooler as SP
from nupic.bindings.algorithms import SpatialPooler as SP
from nupic.research.TP10X2 import TP10X2 as TP

import wordnet_fp

class LemmaSDR():
	"""
	A lemma and its fingerprint and NuPIC generated SDR
	In addition to relating the fingerprint and SDR to a lemma, this class
	also tracks the training statistics for determining when the spatial
	pooler has converged.
	"""

	def __init__(self, lemma, fp):
		self.lemma = lemma
		self.fp = fp
		self.sdrs = dict()		# maps SDR to count
		self.last_sdr_key = None
		self.predicted = False 		# flags if count of last_sdr_key was incremented on last call to set_sdr


	def set_sdr(self, sdr):
		sdr_key = str(sdr)
		if not self.sdrs.has_key(sdr_key):
			self.sdrs[sdr_key] = 0
		self.sdrs[sdr_key] += 1
		if sdr_key == self.last_sdr_key:
			self.predicted = True
		else:
			self.predicted = False
		self.last_sdr_key = sdr_key


	def is_bit_set(self, i):
		try:
			return self.fp.index(i) >= 0
		except: # catch ValueError if i is not found in the list
			return 0


class SPTrainer():
  
	"""Trainer for the spatial pooler.
	 Takes fingerprints from an input file and feeds them to the
	 spatial pooler.  Captures resulting SDR and monitors convergence
	 of the spatial pooler.  Convergence is defined as the percentage
	 of input fingerprints which are reliably mapped to the same output
	 SDR each time the fingerprint is presented to the spatial pooler.
	 Therefore, convergence is defined with two parameters:
	    the fraction of input fingerprints which are reliably matched
	    the fraction of presentations which must produce same SDR for
		    the fingerprint to be considered reliable
	"""

	def __init__(self, fp_length):
		"""
		 Parameters:
		 ----------
		 fp_length	:	The length of a fingerprint.
		 """
		self.fp_length = fp_length
		self.num_columns = 2048
		self.input_array = np.zeros(self.fp_length, dtype="int32")
		self.active_array = np.zeros(self.num_columns, dtype="int32")
		self.lemma_to_sdr = dict()

		self.sp = SP((self.fp_length, 1), 
			     (self.num_columns, 1),
			     potentialRadius = 512,
			     numActiveColumnsPerInhArea = int(0.02*self.num_columns),
			     globalInhibition = True,
			     synPermActiveInc = 0.0, # default 0.01
			     synPermInactiveDec = 0.0, # default 0.01
			     spVerbosity = 1,
			     maxBoost = 1.0,    # down from 10
			     potentialPct = 0.8 # up from .5
			     )

    
	def run(self, lemma, fp):
		"""Run the spatial pooler with the input fingerprint"""

		#clear the input_array to zero before creating a new input vector
		self.input_array[0:] = 0
		self.input_array[list(fp)] = 1

		#active_array[column]=1 if column is active after spatial pooling
		self.sp.compute(self.input_array, True, self.active_array)

		sdr = self.active_array.nonzero()

		if not self.lemma_to_sdr.has_key(lemma):
			self.lemma_to_sdr[lemma] = LemmaSDR(lemma, fp)
		self.lemma_to_sdr[lemma].set_sdr(sdr)


class TPTrainer():
  
	"""Trainer for the temporal pooler.
	Takes word fingerprints from an input file and feeds them to the
	temporal pooler by first getting the SDR from the spatial pooler
	and then passing that to the temporal pooler.
	"""

	def __init__(self, sp_trainer):
		"""
		Parameters:
		----------
		sp_trainer	:	The spatial pooler trainer
		"""
		self.sp_trainer = sp_trainer
		self.input_array = np.zeros(self.sp_trainer.fp_length, dtype="int32")
		self.active_array = np.zeros(self.sp_trainer.num_columns, dtype="int32")
		self.is_learning = True
		self.compute_inference = False

		self.tp = TP(numberOfCols=self.sp_trainer.num_columns, cellsPerColumn=2,
			    initialPerm=0.5, connectedPerm=0.5,
			    minThreshold=10, newSynapseCount=10,
			    permanenceInc=0.1, permanenceDec=0.0,
			    activationThreshold=5,
			    globalDecay=0, burnIn=1,
			    checkSynapseConsistency=False,
			    pamLength=10)

    
	def run(self, fp):
		"""Run the spatial pooler and temporal pooler with the input fingerprint"""

		# clear the input_array to zero before creating a new input vector
		self.input_array[0:] = 0
		self.input_array[list(fp)] = 1

		# active_array[column] = 1 if column is active after spatial pooling
		self.sp_trainer.sp.compute(self.input_array, False, self.active_array)
		self.tp.compute(self.active_array, enableLearn = self.is_learning, computeInfOutput = self.compute_inference)

		if self.compute_inference:
			self.predicted_sdr = set(self.tp.getPredictedState().max(axis=1).nonzero()[0].flat)
			lemma_sdrs = np.array([l for l in self.sp_trainer.lemma_to_sdr.values()])
			# convert string key to list by stripping front and end:
			# Example: (array([   8,   12, ... , 1018]),)
			all_lemma_sdrs = [set(eval(l.last_sdr_key[7:-3])) for l in lemma_sdrs]
			_, indexes = wordnet_fp.find_matching(self.predicted_sdr, all_lemma_sdrs, 1, 10)
			self.predicted_lemmas = lemma_sdrs[indexes]

def process_fingerprints(filename, process):
	with open(filename, "r") as f:
		N = 0
		for line in f:
			values = line.strip().split(":")
			lemma = values[0]
			fp = set(eval(values[1]))
			process(lemma, fp)
			N += 1
		return N
    

def train_spatial_pooler():
	if os.path.exists("spatial_pooler.p"):
		print "load previously saved spatial pooler"
		with open("spatial_pooler.p", "r") as f:
			trainer = pickle.load(f)
	else:
		print "instantiate new spatial pooler"
		trainer = SPTrainer(4096)

	round = 0
	percent_prediction = 0
	while round < 100 and percent_prediction < 100:
		print "==================================================="
		round += 1
		print "round", round
		N = process_fingerprints("train_lemmas.txt",
			lambda lemma, fp:
				trainer.run(lemma, fp))

		count = 0
		for lemma in trainer.lemma_to_sdr.keys():
			if trainer.lemma_to_sdr[lemma].predicted == True:
				count += 1
			#lemmaSDR = trainer.lemma_to_sdr[lemma]
			#print lemma, lemmaSDR.sdrs[lemmaSDR.last_sdr_key]
		percent_prediction = (count / N) * 100
		print "N=", N, "count=", count, "pct=", percent_prediction

	with open("spatial_pooler.p", "w") as f:
		pickle.dump(trainer, f)


def load_htm():
	if os.path.exists("spatial_pooler.p"):
		print "load previously saved spatial pooler"
		with open("spatial_pooler.p", "r") as f:
			sp_trainer = pickle.load(f)
	else:
		print "A previously saved spatial pooler is required to train the temporal pooler"
		sys.exist(1)

	# Load or instantiate temporal pooler
	if os.path.exists("temporal_pooler.p"):
		print "load previously saved temporal pooler"
		with open("temporal_pooler.p", "r") as f:
			tp_trainer = pickle.load(f)
		tp_trainer.tp.loadFromFile("temporal_pooler.tp")
		tp_trainer.sp_trainer = sp_trainer	# link the spatial and temporal poolers
	else:
		tp_trainer = TPTrainer(sp_trainer)

	return sp_trainer, tp_trainer


def train_temporal_pooler():
	sp_trainer, tp_trainer = load_htm()

	# Train on sentences
	round = 0
	while round < 10:
		print "==================================================="
		round += 1
		print "round", round
		with open("train_sentences.txt", "r") as f:
			N = 0
			start_time = time.time()
			for line in f:
				# expecting line in the format <word>:<pos>:<fp>
				values = line.strip().split(":")
				fp = set(eval(values[2]))
				tp_trainer.run(fp)
				N += 1
				if N % 1000 == 0:
					rate = N / (time.time() - start_time)
					print "Processed", N, "Rate", rate, "per sec"

				# TODO send reset if anomaly score exceeds a threshold

	# Write out trained temporal pooler
	tp_trainer.tp.saveToFile("temporal_pooler.tp")
	with open("temporal_pooler.p", "w") as f:
		tp_trainer.sp_trainer = None	# don't duplicate the SP inside the TP; keep in separate files
		pickle.dump(tp_trainer, f)
	


if __name__ == "__main__":
	if len(sys.argv) != 2:
		print "Usage: " + sys.argv[0] + " sp|tp"
		print "\tsp\ttrain spatial pooler"
		print "\ttp\ttrain temporal pooler"
		sys.exit(1)

	command = sys.argv[1]

	if "sp" == command:
		train_spatial_pooler()
	elif "tp" == command:
		train_temporal_pooler()
	else:
		print "Unrecognised command:", command
		sys.exit(1)

