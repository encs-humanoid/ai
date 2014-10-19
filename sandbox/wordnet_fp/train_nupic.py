#!/usr/bin/python
#===================================================================
# Train the NuPIC spatial pooler to recognize a set of fingerprints
# Copyright 2014, IEEE ENCS Humanoid Robot Project
#===================================================================
from __future__ import division

import pickle
import numpy as np
from random import randrange, random, shuffle

#from nupic.research.spatial_pooler import SpatialPooler as SP
from nupic.bindings.algorithms import SpatialPooler as SP

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
		 _fpLength	:	The length of a fingerprint.
		 """
		self.fp_length = fp_length
		self.num_columns = 2048
		self.input_array = np.zeros(self.fp_length)
		self.active_array = np.zeros(self.num_columns)
		self.lemma_to_sdr = dict()

		self.sp = SP((self.fp_length, 1), 
			     (self.num_columns, 1),
			     potentialRadius = 512,
			     numActiveColumnsPerInhArea = int(0.02*self.num_columns),
			     globalInhibition = True,
			     synPermActiveInc = 0.01,
			     spVerbosity = 1
			     )

    
	def run(self, lemma, fp):
		"""Run the spatial pooler with the input fingerprint"""

		#clear the input_array to zero before creating a new input vector
		self.input_array[0:] = 0

		for i in fp:
		  self.input_array[i] = 1

		#active_array[column]=1 if column is active after spatial pooling
		self.sp.compute(self.input_array, True, self.active_array)

		sdr = self.active_array.nonzero()

		if not self.lemma_to_sdr.has_key(lemma):
			self.lemma_to_sdr[lemma] = LemmaSDR(lemma, fp)
		self.lemma_to_sdr[lemma].set_sdr(sdr)


def process_fingerprints(filename, process):
	with open(filename, "r") as f:
		N = 0
		lemmas = list()
		fps = list()
		for line in f:
			values = line.strip().split(":")
			lemma = values[0]
			fp = eval(values[1])
			lemmas.append(lemma)
			fps.append(fp)
			N += 1

		# shuffle the order that the lemmas are presented to the spatial pooler
		# This is trying to avoid training cycles where the same sequence of
		# inputs causes the learning algorithm to follow the same pattern of
		# learning and forgetting, which may prevent convergence
		order = [i for i in xrange(N)]
		shuffle(order)
		for i in xrange(N):
			lemma = lemmas[order[i]]
			fp = fps[order[i]]
			process(lemma, fp)
		return N
    

if __name__ == "__main__":
	print "instantiate spatial pooler"
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

