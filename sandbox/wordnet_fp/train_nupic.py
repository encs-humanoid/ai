#!/usr/bin/python

import numpy as np
from random import randrange, random
from nupic.research.spatial_pooler import SpatialPooler as SP


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
		self.sdrs = dict() # maps SDR to count
		self.last_sdr_key = None


	def set_sdr(self, sdr):
		sdr_key = str(sdr)
		if not self.sdrs.has_key(sdr_key):
			self.sdrs[sdr_key] = 0
		self.sdrs[sdr_key] += 1
		self.last_sdr_key = sdr_key


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
			     synPermActiveInc = 0.01
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

    
# the length of the fingerprint can be determined by `wc -l vocab_lemmas.txt`
print "instantiate spatial pooler"
trainer = SPTrainer(61955)

for i in xrange(10):
	print "round", i
	with open("train_lemmas.txt", "r") as f:
		for line in f:
			values = line.strip().split(":")
			lemma = values[0]
			fp = eval(values[1])
			trainer.run(lemma, fp)

for lemma in trainer.lemma_to_sdr.keys():
	lemmaSDR = trainer.lemma_to_sdr[lemma]
	print lemma, lemmaSDR.sdrs[lemmaSDR.last_sdr_key]

