#!/usr/bin/env python
#===================================================================
# This file contains functions for manipulating fingerprints.
# 
# overlap - computes the number of common bits in two fingerprints
#
# Copyright 2014, IEEE ENCS Humanoid Robot Project
#===================================================================

import numpy as np
from nltk.corpus import wordnet as wn

import train_nupic

def overlap(fp1, fp2):
	"""
	Compute the overlap between two fingerprints.
	The fingerprints are sets of numbers, where the numbers
	correspond to the on-bits in the fingerprint.
	"""
	return len(fp1 & fp2)


def find_matching(test_fp, all_fps, threshold, max_to_return):
	"""
	Find the closest matching fingerprints given a query fingerprint and a match threshold
	"""
	overlaps = list()
	for fp in all_fps:
		overlaps.append(overlap(test_fp, fp))
	order = np.argsort(overlaps)
	result_fingerprints = list()
	result_indexes = list()
	for i in xrange(max_to_return):
		index = order[-(i+1)]
		if overlaps[index] >= threshold:
			result_fingerprints.append(all_fps[index])
			result_indexes.append(index)
	return result_fingerprints, result_indexes


class WordFingerprintGenerator():
	"""
	This class generates word fingerprints given WordNet and a database of
	lemma fingerprints.  The word fingerprint is the union of lemma fingerprints
	with the same part of speech.
	"""

	def __init__(self, fingerprints_filename):
		# read in all the lemmas and fingerprints
		self.lemma_sdrs = dict()
		train_nupic.process_fingerprints(fingerprints_filename, lambda lemma, fp: self.lemma_sdrs.update([(lemma, fp)]))
		
	def get_word_fp(self, word, part_of_speech):
		fp = set()
		for synset in wn.synsets(word):
			if synset.pos() == part_of_speech or (part_of_speech == 'a' and synset.pos() == 's'):
				for lemma in synset.lemmas():
					fp.update(self.get_lemma_fp(synset.name() + "." + lemma.name()))
		return fp

	def get_lemma_fp(self, lemma_name):
		if self.lemma_sdrs.has_key(lemma_name):
			return self.lemma_sdrs[lemma_name]
		else:
			return {}






