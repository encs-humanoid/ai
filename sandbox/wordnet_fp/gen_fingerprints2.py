#!/usr/bin/python
#===================================================================
# Derive fingerprint using multiple, randomly assigned bits per lemma
# For each lemma, assign a set of B random bit positions to represent it.  
# Build the fingerprint for each lemma by taking the union of the bit
# positions for the related lemmas.
# Copyright 2014, IEEE ENCS Humanoid Robot Project
#===================================================================

from nltk.corpus import wordnet as wn
from random import randrange, random

import gen_fingerprints

class WordnetFingerprintGenerator:
	"""
	Given the size of the fingerprint and the number of bits per lemma,
	"""
	def __init__(self, bits_per_lemma=40, fp_length=2048):
		self.bits_per_lemma = bits_per_lemma
		self.fp_length = fp_length
		self.lemma_bits = dict()	# dict mapping lemma names to the set of random bits assigned to it
		self.lemma_fps = dict()		# dict mapping lemma names to generated fingerprints

	def get_fingerprint(self, lemma):
		if self.lemma_fps.has_key(lemma):
			return self.lemma_fps[lemma]
		else:
			return self._generate_fingerprint(lemma) # has side effect of caching fingerprint

	def _generate_fingerprint(self, lemma):
		wordnet_lemma = wn.lemma(lemma)
		wordnet_lemmas = gen_fingerprints.get_related_lemmas(wordnet_lemma) 

		lemmas = [l.synset().name() + "." + l.name() for l in wordnet_lemmas]

		fp = set()
		for lemma in lemmas:
			bits = self._get_bits_for_lemma(lemma)
			fp.update(bits)
		return fp


	def _get_bits_for_lemma(self, lemma):
		if not self.lemma_bits.has_key(lemma):
			bits = set()
			while len(bits) < self.bits_per_lemma:
				bits.add(randrange(self.fp_length))
			self.lemma_bits[lemma] = bits

		return self.lemma_bits[lemma]

if __name__ == "__main__":
	generator = WordnetFingerprintGenerator(bits_per_lemma=5, fp_length=4096)

	lemmas_list = list()
	with open('vocab_lemmas.txt', 'r') as f:
		for line in f:
			lemmas_list.append(line.strip())

	with open('fingerprints.txt', 'w') as f:
		i = 0
		for lemma in lemmas_list:
			print i, lemma
			i += 1
			fp = generator.get_fingerprint(lemma)
			f.write(lemma + ":" + str(sorted(fp)) + "\n")


