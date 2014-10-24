#!/usr/bin/python
#===================================================================
# Derive fingerprint using multiple, randomly assigned bits per word
# Copyright 2014, IEEE ENCS Humanoid Robot Project
#===================================================================

from __future__ import division

import sys
import numpy as np

import nltk
from nltk.corpus import gutenberg
from nltk.corpus import wordnet as wn
from random import randrange, random

class LexicalFingerprintGenerator:
	"""
	Given the size of the fingerprint and the number of bits per word,
	"""
	def __init__(self, bits_per_word=40, fp_length=2048):
		self.bits_per_word = bits_per_word
		self.fp_length = fp_length
		self.start_bits = dict()	# dict mapping start letter to the set of random bits assigned to it
		self.pair_bits = dict()		# dict mapping letter pairs to the set of random bits assigned to it
		self.end_bits = dict()		# dict mapping end letter to the set of random bits assigned to it
		self.lemma_bits = dict()	# dict mapping a lemma to the set of random bits assigned to it
		self.word_fps = dict()		# dict mapping word names to generated fingerprints

	def get_fingerprint(self, word):
		if self.word_fps.has_key(word):
			return self.word_fps[word]
		else:
			return self._generate_fingerprint(word) # has side effect of caching fingerprint

	def _generate_fingerprint(self, word):
		fp = set()
		if len(word) > 0:
			# set bits for starting letter
			bits = self._get_bits(self.start_bits, word[0])
			fp.update(bits)
			# set bits for ending letter
			bits = self._get_bits(self.end_bits, word[-1])
			fp.update(bits)
			for synset in wn.synsets(word):
				for lemma in synset.lemmas():
					bits = self._get_bits(self.lemma_bits, synset.name() + "." + lemma.name())
					fp.update(bits)
		if len(word) > 1:
			# set bits for pairs of letters
			for i in xrange(len(word) - 1):
				letter_sequence = word[i:i+2]
				bits = self._get_bits(self.pair_bits, letter_sequence)
				fp.update(bits)
		self.word_fps[word] = fp	# put word and fp in cache
		return fp
			

	def _get_bits(self, bit_dict, key):
		if not bit_dict.has_key(key):
			bits = set()
			# use while loop to avoid duplicates giving fewer than desired number of bits
			while len(bits) < self.bits_per_word:
				bits.add(randrange(self.fp_length))
			bit_dict[key] = bits

		return bit_dict[key]


	def _describe_constituents(self, bit_dict, fp):
		constituents = list()
		for key in bit_dict.keys():
			bits = bit_dict[key]
			if (len(bits & fp) == len(bits)):
				constituents.append(key)
		return constituents


	def describe_constituents(self, fp):
		constituents = list()
		constituents.append(('start', self._describe_constituents(self.start_bits, fp)))
		constituents.append(('pair', self._describe_constituents(self.pair_bits, fp)))
		constituents.append(('end', self._describe_constituents(self.end_bits, fp)))
		constituents.append(('lemma', self._describe_constituents(self.lemma_bits, fp)))
		return constituents


if __name__ == "__main__":
	if len(sys.argv) != 2:
		print "Usage: " + sys.argv[0] + " <filename>"
		sys.exit(1)

	input_file = sys.argv[1]

	generator = LexicalFingerprintGenerator(bits_per_word=16, fp_length=65536)

	input_words = set(gutenberg.words(input_file))

	with open('lexical_fp.txt', 'w') as f:
		i = 0
		for word in input_words:
			print i, word
			i += 1
			fp = generator.get_fingerprint(word)
			f.write(word + ":" + str(sorted(fp)) + "\n")

	while True:
		line = raw_input("Enter some text: ")

		print "Processing:", line.strip()
		query_sentence = nltk.word_tokenize(line.strip())
		print "Finding matching words..."
		for query_word in query_sentence:
			query_fp = generator.get_fingerprint(query_word)

			results = list()
			result_score = list()
			result_fps = list()
			print len(generator.word_fps.keys())
			for word in generator.word_fps.keys():
				test_fp = generator.get_fingerprint(word)

				result_fp = test_fp & query_fp
				#score = len(test_fp & query_fp) / max(len(test_fp), len(query_fp))
				score = len(result_fp)
				
				results.append(word)
				result_score.append(score)
				result_fps.append(result_fp)

			print "Closest matches for", query_word
			print str(wn.synsets(query_word))
			order = np.argsort(result_score)
			for i in xrange(min(10, len(results))):
				score = result_score[order[-(i+1)]]
				if score > 0:
					print score, results[order[-(i+1)]], str(generator.describe_constituents(result_fps[order[-(i+1)]]))


