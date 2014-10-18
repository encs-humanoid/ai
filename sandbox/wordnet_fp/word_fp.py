#!/usr/bin/python
#===================================================================
# Usage:  python word_fp.py <word>
# Generates the ambiguous word fingerprint given a word by unioning
# the fingerprints for the lemmas in the word's synsets.
# Copyright 2014, IEEE ENCS Humanoid Robot Project
#===================================================================

import sys

from nltk.corpus import wordnet as wn

import train_nupic

def generate_word_fingerprint(word, lemmas):
	word_fp = set()
	for synset in wn.synsets(word):
		for lemma in synset.lemmas():
			key = synset.name() + "." + lemma.name()
			if lemmas.has_key(key):
			    lemma_fp = lemmas[key]
			    word_fp.update(lemma_fp)
	fp = list(word_fp)
	fp.sort()
	return fp;

def assign(lemmas, lemma, fp):
	lemmas[lemma] = fp

if __name__ == "__main__":
	lemmas = dict()
	train_nupic.process_fingerprints("fingerprints.txt", lambda lemma, fp: assign(lemmas, lemma, fp))
	for word in sys.argv:
		fp = generate_word_fingerprint(word, lemmas)
		print word + ":" + str(fp)

