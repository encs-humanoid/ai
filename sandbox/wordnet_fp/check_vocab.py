#!/usr/bin/python
#===================================================================
# This codelet reads the vocabulary lemmas and verifies that each
# is found in NLTK WordNet.  Some lemmas in WordNet cannot be looked
# up because of parsing errors due to dots (.) in the lemma name.
# Copyright 2014, IEEE ENCS Humanoid Robot Project
#===================================================================

from nltk.corpus import wordnet as wn

with open('vocab_lemmas.txt', 'r') as f:
	for line in f:
		try:
			wn.lemma(line.strip()) # will blow up if line isn't a lemma
		except:
			print line.strip()


