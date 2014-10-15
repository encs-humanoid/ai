#!/usr/bin/python
#===================================================================
# This codelet extracts all lemmas from WordNet by looking up
# the synsets for each lemma name and collecting the associated
# lemmas into a set.  Lemmas with a dot in the name are excluded.
# Copyright 2014, IEEE ENCS Humanoid Robot Project
#===================================================================

from __future__ import division
from nltk.corpus import wordnet as wn

lemmas = set()
for lemma_name in wn.all_lemma_names():
	for synset in wn.synsets(lemma_name):
		for lemma in synset.lemmas():
			if lemma.name().find('.') < 0: # exlude lemmas with a dot in the name, because WN cannot parse them
				lemmas.add(synset.name() + '.' + lemma.name())

with open('all_lemmas.txt', 'w') as f:
	for lemma in lemmas:
		f.write(lemma + "\n")
