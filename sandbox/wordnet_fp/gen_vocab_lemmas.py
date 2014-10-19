#!/usr/bin/python
#===================================================================
# This codelet builds a vocabulary by taking unique words from the
# book Moby Dick and intersecting this with lemma names in WordNet.
# For each remaining word, the synsets for it are looked up and
# all lemmas extracted.  Lemmas containing a dot in the name are
# filtered out and the remaining lemmas written to a file.
# Copyright 2014, IEEE ENCS Humanoid Robot Project
#===================================================================

from __future__ import division
from nltk.corpus import wordnet as wn
from nltk.corpus import gutenberg
import sys

if len(sys.argv) != 2:
	print "Usage: " + sys.argv[0] + " <filename>"
	sys.exit(1)

input_file = sys.argv[1]

input_words = set(gutenberg.words(input_file))
lemma_names = set(wn.all_lemma_names())
# keep all Moby Dick words that are also found in wordnet
vocab = input_words & lemma_names

lemmas = set()
for lemma_name in vocab:
	for synset in wn.synsets(lemma_name):
		for lemma in synset.lemmas():
			if lemma.name().find('.') < 0: # exlude lemmas with a dot in the name, because WN cannot parse them
                               lemmas.add(synset.name() + '.' + lemma.name())


with open('vocab_lemmas.txt', 'w') as f:
	for lemma in lemmas:
		f.write(lemma + "\n")

#5 25 26 austen-emma.txt
#5 26 17 austen-persuasion.txt
#5 28 22 austen-sense.txt
#4 34 79 bible-kjv.txt
#5 19 5 blake-poems.txt
#4 19 14 bryant-stories.txt
#4 18 12 burgess-busterbrown.txt
#4 20 13 carroll-alice.txt
#5 20 12 chesterton-ball.txt
#5 23 11 chesterton-brown.txt
#5 18 11 chesterton-thursday.txt
#4 21 25 edgeworth-parents.txt
#5 26 15 melville-moby_dick.txt
#5 52 11 milton-paradise.txt
#4 12 9 shakespeare-caesar.txt
#4 12 8 shakespeare-hamlet.txt
#4 12 7 shakespeare-macbeth.txt
#5 36 12 whitman-leaves.txt
