#!/usr/bin/python
#===================================================================
# Generate fingerprints for the words in the sentences of the corpus
# indicated on the command line.
# Copyright 2014, IEEE ENCS Humanoid Robot Project
#===================================================================

from __future__ import division
from nltk.corpus import wordnet as wn
from nltk.corpus import gutenberg
import sys
import nltk

import wordnet_fp

def get_wordnet_pos(treebank_tag):
	if treebank_tag.startswith('J'):
		return wn.ADJ
	elif treebank_tag.startswith('V'):
		return wn.VERB
	elif treebank_tag.startswith('N'):
		return wn.NOUN
	elif treebank_tag.startswith('R'):
		return wn.ADV
	else:
		return ''


def generate_fingerprints(generator, sent):
	fps = list()
	pos_tagged_sent = nltk.pos_tag(sent)
	# determine if word is 'n' (noun), 'v' (verb), 'r' (adverb), 'a' (adjective)
	for pos_tagged_word in pos_tagged_sent:
		word = pos_tagged_word[0]
		pos = get_wordnet_pos(pos_tagged_word[1])
		if len(pos) > 0:
			fp = generator.get_word_fp(word, pos)
			if len(fp) > 0:
				fps.append((word, pos, fp))
	return fps


if __name__ == "__main__":
	if len(sys.argv) != 2:
		print "Usage: " + sys.argv[0] + " <filename>"
		sys.exit(1)

	input_file = sys.argv[1]

	input_sents = gutenberg.sents(input_file)

	with open("sentences.txt", "a") as f:
		generator = wordnet_fp.WordFingerprintGenerator("fingerprints.txt")
		for sent in input_sents:
			# [(<word>, <pos>, <fp>), ...]
			sentence_fingerprints = generate_fingerprints(generator, sent)
			for (word, pos, fp) in sentence_fingerprints:
				f.write(word + ":" + pos + ":" + str(sorted(list(fp))) + "\n")
