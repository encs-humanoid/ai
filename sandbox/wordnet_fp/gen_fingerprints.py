#!/usr/bin/python
#===================================================================
# This codelet generates semantic fingerprints given a file of
# WordNet lemmas.  The order of lemmas in the file defines the bit
# position in the fingerprint.  WordNet associations to the lemma
# determine which bits are set in each fingerprint.
# Copyright 2014, IEEE ENCS Humanoid Robot Project
#===================================================================

from nltk.corpus import wordnet as wn

lemmas = list()
with open('vocab_lemmas.txt', 'r') as f:
	for line in f:
		lemmas.append(line.strip())

N = len(lemmas)

def set_bit(fp, lemmas, lemma):
	if lemma.name().find('.') < 0: # exlude lemmas with a dot in the name, because WN cannot parse them
		try:
			fp.append(lemmas.index(lemma.synset().name + '.' + lemma.name))
		except:
			pass # ignore if lemma not found in the list for our vocab

with open('fingerprints.txt', 'w') as f:
	for i in xrange(N):
		lemma = wn.lemma(lemmas[i])
		fp = list()
		ls = set() # set to collect unique lemmas for the fingerprint

		# 1. Find lemmas in the same synset and add their indexes to the fingerprint
		for l in lemma.synset().lemmas():
			ls.add(l)
		for l in lemma.antonyms():
			ls.add(l)
		for l in lemma.derivationally_related_forms():
			ls.add(l)
		for l in lemma.pertainyms():
			ls.add(l)

		for s in lemma.synset().causes():
			for l in s.lemmas():
				ls.add(l)
		for s in lemma.synset().entailments():
			for l in s.lemmas():
				ls.add(l)
		for synset in lemma.synset().hyponyms():
			for l in synset.lemmas():
				ls.add(l)
			for s in synset.hyponyms(): # traverse hyponyms down two levels
				for l in synset.lemmas():
					ls.add(l)
		for synset in lemma.synset().instance_hyponyms():
			for l in synset.lemmas():
				ls.add(l)
			for s in synset.instance_hyponyms(): # traverse instance hyponyms down two levels
				for l in s.lemmas():
					ls.add(l)
		for s in lemma.synset().member_holonyms():
			for l in s.lemmas():
				ls.add(l)
		for s in lemma.synset().member_meronyms():
			for l in s.lemmas():
				ls.add(l)
		for s in lemma.synset().part_holonyms():
			for l in s.lemmas():
				ls.add(l)
		for s in lemma.synset().part_meronyms():
			for l in s.lemmas():
				ls.add(l)
		for s in lemma.synset().similar_tos():
			for l in s.lemmas():
				ls.add(l)
		for s in lemma.synset().substance_holonyms():
			for l in s.lemmas():
				ls.add(l)
		for s in lemma.synset().substance_meronyms():
			for l in s.lemmas():
				ls.add(l)

		for l in ls:
			set_bit(fp, lemmas, l)
				
		f.write(lemmas[i] + ":" + str(fp) + "\n")
