#!/usr/bin/python
#===================================================================
# This codelet generates semantic fingerprints given a file of
# WordNet lemmas.  The order of lemmas in the file defines the bit
# position in the fingerprint.  WordNet associations to the lemma
# determine which bits are set in each fingerprint.
# Copyright 2014, IEEE ENCS Humanoid Robot Project
#===================================================================

from nltk.corpus import wordnet as wn

def set_bit(fp, lemmas, lemma):
	if lemma.name().find('.') < 0: # exclude lemmas with a dot in the name, because WN cannot parse them
		lookup = lemma.synset().name() + '.' + lemma.name()
		if lemmas.has_key(lookup):
			fp.add(lemmas[lookup])
		else:
			pass # ignore if lemma not found in the list for our vocab
	# else TODO look up synset instead and search it for a matching lemma name to handle the dot issue


def add_hypernyms_recursive(synset, ls):
	list = synset.hypernyms()
	for item in list:
		for lemma in item.lemmas():
			ls.add(lemma)
		add_hypernyms_recursive(item, ls)


def read_lemmas(filename):
	lemmas = dict()
	lemmas_list = list()
	with open(filename, 'r') as f:
		i = 0
		for line in f:
			lemma = line.strip()
			lemmas[lemma] = i
			i += 1
			lemmas_list.append(lemma)
	return lemmas, lemmas_list


def get_related_lemmas(lemma):
	ls = set() # set to collect unique lemmas for the fingerprint

	# 1. Find lemmas in the same synset and add their indexes to the fingerprint
	# Note: the bit for the lemma itself is set in this process
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
	add_hypernyms_recursive(lemma.synset(), ls)

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

	return ls


def generate_fp(lemma, lemmas):
	fp = set()
	ls = get_related_lemmas(lemma, lemmas)

	for l in ls:
		set_bit(fp, lemmas, l)

	return fp


if __name__ == "__main__":
	lemmas, lemmas_list = read_lemmas('vocab_lemmas.txt')

	N = len(lemmas)

	with open('fingerprints.txt', 'w') as f:
		for lookup in lemmas_list:
			lemma = wn.lemma(lookup)
			print lemmas[lookup], lemma
			fp = generate_fp(lemma, lemmas)
			f.write(lookup + ":" + str(fp) + "\n")
