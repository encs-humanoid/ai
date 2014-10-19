#!/usr/bin/python
#===================================================================
# Optimize fingerprints by identifying the most meaningful bits
# and removing less meaningful bits.  A bit is deemed meaningful
# according to its discriminatory power:
#	dp_i = 1 - 4 * (S_i/N - 1/2)^2
# where S_i = number of fingerprints with bit i set
#       N   = number of fingerprints
# dp_i has the characteristic that it equals 1 when half the fingerprints
#      have the bit set and half do not, and it decays to zero as the
#      S_i decreases to zero or increases to N
# Copyright 2014, IEEE ENCS Humanoid Robot Project
#===================================================================
from __future__ import division
import numpy as np
import train_nupic

def print_max(bit):
	print "bit=", bit, "count=", bit_counts[bit], "fpbits=", len(lemma_sdrs[bit].fp), "lemma=", lemma_sdrs[bit].lemma

def has_top_bit(lemma):
	for i in lemma.fp:
		if i in top_bits:
			return True
	return False


if __name__ == "__main__":
	# read in all the lemmas and fingerprints
	lemma_sdrs = list()
	train_nupic.process_fingerprints("fingerprints.txt", lambda lemma, fp: lemma_sdrs.append(train_nupic.LemmaSDR(lemma, fp)))

	# determine number of bits in fingerprints
	N = max([max(lemma.fp) for lemma in lemma_sdrs]) + 1  # plus 1 to convert max index to count

	print "number of fingerprints = ", len(lemma_sdrs)
	print "number of bits = ", N

	# for each fingerprint bit, count the number of fingerprints with that bit set
	bit_counts = [0] * N
	for lemma in lemma_sdrs:
		for i in lemma.fp:
			bit_counts[i] += 1

	fp_lengths = [len(lemma.fp) for lemma in lemma_sdrs]

	length_order = np.argsort(fp_lengths)
	for i in xrange(len(lemma_sdrs)):
	    #print_max(bit_count_order[-(i+1)])
	    #print_max(bit_count_order[i])
	    print "fpbits=", fp_lengths[length_order[i]], "lemma=", lemma_sdrs[length_order[i]].lemma

#	top_bits = set(bit_count_order[-5000:])

	# find lemmas with no bit set in the top_bits
#	no_top_bit = list()
#	for lemma in lemma_sdrs:
#		if has_top_bit(lemma):
#			continue
#		no_top_bit.append(lemma)
#
#	print "no_top_bit=", len(no_top_bit)

	# calculate dp_i for each bit i
	#	dp_i = 1 - 4 * (S_i/N - 1/2)^2
	# sort the bits by dp
	# select the top M bits
	# write out optimized fingerprints retaining only the top M bits

	# TODO update/add check script to identify duplicate lemma fingerprints after optimization
