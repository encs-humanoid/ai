#!/usr/bin/env python
#===================================================================
# This file contains functions for manipulating fingerprints.
# 
# overlap - computes the number of common bits in two fingerprints
#
# Copyright 2014, IEEE ENCS Humanoid Robot Project
#===================================================================

import numpy as np


def overlap(fp1, fp2):
	"""
	Compute the overlap between two fingerprints.
	The fingerprints are sets of numbers, where the numbers
	correspond to the on-bits in the fingerprint.
	"""
	return len(fp1 & fp2)


def find_matching(test_fp, all_fps, threshold, max_to_return):
	"""
	Find the closest matching fingerprints given a query fingerprint and a match threshold
	"""
	overlaps = list()
	for fp in all_fps:
		overlaps.append(overlap(test_fp, fp))
	order = np.argsort(overlaps)
	result_fingerprints = list()
	result_indexes = list()
	for i in xrange(max_to_return):
		index = order[-(i+1)]
		if overlaps[index] >= threshold:
			result_fingerprints.append(all_fps[index])
			result_indexes.append(index)
	return result_fingerprints, result_indexes


