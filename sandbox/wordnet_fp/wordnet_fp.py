#!/usr/bin/env python
#===================================================================
# This file contains functions for manipulating fingerprints.
# 
# overlap - computes the number of common bits in two fingerprints
#
# Copyright 2014, IEEE ENCS Humanoid Robot Project
#===================================================================


def overlap(fp1, fp2):
	"""
	Compute the overlap between two fingerprints.
	The fingerprints are lists of numbers, where the numbers
	correspond to the on-bits in the fingerprint.
	"""
	return len(set(fp1) & set(fp2))

