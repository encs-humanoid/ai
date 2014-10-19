#!/usr/bin/python
#===================================================================
# Test the NuPIC spatial pooler's ability to recognize a set of fingerprints
# Copyright 2014, IEEE ENCS Humanoid Robot Project
#===================================================================

import pickle

from nupic.bindings.algorithms import SpatialPooler as SP

from train_nupic import *

if __name__ == "__main__":
	with open("spatial_pooler.p", "r") as f:
	    trainer = pickle.load(f)

	lemma_sdrs = trainer.lemma_to_sdr.values()

	sdr_to_lemma = dict()

	for lemma_sdr in lemma_sdrs:
		if sdr_to_lemma.has_key(lemma_sdr.last_sdr_key) and lemma_sdr.fp != sdr_to_lemma[lemma_sdr.last_sdr_key].fp:
			print "Duplicate SDR:", lemma_sdr.lemma, "/", sdr_to_lemma[lemma_sdr.last_sdr_key].lemma
		else:
			sdr_to_lemma[lemma_sdr.last_sdr_key] = lemma_sdr

