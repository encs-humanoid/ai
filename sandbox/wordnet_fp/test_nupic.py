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

	print trainer
