#!/usr/bin/env python
#===================================================================
# Copyright 2014, IEEE ENCS Humanoid Robot Project
#===================================================================
import unittest

import wordnet_fp

class WordnetFingerprintTest(unittest.TestCase):
	def testOverlap(self):
		fp1 = [1, 2, 3, 4]
		fp2 = [2, 3, 4, 5]
		self.assertEqual(wordnet_fp.overlap(fp1, fp2), 3) # has overlap
		fp1 = [1, 2, 3, 4]
		fp2 = [5, 6, 7, 8]
		self.assertEqual(wordnet_fp.overlap(fp1, fp2), 0) # no overlap


if __name__ == "__main__":
	unittest.main()
