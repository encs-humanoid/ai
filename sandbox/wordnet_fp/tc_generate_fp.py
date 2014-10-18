#!/usr/bin/env python
#===================================================================
# Copyright 2014, IEEE ENCS Humanoid Robot Project
#===================================================================
import unittest

import gen_fingerprints2

class GenerateFingerprintTest(unittest.TestCase):

	def test_generate_fingerprint(self):
		generator = gen_fingerprints2.WordnetFingerprintGenerator(bits_per_lemma=5, fp_length=2048)

		lemma1 = "manoeuver.v.03.operate"
		fp1 = generator.get_fingerprint(lemma1)
		self.assertTrue(len(fp1) > 5, "should have all bits from lemma plus some bits from related lemmas")

		lemma2 = "operation.n.05.operation"	# note: lemma2 is chosen because it is semantically related to lemma1
		fp2 = generator.get_fingerprint(lemma2)
		self.assertTrue(len(fp1 & fp2) >= 5, "should have complete overlap in bits from semantically related concepts")


		lemma3 = "crumb.v.02.crumb"		# note: lemma3 is chosen because it is not semantically related to lemma1
		fp3 = generator.get_fingerprint(lemma3)
		print "fp1 length=", len(fp1)
		print "fp2 length=", len(fp2)
		print "length=", len(fp1 & fp3)
		self.assertTrue(len(fp1 & fp3) < 2)	# expectation is that it is very unlikely to have many bits in common


if __name__ == "__main__":
	unittest.main()
