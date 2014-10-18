#!/usr/bin/env python
#===================================================================
# Copyright 2014, IEEE ENCS Humanoid Robot Project
#===================================================================
import unittest

import wordnet_fp

class WordnetFingerprintTest(unittest.TestCase):
	def test_overlap(self):
		fp1 = [1, 2, 3, 4]
		fp2 = [2, 3, 4, 5]
		self.assertEqual(wordnet_fp.overlap(fp1, fp2), 3) # has overlap
		fp1 = [1, 2, 3, 4]
		fp2 = [5, 6, 7, 8]
		self.assertEqual(wordnet_fp.overlap(fp1, fp2), 0) # no overlap

	def test_find_matching(self):
		all_fps = list()
		all_fps.append({1, 2, 3})
		all_fps.append({2, 3, 4})
		all_fps.append({3, 4, 5})
		all_fps.append({4, 5, 6})

		# case where exact match is found
		match_list, index_list = wordnet_fp.find_matching({1, 2, 3}, all_fps, 3, 1)
		self.assertEqual(str(match_list), "[set([1, 2, 3])]")
		self.assertEqual(str(index_list), "[0]")

		# case where no match is found
		match_list, index_list = wordnet_fp.find_matching({1, 5, 6}, all_fps, 3, 1)
		self.assertEqual(str(match_list), "[]", "match not found because threshold is too high")
		self.assertEqual(str(index_list), "[]", "match index not found because threshold is too high")

		# case where partial match is found
		match_list, index_list = wordnet_fp.find_matching({1, 5, 6}, all_fps, 2, 1)
		self.assertEqual(str(match_list), "[set([4, 5, 6])]")
		self.assertEqual(str(index_list), "[3]")

		# case where partial match is found with multiple results
		match_list, index_list = wordnet_fp.find_matching({4, 5, 7}, all_fps, 1, 3)
		self.assertEqual(str(match_list), "[set([4, 5, 6]), set([3, 4, 5]), set([2, 3, 4])]")
		self.assertEqual(str(index_list), "[3, 2, 1]")

if __name__ == "__main__":
	unittest.main()
