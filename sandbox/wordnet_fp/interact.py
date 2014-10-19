#!/usr/bin/python
#===================================================================
# Generate sentences from a grammar given inputs from an external
# source regarding word choices for terminals.
# Copyright 2014, IEEE ENCS Humanoid Robot Project
#===================================================================
# Adapted from http://www.nltk.org/_modules/nltk/parse/generate.html
# Natural Language Toolkit: Generating from a CFG
#
# Copyright (C) 2001-2014 NLTK Project
# Author: Steven Bird <stevenbird1@gmail.com>
#         Peter Ljunglof <peter.ljunglof@heatherleaf.se>
# URL: <http://nltk.org/>
#===================================================================
from __future__ import print_function

import itertools
import sys
from nltk.grammar import Nonterminal

from gen_sentences import *
from train_nupic import *


def generate(grammar, start=None, depth=None, n=None):
	"""
	Generates an iterator of all sentences from a CFG.
	:param grammar: The Grammar used to generate sentences.
	:param start: The Nonterminal from which to start generate sentences.
	:param depth: The maximal depth of the generated tree.
	:param n: The maximum number of sentences to return.
	:return: An iterator of lists of terminal tokens.
	"""
	if not start:
		start = grammar.start()
	if depth is None:
		depth = sys.maxsize

	iter = _generate_all(grammar, [start], depth)

	if n:
		iter = itertools.islice(iter, n)

	return iter

def _generate_all(grammar, items, depth):
    if items:
        for frag1 in _generate_one(grammar, items[0], depth):
            for frag2 in _generate_all(grammar, items[1:], depth):
                yield frag1 + frag2
    else:
        yield []

def _generate_one(grammar, item, depth):
    if depth > 0:
        if isinstance(item, Nonterminal):
            for prod in grammar.productions(lhs=item):
                for frag in _generate_all(grammar, prod.rhs(), depth-1):
                    yield frag
        else:
            yield [item]

demo_grammar = """
  S -> NP VP
  NP -> Det N
  PP -> P NP
  VP -> 'slept' | 'saw' NP | 'walked' PP
  Det -> 'the' | 'a'
  N -> 'man' | 'park' | 'dog'
  P -> 'in' | 'with'
"""

def demo(N=23):
	from nltk.grammar import CFG
	print('Generating the first %d sentences for demo grammar:' % (N,))
	print(demo_grammar)
	grammar = CFG.fromstring(demo_grammar)
	for n, sent in enumerate(generate(grammar, n=N), 1):
		print('%3d. %s' % (n, ' '.join(sent)))

if __name__ == '__main__':
	sp_trainer, tp_trainer = load_htm()
	tp_trainer.is_learning = False
	tp_trainer.compute_inference = True

	print("Enter some text:")
	for line in sys.stdin:
		sentence = nltk.word_tokenize(line.strip())
		fingerprints = generate_fingerprints(sentence)
		for fp in fingerprints:
			tp_trainer.run(fp)

		#sentence_generator = new_sentence_generator()  # TODO could provide context to constructor
		sentence_generator = ['the',
'good.n.01|good.n.02|good.n.03|commodity.n.01|good.a.01|full.s.06|good.a.03|estimable.s.02|beneficial.s.01|good.s.06|good.s.07|adept.s.01|good.s.09|dear.s.02|dependable.s.04|good.s.12|good.s.13|effective.s.04|good.s.15|good.s.16|good.s.17|good.s.18|good.s.19|good.s.20|good.s.21',
'person.n.01.somebody',
'bash.n.02|do.n.02|doctor_of_osteopathy.n.01|make.v.01|perform.v.01|do.v.03|do.v.04|cause.v.01|practice.v.01|suffice.v.01|do.v.08|act.v.02|serve.v.09|do.v.11|dress.v.16|do.v.13',
'well.r.01|well.r.02|well.r.03|well.r.04|well.r.05|well.r.06|well.r.07|well.r.08|well.r.09|well.r.10|well.r.11|well.r.12|well.r.13'
]
		for fragment in sentence_generator:
			lemma_sdrs = tp_trainer.predicted_lemmas
			lemma_names = fragment.split('|')
			if tp_trainer.sp_trainer.lemma_to_sdr.has_key(lemma_names[0]):
				grammar_fp = set()
				for lemma_name in lemma_names:
					if tp_trainer.sp_trainer.lemma_to_sdr.has_key(lemma_name):
						terminal_lemma = tp_trainer.sp_trainer.lemma_to_sdr[lemma_name]
						grammar_fp.update(terminal_lemma.fp)

				htm_fingerprints = [l.fp for l in lemma_sdrs]

				# find the 10 best overlapping lemmas with the terminal lemma from the grammar
				_, indexes = find_matching(grammar_fp, htm_fingerprints, sys.maxsize, 1)
				lemma_sdr = lemma_sdrs[indexes[0]]
				wordnet_lemma = wn.lemma(lemma_sdr.lemma)
				output_sentence.append(wordnet_lemma.name())
				tp_trainer.run(lemma_sdr.fp)
			else:
				output_sentence.append(fragment)




