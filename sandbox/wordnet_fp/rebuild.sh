#!/bin/bash

python gen_vocab_lemmas.py melville-moby_dick.txt
# python check_vocab.py

python gen_fingerprints2.py
# python opt_fingerprints.py

# take a subset of the data for training the spatial pooler
head -1000 fingerprints.txt > train_lemmas.txt
date >> runs.out
time unbuffer python train_nupic.py sp >> runs.out

# write out some test information about the spatial pooler
python test_nupic.py

# generate word fingerprints for the sentences in a corpus
python gen_sentences.py melville-moby_dick.txt

# take a subset of the sentences for training the temporal pooler
date >> runs.out
time unbuffer python train_nupic.py tp >> runs.out
