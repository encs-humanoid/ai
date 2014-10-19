#!/bin/bash

python gen_vocab_lemmas.py melville-moby_dick.txt
# python check_vocab.py
python gen_fingerprints2.py
# python opt_fingerprints.py
head -1000 vocab_lemmas.txt > train_lemmas.txt
date >> runs.out
time unbuffer python train_nupic.py >> runs.out
python test_nupic.py
