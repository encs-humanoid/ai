#!/bin/bash

python gen_vocab_lemmas.py
python check_vocab.py
python gen_fingerprints.py
python opt_fingerprints.py
head vocab_lemmas.txt > train_lemmas.txt
date >> runs.out
time unbuffer python train_nupic.py >> runs.out
