#!/usr/bin/env python
#===================================================================
# For NLTK Chat, pairings of input patterns with possible responses.
# We'll flesh these out over time...
# Copyright (C) 2001-2014 NLTK Project
#===================================================================

pairs = (
  (r'Hello(.*)',
  ( "Hello... How are you liking the conference?",
    "Hi... how are you doing?",
    "Hello, do I know you?")),

  (r'(.*)',
  ( "My hearing is weak. What I think you said was %1",
    "Why do you say that?"))
)
