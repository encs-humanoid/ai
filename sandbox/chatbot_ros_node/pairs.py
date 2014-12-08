#!/usr/bin/env python
#===================================================================
# For NLTK Chat, pairings of input patterns with possible responses.
# We'll flesh these out over time...
# Copyright 2014, IEEE ENCS Humanoid Robot Project
#===================================================================

pairs = (
  (r'Hello(.*)',
  ( "Hello... How are you liking the conference?",
    "Hi... how are you doing?",
    "Hello, do I know you?")),

  (r'You are (.*)',
  ( "I know you are %1, but what am I?",
    "What makes you say I am %1?")),

  (r'I am pleased (.*)',
  ( "I am so pleased that you are pleased (.*)")),

  (r'robot',
  ( "Yes, let's talk about robots. I like robots. What about you?",
    "Ahh, robots are one of my favorite topics.")),

  (r'I am (.*)',
  ( "Hello %1. I am glad to meet you!",
    "Are you sure you are %1?")),

  (r'(.*)',
  ( "My hearing is weak. What I think you said was %1",
    "Why do you say that?",
    "Gosh! I would never have thought of that!")),
)
