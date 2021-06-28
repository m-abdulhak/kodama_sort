#!/usr/bin/env python
"""
Tests a couple of little songs.
"""

from time import sleep
from three_pi.ThreePi import ThreePi

with ThreePi() as three_pi:
    three_pi.play_happy_song()
    sleep(2)
    three_pi.play_sad_song()
