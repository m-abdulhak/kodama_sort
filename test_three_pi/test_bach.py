#!/usr/bin/env python
"""
Tests connection with the 3pi robot by playing a little Bach tune.
"""

from three_pi.ThreePi import ThreePi

with ThreePi() as three_pi:
    three_pi.play_bach()
