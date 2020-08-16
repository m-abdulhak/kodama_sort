#!/usr/bin/env python

from three_pi.ThreePi import ThreePi

with ThreePi() as three_pi:
    three_pi.autocalibrate()

