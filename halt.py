#!/usr/bin/env python

from three_pi.ThreePi import ThreePi
from time import sleep

with ThreePi() as three_pi:
    three_pi.send_speeds(0, 0)
    sleep(1.0)