#!/usr/bin/env python
"""
Turn the robot in place, back and forth.
"""

from three_pi.ThreePi import ThreePi
from time import sleep

with ThreePi() as three_pi:
    three_pi.send_speeds(1.0, -1.0)
    sleep(1.0)
    three_pi.send_speeds(0, 0)
    sleep(1.0)
    three_pi.send_speeds(-1.0, 1.0)
    sleep(1.0)
