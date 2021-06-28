#!/usr/bin/env python
"""
Testing whether the robot will stop when we enter an infinite loop while moving and the user hits CTRL-C.
"""

from three_pi.ThreePi import ThreePi
from time import sleep

with ThreePi() as three_pi:
    while True:
        three_pi.send_speeds(1.0, -1.0)
        print("Hit CTRL-C and the robot should stop.  This is because the")
        print("__exit__ method in ThreePi will be invoked which stops the 3pi.")
