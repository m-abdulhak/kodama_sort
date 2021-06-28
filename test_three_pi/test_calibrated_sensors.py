#!/usr/bin/env python
"""
Prints calibrated IR sensor values.
"""

from three_pi.ThreePi import ThreePi

with ThreePi() as three_pi:
    while True:
        print(three_pi.get_calibrated_sensors())
