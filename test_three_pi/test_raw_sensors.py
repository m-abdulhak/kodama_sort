#!/usr/bin/env python
"""
Prints raw IR sensor values.
"""

from three_pi.ThreePi import ThreePi

with ThreePi() as three_pi:
    while True:
        print(three_pi.get_raw_sensors())
