#!/usr/bin/env python
"""
Prints 3pi battery voltage.
"""

from three_pi.ThreePi import ThreePi

with ThreePi() as three_pi:
    print(three_pi.get_battery_voltage())
