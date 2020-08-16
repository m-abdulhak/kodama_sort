#!/usr/bin/env python
"""
A simple controller which gives the rotate a different behaviour depending on
its pose.  Requires local file 'cvss_config.json'.
"""

from three_pi.ThreePi import ThreePi
from cvss.Controller import Controller
from cvss.client_loop import Config, client_loop

with ThreePi() as three_pi:

    class MyController(Controller):
        def update(self, sensor_data):
            if sensor_data.pose.x > 0:
                three_pi.send_speeds(-1, 1)
            elif sensor_data.pose.x < -10:
                three_pi.send_speeds(1, -1)
            else:
                three_pi.send_speeds(0, 0)

    # Read config file
    config = Config('cvss_config.json')
        
    # Create the controller and apply it in a continuous loop.
    my_controller = MyController()
    client_loop(config, my_controller)
