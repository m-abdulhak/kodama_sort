#!/usr/bin/env python
"""
Checks frequency of updates from server running CVSS.
"""

from cvss.Controller import Controller
from cvss.client_loop import Config, client_loop
from time import time

class MyController(Controller):
    def __init__(self):
        self.reset()

    def reset(self):
        self.calls_to_update = 0
        self.start_time = time()
    
    def update(self, sensor_data):
        self.calls_to_update += 1
        if self.calls_to_update == 10:
            elapsed_time = time() - self.start_time
            freq = 10 / elapsed_time
            print("===================>>>      update rate: {}".format(freq))
            self.reset()

# Read config file
config = Config('cvss_config.json')
    
# Create the controller and apply it in a continuous loop.
my_controller = MyController()
client_loop(config, my_controller)
