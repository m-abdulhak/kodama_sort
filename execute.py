#!/usr/bin/env python
"""
Defines 'execute' which just needs to be passed a suitable controller to
execute.  Requires local file 'cvss_config.json' which is the configuration
file needed for the connection with the server.
"""

from cvss.client_loop import Config, client_loop
from three_pi.ThreePi import ThreePi

def execute(controller):
    # Read config file
    config = Config('cvss_config.json')
        
    # Apply the controller in a continuous loop.
    client_loop(config, controller)

def execute_with_three_pi(controller):
    with ThreePi() as three_pi:
        controller.set_three_pi(three_pi)
        execute(controller)
