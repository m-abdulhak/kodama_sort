#!/usr/bin/env python
"""
Executes a simple controller to test the server running CVSS.
"""

from cvss.Controller import Controller
from execute import execute
from time import time

class MyController(Controller):
    def update(self, sensor_data):
        print("type(sensor_data): ")
        print(type(sensor_data))
        print("sensor_data: ")
        print(sensor_data)

execute(MyController())
