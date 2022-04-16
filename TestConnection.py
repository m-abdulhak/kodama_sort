#!/usr/bin/env python
import time
from three_pi.ThreePi import ThreePi
from ThreePiController import ThreePiController
from execute import execute_with_three_pi

class TestConnection(ThreePiController):
    def __init__(self):
        pass
        
    def update(self, sensor_data, _):
        elapsed_time = time.time() - self.start_time
        if elapsed_time > self.maxRunTime:
            print("Run-time elapsed.")
            return True

        # Get Robot Position and orientation
        x = sensor_data.pose.x
        y = sensor_data.pose.y
        theta = sensor_data.pose.yaw
        print("x, y, theta: {}, {}, {}".format(x, y, theta))

execute_with_three_pi(TestConnection())
