#!/usr/bin/env python
"""
Task 1: A simple controller that always steers towards a goal point.  The robot will maintain constant forward speed, but alter its turn rate.
"""

from three_pi.ThreePi import ThreePi
from ThreePiController import ThreePiController
from math import sqrt, atan2, pi
from utils.angles import get_smallest_signed_angular_difference
from execute import execute_with_three_pi

class MoveTowardsPointController(ThreePiController):
    def __init__(self, goal_x, goal_y):
        self.goal_x = goal_x
        self.goal_y = goal_y

        self.maxMotorSpeed = 1
        self.maxMotorSpeedBack = -1
        self.minMotorSpeed = .3
        self.minMotorSpeedBack = -.3

    def getMotorSpeeds(self, forwardSpeed, angularSpeed):
        v_right = forwardSpeed - angularSpeed / 2
        v_left = forwardSpeed + angularSpeed / 2

        if forwardSpeed == 0 and angularSpeed != 0:
            if(v_right > 0 ):
                v_right = max(min(v_right, self.maxMotorSpeed), self.minMotorSpeed) 
            else:
                v_right = max(min(v_right, self.minMotorSpeedBack), self.maxMotorSpeedBack)

            if (v_left > 0):
                v_left = max(min(v_left, self.maxMotorSpeed), self.minMotorSpeed) 
            else:
                v_left = max(min(v_left, self.minMotorSpeedBack), self.maxMotorSpeedBack)
        else:
            v_right = max(min(v_right,1), -1) 
            v_left = max(min(v_left,1), -1)

        return v_left, v_right

    def update(self, sensor_data, bvcCell):
        # Get Robot Position and orientation
        x = sensor_data.pose.x
        y = sensor_data.pose.y
        theta = sensor_data.pose.yaw
        # print((x, y, theta))

        # TODO: Get the intermediate goal within BVC Cell
        goal_x = self.goal_x
        goal_y = self.goal_y

        # Compute the relative position of the goal in polar coordinates (distanceToGoal, angleToGoal)
        dx = goal_x - x
        dy = goal_y - y

        # Calculate the angle to the goal
        distanceToGoal = sqrt(dx**2 + dy**2)
        angleToGoal = get_smallest_signed_angular_difference(atan2(dy, dx), theta)

        
        if (distanceToGoal < 20 ):
            print "reached goal!"
            return True

        # print(x, y, theta, "====>", self.goal_x, self.goal_y)
        # print("Distance:", distanceToGoal, "Angle:", angleToGoal)

        # Apply constants to convert to forward and angular speed (v, w)
        forwardSpeed = (min(distanceToGoal/100, .3) if abs(angleToGoal) < 0.6 else 0) if distanceToGoal > 20 else 0
        angularSpeed = (min(angleToGoal / pi, 0.2) if abs(angleToGoal) > 0.1  else 0) if distanceToGoal > 20 else 0

        # Convert to right and left speeds and send to robot.
        v_left, v_right = self.getMotorSpeeds(forwardSpeed, angularSpeed)

        # print('fwd',forwardSpeed, 'ang', angularSpeed, 'r', v_right, 'l', v_left) 

        self.three_pi.send_speeds(v_left, v_right)

        return False

print("Starting")
execute_with_three_pi(MoveTowardsPointController(100, 100))
