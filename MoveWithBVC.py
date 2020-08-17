#!/usr/bin/env python
from three_pi.ThreePi import ThreePi
from ThreePiController import ThreePiController
from math import sqrt, atan2, pi
from utils.angles import get_smallest_signed_angular_difference
from execute import execute_with_three_pi
from bvc_navigator import *

class MoveTowardsPointController(ThreePiController):
    def __init__(self, goal_x, goal_y):
        self.goal = {"x": goal_x, "y": goal_y}

        # Initialize BVC Navigator
        self.bvcNav = BvcNavigator(goal_x, goal_y)

        self.goalIsReachedDistance = 20
        self.maxAngleToMoveStraightToGoal = 0.6
        self.robotIsFacingGoalMaxAngle = 0.1

        self.maxForwardSpeed = 0.3
        self.maxAngularSpeed = 0.06

        self.maxMotorSpeed = 1
        self.maxMotorSpeedBack = -1
        self.minMotorSpeed = .3
        self.minMotorSpeedBack = -.3
        

    def getFowrardAndAngularSpeeds(self, distanceToGoal, angleToGoal):
        forwardSpeed, angularSpeed = 0, 0

        print("Distance:", distanceToGoal, "Angle:", angleToGoal)
        
        if (distanceToGoal > self.goalIsReachedDistance):
            # If goal is not reached

            if (abs(angleToGoal) < self.maxAngleToMoveStraightToGoal):
                # If angle to goal is small enough => move in a straight line
                print("Moving Forward")
                forwardSpeed = min(distanceToGoal/100, self.maxForwardSpeed) 
            else:
                # Else, turn in place => No Forward Speed
                print("Not Moving Forward")
                forwardSpeed = 0

            
            if (abs(angleToGoal) > self.robotIsFacingGoalMaxAngle):
                # If robot is not facing goal, turn to goal
                print("Turning")
                angularSpeed = min(angleToGoal / pi, self.maxAngularSpeed)
            else:
                # Else do not turn (move in straight line)
                print("Not Turning")
                angularSpeed = 0

        else:
            forwardSpeed = 0
            angularSpeed = 0

        return forwardSpeed, angularSpeed

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
        print("Pos:", (x, y, theta))

        # Check if final goal reached 
        distanceToFinalGoal = distanceBetween2Points(self.goal, {"x": x, "y": y})
        print(distanceToFinalGoal)

        if (distanceToFinalGoal < self.goalIsReachedDistance ):
            print "reached goal!"
            return True

        # Get the intermediate goal within BVC Cell
        temp_goal = self.bvcNav.findPointInCellClosestToGoal(bvcCell)

        print("Goal:", self.goal, "tempGoal:", temp_goal)

        # Compute the relative position of the goal in polar coordinates (distanceToGoal, angleToGoal)
        dx = temp_goal["x"] - x
        dy = temp_goal["y"] - y

        distanceToGoal = sqrt(dx**2 + dy**2)
        angleToGoal = get_smallest_signed_angular_difference(atan2(dy, dx), theta)

        # Apply constants to convert to forward and angular speed (v, w)
        forwardSpeed, angularSpeed = self.getFowrardAndAngularSpeeds(distanceToGoal, angleToGoal)

        # Convert to right and left speeds and send to robot.
        v_left, v_right = self.getMotorSpeeds(forwardSpeed, angularSpeed)

        print('fwd',forwardSpeed, 'ang', angularSpeed, 'r', v_right, 'l', v_left) 

        self.three_pi.send_speeds(v_left, v_right)

        return False

# Env: Min: 40, 40 Max: 610, 440 
print("Starting")
execute_with_three_pi(MoveTowardsPointController(70, 400))
