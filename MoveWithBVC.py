#!/usr/bin/env python
import os
import time
from three_pi.ThreePi import ThreePi
from ThreePiController import ThreePiController
from math import sqrt, atan2, pi
from utils.angles import get_smallest_signed_angular_difference
from execute import execute_with_three_pi
from bvc_navigator import *
from goalSelect import updateGoal

class MoveTowardsPointController(ThreePiController):
    def __init__(self, goal_x, goal_y):
        # No default goal
        self.goal = None

        # Initialize BVC Navigator
        self.bvcNav = BvcNavigator(goal_x, goal_y)

        self.maxAngleToMoveStraightToGoal = 0.6
        #self.robotIsFacingGoalMaxAngle = 0.1
        self.robotIsFacingGoalMaxAngle = 0.4

        self.maxForwardSpeed = 0.3
        self.maxAngularSpeed = 0.06

        self.maxMotorSpeed = 1
        self.maxMotorSpeedBack = -1
        #self.minMotorSpeed = .5
        #self.minMotorSpeedBack = -.5
        self.minMotorSpeed = .35
        self.minMotorSpeedBack = - self.minMotorSpeed

        # Maximum run-time in seconds
        self.maxRunTime = 1 * 60

        # logging
        self.logSize = 0
        now = time.time()
        self.logFileLocation = 'logs/log-{}-{}.txt'.format(str(self.bvcNav.deadLockRecoveryAlgorithm), now)
        try:
            self.logFile = open(self.logFileLocation, 'w+')
            log("Loggin to : {}".format(self.logFileLocation))
        except Exception as e:
            log("Error! Cannot Open Log File!", e)

        log("Recording start time")
        self.start_time = time.time()
        
    def setGoal(self, x, y):
        log("Setting Goal To:", x, y)
        self.goal = {"x": x, "y": y}
        self.bvcNav.setGoal(x, y)  

    def setEnv(self, poly):
        self.env = poly
        log("Environment Boundary:", self.env.wkt)

    def setStaticObstacles(self, polys):
        self.staticObstacles = polys

    def setClosestPointToStaticObstacles(self, point):
        self.closestPointToStaticObstacles = point

    def getFowrardAndAngularSpeeds(self, distanceToGoal, angleToGoal):
        forwardSpeed, angularSpeed = 0, 0

        # log("Distance:", distanceToGoal, "Angle:", angleToGoal)
        
        # If goal is not reached
        if (not self.bvcNav.reached(self.goal)):

            # If angle to goal is small enough => move in a straight line
            if (abs(angleToGoal) < self.maxAngleToMoveStraightToGoal):
                log("Moving Forward")
                forwardSpeed = min(distanceToGoal/10, self.maxForwardSpeed) 
            # Else, turn in place => No Forward Speed
            else:
                log("Not Moving Forward")
                forwardSpeed = 0

            # If robot is not facing goal, turn to goal
            if (abs(angleToGoal) > self.robotIsFacingGoalMaxAngle):
                log("Turning")
                #angularSpeed = min(angleToGoal / pi, self.maxAngularSpeed)
                angularSpeed = 0.1 * angleToGoal / pi
            # Else do not turn (move in straight line)
            else:
                log("Not Turning")
                angularSpeed = 0
                
        else:
            log("Stopped")
            forwardSpeed = 0
            angularSpeed = 0

        return forwardSpeed, angularSpeed

    def getMotorSpeeds(self, forwardSpeed, angularSpeed):
        v_right = forwardSpeed - angularSpeed / 2
        v_left = forwardSpeed + angularSpeed / 2
        log("Wheels Speeds (L-R):", v_left,v_right)

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
        elapsed_time = time.time() - self.start_time
        if elapsed_time > self.maxRunTime:
            print("Run-time elapsed.")
            return True

        # Get Robot Position and orientation
        x = sensor_data.pose.x
        y = sensor_data.pose.y
        theta = sensor_data.pose.yaw
        # log("Pos:", (x, y, theta))
        # log(self.goal)

        self.logStateToFile(sensor_data, bvcCell)

        self.bvcNav.update({"x": x, "y": y}, bvcCell, sensor_data)
        log("Cur Position:", self.bvcNav.position)

        # Get new goal
        log("*** Calculating New Goal ***")
        newGoal = updateGoal(self, {"x": x, "y": y}, bvcCell, sensor_data, self.env)
        log("*** New Goal ***", newGoal)
        self.setGoal(newGoal["x"], newGoal["y"])

        # Check if final goal reached 
        # if (self.bvcNav.reached(self.goal)):
            # print "reached goal!"
            # return True

        # Get the intermediate goal within BVC Cell
        temp_goal = self.bvcNav.setTempGoalInCell()

        # log("Goal:", self.goal, "tempGoal:", temp_goal)

        # Compute the relative position of the goal in polar coordinates (distanceToGoal, angleToGoal)
        dx = temp_goal["x"] - x
        dy = temp_goal["y"] - y

        distanceToGoal = sqrt(dx**2 + dy**2)
        angleToGoal = get_smallest_signed_angular_difference(atan2(dy, dx), theta)

        # Apply constants to convert to forward and angular speed (v, w)
        forwardSpeed, angularSpeed = self.getFowrardAndAngularSpeeds(distanceToGoal, angleToGoal)

        # Convert to right and left speeds and send to robot.
        v_left, v_right = self.getMotorSpeeds(forwardSpeed, angularSpeed)
        v_left, v_right = v_left, v_right 

        log('fwd',forwardSpeed, 'ang', angularSpeed, 'r', v_right, 'l', v_left) 

        self.three_pi.send_speeds(v_left, v_right)
        log("Sent Wheel Speeds")
        
        return False

    def logStateToFile(self, sensorData, bvcCell):
        try:
            if self.logFile and self.logFile.closed:
                self.logFile = open(self.logFileLocation, 'w+')
                
            self.logFile.write(self.getLogStateToFileRow(sensorData, bvcCell))
            
            self.logSize = self.logSize+1
            if self.logSize > 1000:
                self.logFile.flush()
                self.logSize = 0
                
        except Exception as e:
            log("Error! Write to Log File!", e)
    
    def getLogStateToFileRow(self, sensorData, bvcCell):
        now = time.time()
        return str(now) + " , " + str(sensorData).replace("\n", "") + " , " + str(bvcCell) + " , " + str(self.goal) + "\n"


logging = True
# logging = True

def log(*msg):
    if(logging):
        print(msg)

# Env: Min: 40, 40 Max: 610, 440 
log("Starting")
execute_with_three_pi(MoveTowardsPointController(150, 190))
