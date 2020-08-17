#!/usr/bin/env python
from __future__ import division
from enum import Enum
from math import sqrt, atan2, pi
from utils.angles import get_smallest_signed_angular_difference
from utils.geometry import *
from execute import execute_with_three_pi

class DeadLockRecovery(Enum):
    none = 0
    simple = 1
    advanced = 2

class BvcNavigator:
    def __init__(self, goal_x, goal_y):
        self.goal = {"x": goal_x, "y": goal_y}
        self.tempGoal = self.goal
        self.position = None
        self.bvc = None

        # Configurations
        self.goalIsReachedDistance = 20

        # Initialize deadlock detection mechanisms
        self.deadLockDetectionEnabled = True
        self.deadLockDetectionDuration = 5
        self.stuckAtTempGoalDuration = 0
        
        # Initialize deadlock recovery mechanisms
        self.deadLockRecoveryAlgorithm = DeadLockRecovery.none
        self.deadLockManeuverInProgress = False
        self.lastDeadlockPosition = None
        self.lastDeadlockAreaRadius = None
        self.lastDeadlockNeighborsCount = None
        self.rightHandPoint = None

        self.remainingDeadlockManeuvers = 0
        self.maxConsecutiveDeadlockManeuvers = 6
        self.maneuverDirection = 0

        # Deadlock parameters
        # TODO: get robot radius from config
        self.radius = 35
        robotArea = circleArea(self.radius)
        self.bvcAreaThreshold = robotArea * 3

    def update(self, position, cell, sensorData):
        self.position = position
        self.bvc = cell
        self.neighbors = sensorData.neighbors

    def setTempGoalInCell(self):
        cell = self.bvc

        # If cell is undefined (shouldn't happen in collision-free configurations) => set localgoal = goal
        if(cell == None or len(cell)<2):
            self.tempGoal = self.goal
            return self.tempGoal

        # If the goal is within the Buffered Voronoi cell => set localgoal = goal
        if (cellContains(cell, self.goal)):
            self.tempGoal = self.goal
            return self.tempGoal

        # If deadlocked or deadlock is expected or currently recovering from deadlock
        # set local goal according to deadlock recovery policies
        # if (self.setLocalGoalByDeadlockRecovery(cell)):
        #     return self.tempGoal

        # Default behavior: set local goal as the point in cell that is closest to the goal
        self.tempGoal = self.findPointInCellClosestToGoal(cell)
        return self.tempGoal

    def findPointInCellClosestToGoal(self, cell):
        tempG = None
        minDist = -1

        for index, point in enumerate(cell):
            v1 = point
            v2 = cell[nxtCircIndx(index, len(cell))]
            closestPointInLineSeg = closestPointInLineSegToPoint(self.goal["x"], self.goal["y"], v1[0], v1[1], v2[0], v2[1])

            distGoalToLineSeg = distanceBetween2Points(self.goal, closestPointInLineSeg)
            
            if (tempG == None or distGoalToLineSeg < minDist):
                tempG = {"x":closestPointInLineSeg["x"], "y":closestPointInLineSeg["y"]}
                minDist = distGoalToLineSeg
            
        return tempG

    def setLocalGoalByDeadlockRecovery(self, cell):
        # tests whether local goal should be set according to deadlock recovery policies
        # if so => sets local goal accordingly and returns True, else returns False
    
        # If currently recovering from deadlock
        if(self.recoveringFromDeadLock()):
            # if current maneuver's tempGoal is still valid (the current tempGoal has not been reached) => do not change it, return True
            if(self.deadLockTempGoalStillValid()):
                return True
            
        #     # if not, then current maneuver's tempGoal has been reached => end current maneuver
        #     self.remainingDeadlockManeuvers -= 1
        #     self.deadLockManeuverInProgress = False
            
        #     # if another maneuver is needed => initiate it, localGoal is set there so return True
        #     if(self.shouldPerformAnotherManeuver()):
        #         self.initiateDeadlockManeuver(cell)
        #         return True
        #     else:
        #         self.remainingDeadlockManeuvers = 0
        #         self.rightHandPoint = None

        # # if not recovering from deadlock, test wether currently deadlocked
        # elif(self.deadlockRecoveryIsEnabled() and (self.deadLocked() or self.deadLockExpected(self.tempGoal))):
        #     # if deadlocked => start deadlock recovery, localGoal is set there so return True
        #     self.startDeadlockRecovery(cell)
        #     return True
        

        # If all condition fails => localGoal should not be set according to deadlock recovery policies
        return False

    def deadlockRecoveryIsEnabled(self):
        return self.deadLockRecoveryAlgorithm != DeadLockRecovery.none
    
    def recoveringFromDeadLock(self):
        return self.deadLockManeuverInProgress or self.remainingDeadlockManeuvers > 0
        
    def deadLockTempGoalStillValid(self):
            # Temp goal has not been reached
            tempGoalNotReached = not self.reached(self.tempGoal) 

            # Temp goal is still within BVC
            # TODO: !CHANGED! changed cell from vc to bvc, is it correct?
            currentVCellContainsTempGoal = cellContains(self.bvc, self.tempGoal)
            
            # 
            recoveryManeuverHasNotSucceeded = False
            if (self.deadLockRecoveryAlgorithm != DeadLockRecovery.advanced):
                recoveryManeuverHasNotSucceeded = True
            else:
                recoveryManeuverHasNotSucceeded = not (self.deadLockManeuverInProgress and self.neighborsAvoided())

            return tempGoalNotReached and currentVCellContainsTempGoal and recoveryManeuverHasNotSucceeded 
    
    def neighborsAvoided(self):
        robotsMeasurements = self.getNeighborsMeasurementsWithin(self.lastDeadlockPosition, self.lastDeadlockAreaRadius)
        robots = robotsMeasurements["robots"]
        robotPositions = map(lambda r: {"x": r.x, "y": r.y}, robots)
        
        if(self.lastDeadlockNeighborsCount > 1):
            if(len(robots) < 2):
                # print("Successfully Recovered From Deadlock! 1")
                return True

            neighborsOnSameSide = allPointsAreOnSameSideOfVector(robotPositions, self.position, self.goal)
            neighborsAvoided = minDistanceToLine(robotPositions, self.position, self.goal) > self.radius

            if(neighborsOnSameSide and neighborsAvoided):
                # print("Successfully Recovered From Deadlock! 2")
                return True

        return False

    def getNeighborsMeasurementsWithin(self, point, distance):
        closeRobots = []
        maxDist = 0

        for r in self.neighbors:
            curDist = distanceBetween2Points({"x": r.x, "y": r.y}, point)

            if(curDist<=distance):
                closeRobots.append(r)
                maxDist = curDist if curDist > maxDist else maxDist

        return {"robots": closeRobots, "maxDistance": maxDist}

    def reached(self, point):
        ret = self.getDistanceTo(point) <= self.goalIsReachedDistance
        return ret

    def getDistanceTo(self, point):
        ret =  distanceBetween2Points(self.position, point)
        return ret

# print(closestPointInLineSegToPoint(0, 0, 0, 1, 1, 0))
# bvcNav = BvcNavigator(0,0)
# print(bvcNav.findPointInCellClosestToGoal([[0,1],[1,1],[1,0]]))
