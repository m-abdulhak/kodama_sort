#!/usr/bin/env python
from __future__ import division
from enum import Enum
from math import sqrt, atan2, pi
from random import random 
from utils.angles import get_smallest_signed_angular_difference
from utils.geometry import *
from execute import execute_with_three_pi

class DeadLockRecovery(Enum):
    none = 0
    simple = 1
    advanced = 2

class BvcNavigator:
    def __init__(self, goal_x, goal_y):
        self.goal = None
        self.tempGoal = self.goal
        self.position = None
        self.bvc = None

        # TODO: get robot radius from config
        self.radius = 35

        # Configurations
        self.goalIsReachedDistance = 20

        # Initialize deadlock detection mechanisms
        self.deadLockDetectionEnabled = True
        self.deadLockDetectionDuration = 5
        self.stuckAtTempGoalDuration = 0
        
        # Initialize deadlock recovery mechanisms
        self.deadLockRecoveryAlgorithm = DeadLockRecovery.advanced
        self.deadLockManeuverInProgress = False
        self.lastDeadlockPosition = None
        self.lastDeadlockAreaRadius = None
        self.lastDeadlockNeighborsCount = None
        self.rightHandPoint = None

        self.remainingDeadlockManeuvers = 0
        self.maxConsecutiveDeadlockManeuvers = 4
        self.maneuverDirection = 0
        
        self.detourPointToOutermostPointRatio = 0.3
        self.detourPointMaxDistance = 3 * self.radius

        self.simpleAlgTempGoalOutOfBvc = 0
        self.simpleAlgTempGoalOutOfBvcMax = 10

        # Deadlock parameters
        robotArea = circleArea(self.radius)
        self.bvcAreaThreshold = robotArea * 3
    
    def setGoal(self, x, y):
        self.goal = {"x": x, "y": y} 
        self.tempGoal = self.goal    

    def update(self, position, cell, sensorData):
        self.position = position
        self.bvc = cell
        self.neighbors = [n for n in sensorData.nearby_robot_poses]

    def setTempGoalInCell(self):
        cell = self.bvc

        # If cell is undefined (shouldn't happen in collision-free configurations) => set localgoal = goal
        if (cell == None or len(cell)<2):
            log("Error, Cell Not Defined!")
            self.tempGoal = self.goal
            return self.tempGoal

        # If the goal is within the Buffered Voronoi cell => set localgoal = goal
        if (cellContains(cell, self.goal)):
            self.tempGoal = self.goal
            log("Goal within bvc!", self.goal, "Temp Goal:", self.tempGoal)
            return self.tempGoal

        # If deadlocked or deadlock is expected or currently recovering from deadlock
        # set local goal according to deadlock recovery policies
        if (self.setLocalGoalByDeadlockRecovery(cell)):
            log("temp goal set by DL recovery!")
            return self.tempGoal

        # Default behavior: set local goal as the point in cell that is closest to the goal
        log("Default Behavior!")
        self.tempGoal = self.findPointInCellClosestToGoal(cell)
        return self.tempGoal

    def findPointInCellClosestToGoal(self, cell, goal = None):
        tempG = None
        minDist = -1
        
        if(goal == None):
            goal = self.goal

        for index, point in enumerate(cell):
            v1 = point
            v2 = cell[nxtCircIndx(index, len(cell))]
            closestPointInLineSeg = closestPointInLineSegToPoint(goal["x"], goal["y"], v1[0], v1[1], v2[0], v2[1])

            distGoalToLineSeg = distanceBetween2Points(goal, closestPointInLineSeg)
            
            if (tempG == None or distGoalToLineSeg < minDist):
                tempG = {"x":closestPointInLineSeg["x"], "y":closestPointInLineSeg["y"]}
                minDist = distGoalToLineSeg
            
        return tempG

    def setLocalGoalByDeadlockRecovery(self, cell):
        # tests whether local goal should be set according to deadlock recovery policies
        # if so => sets local goal accordingly and returns True, else returns False
    
        # If currently recovering from deadlock
        if(self.recoveringFromDeadLock()):
            log("Still Recoving From Deadlock!")

            # if current maneuver's tempGoal is still valid (the current tempGoal has not been reached) 
            # => do not change it, return True
            if(self.deadLockTempGoalStillValid()):
                return True

            # if not, then current maneuver's tempGoal has been reached => end current maneuver
            self.remainingDeadlockManeuvers -= 1
            self.deadLockManeuverInProgress = False
            
            # if another maneuver is needed => initiate it, localGoal is set there so return True
            # This is only for advanced deadlock recovery algorithm, simple always performs single maneuver
            log("========> Remaining DL Maneuvers:", self.remainingDeadlockManeuvers)
            if(self.shouldPerformAnotherManeuver()):
                log("Should perform Another Maneuver!")
                self.initiateDeadlockManeuver(cell)
                return True
            else:
                log("Should Not perform Another Maneuver!")
                self.remainingDeadlockManeuvers = 0
                self.rightHandPoint = None

        # if not recovering from deadlock, test wether currently deadlocked
        elif(self.deadlockRecoveryIsEnabled() and (self.deadLocked() or self.deadLockExpected(self.tempGoal))):
            # if deadlocked => start deadlock recovery, localGoal is set there so return True
            log("Deadlock detected!")
            self.startDeadlockRecovery(cell)
            return True
        

        # If all condition fails => localGoal should not be set according to deadlock recovery policies
        return False
    
    def initiateDeadlockManeuver(self, cell):
        if (self.deadLockRecoveryAlgorithm == DeadLockRecovery.simple):
            # If "DL" is caused by acing a robot, 
            # try to follow right hand rule by moving to the right as much as possible within current bvc
            if (self.rightHandPoint != None):
                log(self.rightHandPoint)
                self.tempGoal = self.findPointInCellClosestToGoal(cell, self.rightHandPoint)
            # Else; deadlock is caused by multiple robots in the way, follow simple DL recovery algorithm
            else:
                self.setTempGoalAccToSimpleDeadlockRec(cell)
        elif (self.deadLockRecoveryAlgorithm == DeadLockRecovery.advanced):
            self.setTempGoalAccToAdvancedDeadlockRec(cell)
        
        log("Finished Initiating Deadlock Maneuver! Temp Goal is: ", self.tempGoal)
        self.deadLockManeuverInProgress = True

    def setTempGoalAccToSimpleDeadlockRec(self, cell):
        # returns temp goal according to simple deadlock recovery algorithm
        # this assumes that deadlocks always occur on a bvc vertex, not on an edges
        # and assumes the sorting (CW or CCW) of the bvc vertices is always the same 
        # TODO: Make sure assumbtions are valid
        log("Setting simple temp goal DL!")
        log(cell, self.tempGoal)

        # for index in range(len(cell) - 1, 0, -1):
        #     point = xyPoint(cell[index])
        #     # log("Checking:", point, "Xs?", point["x"], self.tempGoal["x"], "Ys?", point["y"], self.tempGoal["y"])
        #     if(abs(point["x"] - self.tempGoal["x"]) < 1 and abs(point["y"] - self.tempGoal["y"]) < 1):
        #         # log("TG Found:", point, "Set TG To:", xyPoint(cell[index-1]))
        #         self.tempGoal = xyPoint(cell[index-1])
        #         return

        closestDistanceToGoal = None
        closestIndex = None
        for index in range(len(cell) - 1, 0, -1):
            point = xyPoint(cell[index])
            if(closestDistanceToGoal == None or distanceBetween2Points(point, self.tempGoal) < closestDistanceToGoal):
                closestIndex = index

        diff = 1
        self.tempGoal = xyPoint(cell[closestIndex-diff])
        log("set right {}".format(diff))
        log("temp goal {}".format(self.tempGoal), "Pos: ", self.position, "reached?", self.reached(xyPoint(cell[closestIndex-diff])))
        while(self.reached(xyPoint(cell[closestIndex-diff])) and diff < len(cell)):
            diff += 1
            self.tempGoal = xyPoint(cell[closestIndex-diff])
            log("set right {}".format(diff))
            log("temp goal {}".format(self.tempGoal))

    def setTempGoalAccToAdvancedDeadlockRec(self, cell):
        # returns temp goal according to advanced deadlock recovery algorithm
        # vertices are the vertices of cell that lie on the current maneuver direction
        log("Setting Temp Goal furthest from center!")
        vertecies = self.getVerteciesOnManeuverDir(cell, self.position, self.goal)
        outermostPoint = self.getFurthestVertexFromLineSeg(vertecies, self.position, self.goal)
        distanceToOutermostPoint = self.getDistanceTo(outermostPoint)
        
        log("distance to outermost:{}".format(distanceToOutermostPoint))
        
        if (distanceToOutermostPoint < self.detourPointMaxDistance):
            log("Outermost point less than max, detour point set to outermost")
            self.tempGoal = outermostPoint
        else:
            log("Outermost point larger than max, detour point set per ratio")
            detourRatio = self.detourPointToOutermostPointRatio
            
            distanceWithDefaultRatioLongerThanMax = distanceToOutermostPoint * self.detourPointToOutermostPointRatio > self.detourPointMaxDistance
            if (distanceWithDefaultRatioLongerThanMax) :
                detourRatio = self.detourPointMaxDistance/distanceToOutermostPoint
                log("Distance With Default Ratio ({}) larger than max distance ({}), detour point set with ratio from max distance".format(distanceToOutermostPoint * self.detourPointToOutermostPointRatio, self.detourPointMaxDistance))
            else:
                log("Distance With Default Ratio ({}) Not larger than max distance ({}), detour point with default ratio".format(distanceToOutermostPoint * self.detourPointToOutermostPointRatio, self.detourPointMaxDistance))

            log("Ratio set to:{}".format(detourRatio))
            
            detourPoint = pointOnLineSegmentPerRatio(self.position, outermostPoint, detourRatio)
            log("distance set to:{}".format(self.getDistanceTo(detourPoint)))
            
            self.tempGoal = detourPoint

    def deadlockRecoveryIsEnabled(self):
        return self.deadLockRecoveryAlgorithm != DeadLockRecovery.none
    
    def recoveringFromDeadLock(self):
        return self.deadLockManeuverInProgress or self.remainingDeadlockManeuvers > 0

    def deadLocked(self):
        if(self.reached(self.tempGoal) and not self.reached(self.goal)):
            log("Stuck at temp goal +1!")
            self.stuckAtTempGoalDuration += 1
        else:
            self.stuckAtTempGoalDuration = 0

        ret = self.deadLockDetectionEnabled and self.stuckAtTempGoalDuration > self.deadLockDetectionDuration
        log("Position:{}, Deadlocked? {}".format(self.position, ret))

        return ret

    def deadLockExpected(self, tempGoal):
        if(self.deadLockRecoveryAlgorithm == DeadLockRecovery.simple):
            if(self.facingRobot()):
                log("Facing robot!")
                self.rightHandPoint = shiftPointOfLineSegInDirOfPerpendicularBisector(  \
                    self.position["x"], self.position["y"],                                   \
                    self.position["x"], self.position["y"],                                   \
                    self.goal["x"], self.goal["y"],                                           \
                    self.radius * 3 )
                return True
        
        elif (self.deadLockRecoveryAlgorithm == DeadLockRecovery.advanced):
            neighborGoaldistanceThreshold = self.radius * 3
            neighborNeighbordistanceThreshold = self.radius * 4

            neighborsMeasurements = self.getNeighborsMeasurementsWithin(tempGoal, neighborGoaldistanceThreshold)
            robotsCloseToTempGoal = neighborsMeasurements["robots"]
            maxDistance = neighborsMeasurements["maxDistance"]

            # TODO: Handle case for 1 robot in the way on the edge of environment leading to Deadlock, currently it will be ignored
            if(len(robotsCloseToTempGoal) < 2):
                return False

            for neighborIndex, r in enumerate(robotsCloseToTempGoal):
                # r = robotsCloseToTempGoal[neighborIndex]
                rNext = robotsCloseToTempGoal[nxtCircIndx(neighborIndex, len(robotsCloseToTempGoal))]

                if(distanceBetween2Points({"x": r.x, "y": r.y}, {"x": rNext.x, "y": rNext.y}) < neighborNeighbordistanceThreshold):
                    if( not allPointsAreOnSameSideOfVector([self.goal,self.tempGoal], {"x": r.x, "y": r.y}, {"x": rNext.x, "y": rNext.y})):
                        self.lastDeadlockAreaRadius = maxDistance
                        # console.log("Deadlock Expected With: " + robotsCloseToTempGoal.length + " Robots, with max Distance: " + maxDistance)
                        return True

        return False

    def facingRobot(self):
        log("Checking If Facing Robots!")
        curPos = self.position
        finalGoal = self.goal
        distanceToGoal =  distanceBetween2Points(curPos, finalGoal)
        
        neighborsMeasurements = self.getNeighborsMeasurementsWithin(self.position, self.radius*3)

        robotsCloserToGoal = []
        for r in neighborsMeasurements["robots"]:
            neighborDistToGoal = distanceBetween2Points({"x":r.x, "y":r.x}, finalGoal)
            neighborDistToMe = distanceBetween2Points({"x":r.x, "y":r.x}, curPos)
            if(neighborDistToGoal < distanceToGoal and neighborDistToMe < distanceToGoal):
                robotsCloserToGoal.append(r)
                log("Neighbors Closer To Goal", r)
                log("Distance To Goal: ", neighborDistToGoal, " < ", distanceToGoal, "(My dist To Goal)" )
                log("Distance To Me: ", neighborDistToMe, " < ", distanceToGoal, "(My dist To Goal)" )
        
        facingRobots = []
        for n in robotsCloserToGoal:
            neighborDistanceToLineLessThanRadius = distanceBetweenPointAndLineSeg({"x":n.x, "y":n.x}, curPos, finalGoal) < self.radius
            if(neighborDistanceToLineLessThanRadius):
                facingRobots.append(n)
                log("Facing Robot: ", n )

        return len(facingRobots) > 0

    def startDeadlockRecovery(self, cell):
        self.lastDeadlockPosition = {"x": self.tempGoal["x"], "y":self.tempGoal["y"]}
        
        self.lastDeadlockNeighborsCount = len(self.getNeighborsMeasurementsWithin(self.tempGoal, self.radius*5)["robots"])
        
        self.remainingDeadlockManeuvers = self.maxConsecutiveDeadlockManeuvers
        
        self.maneuverDirection = self.getManeuverDirAccToDLRecoveryAlgo(cell)

        log("Initiating deadlock maneuver!")
        self.initiateDeadlockManeuver(cell)

    def getManeuverDirAccToDLRecoveryAlgo(self, cell):
        if (self.deadLockRecoveryAlgorithm == DeadLockRecovery.simple):
            return random() > 0.3
        elif (self.deadLockRecoveryAlgorithm == DeadLockRecovery.advanced):
            if( random() > .7):
                direc = random() > 0.5
                direcStr = "Right" if direc else "Left"
                log("Detour Direction Set By Random to: {}".format(direcStr))
                return 
            furthestPoint = self.getFurthestVertexFromLineSeg(cell, self.position, self.goal)
            furthestPointDir = pointIsOnRightSideOfVector(furthestPoint["x"], furthestPoint["y"], \
                                                            self.position["x"], self.position["y"], \
                                                            self.goal["x"], self.goal["y"])
                                                            
            direcStr = "Right" if furthestPointDir else "Left"
            log("Furthest Point: {}, direction set to: {}".format(furthestPoint, direcStr))
            return furthestPointDir
        
    def shouldPerformAnotherManeuver(self):
        # return  self.deadLockRecoveryAlgorithm == DeadLockRecovery.advanced and self.remainingDeadlockManeuvers > 0
        return  self.remainingDeadlockManeuvers > 0

    def deadLockTempGoalStillValid(self):
        # Temp goal has not been reached
        tempGoalNotReached = not self.reached(self.tempGoal) 
        log("Pos:", self.position, "Temp Goal:", self.tempGoal, "Reached?", self.reached(self.tempGoal))
        log("Temp goal has been reached?", not tempGoalNotReached)

        # Temp goal is still within BVC
        # TODO: !CHANGED! changed cell from vc to bvc, is it correct?
        currentVCellContainsTempGoal = False

        if(cellContains(self.bvc, self.tempGoal)):
            currentVCellContainsTempGoal = True
        else:
            if(self.simpleAlgTempGoalOutOfBvc < self.simpleAlgTempGoalOutOfBvcMax):
                self.simpleAlgTempGoalOutOfBvc += 1
                self.remainingDeadlockManeuvers += 1

        log("Temp goal within BVC?", currentVCellContainsTempGoal)
        
        # 
        recoveryManeuverHasNotSucceeded = False
        if (self.deadLockRecoveryAlgorithm != DeadLockRecovery.advanced):
            recoveryManeuverHasNotSucceeded = True
        else:
            recoveryManeuverHasNotSucceeded = not (self.deadLockManeuverInProgress and self.neighborsAvoided())
            
        log("Neighbors Avoided?", not recoveryManeuverHasNotSucceeded)

        retVal = tempGoalNotReached and currentVCellContainsTempGoal and recoveryManeuverHasNotSucceeded 
        log("Temp Goal Still Valid?", retVal)

        return retVal
    
    def neighborsAvoided(self):
        robotsMeasurements = self.getNeighborsMeasurementsWithin(self.lastDeadlockPosition, self.lastDeadlockAreaRadius)
        robots = robotsMeasurements["robots"]
        log("Robots:{}".format(robots))
        robotPositions = map(lambda r: {"x": r.x, "y": r.y}, robots)
        
        if(len(robots) == self.lastDeadlockNeighborsCount and self.lastDeadlockNeighborsCount > 1):
            # if(len(robots) < 2):
            #     log("Successfully Recovered From Deadlock! Neighbors Avoided 1")
            #     return True

            neighborsOnSameSide = allPointsAreOnSameSideOfVector(robotPositions, self.position, self.goal)
            neighborsAvoided = minDistanceToLine(robotPositions, self.position, self.goal) > self.radius

            if(neighborsOnSameSide and neighborsAvoided):
                log("Successfully Recovered From Deadlock! Neighbors Avoided 2")
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

    def getVerteciesOnManeuverDir(self, cell, linesSegP1, lineSegP2):
        vertecies = []

        for vertex in cell:
            dir = pointIsOnRightSideOfVector(vertex[0], vertex[1], linesSegP1["x"], linesSegP1["y"], lineSegP2["x"], lineSegP2["y"])
            if( dir == self.maneuverDirection):
                vertecies.append(vertex)

        return vertecies

    def getFurthestVertexFromLineSeg(self, cell, linesSegP1, lineSegP2):
        try:
            bestVertex = cell[0]
            maxDist = None

            for vertex in cell:
                dist = distanceBetweenPointAndLine({"x":vertex[0], "y":vertex[1]}, linesSegP1, lineSegP2)
                if (maxDist == None or dist > maxDist):
                    bestVertex = vertex
                    maxDist = dist

            return {"x":bestVertex[0], "y":bestVertex[1]}
        
        except Exception as e:
            return self.position

    def reached(self, point):
        ret = self.getDistanceTo(point) <= self.goalIsReachedDistance
        return ret

    def getDistanceTo(self, point):
        ret =  distanceBetween2Points(self.position, point)
        return ret

logging = True

def log(*msg):
    if(logging):
        print(msg)

# print(closestPointInLineSegToPoint(0, 0, 0, 1, 1, 0))
# bvcNav = BvcNavigator(0,0)
# print(bvcNav.findPointInCellClosestToGoal([[0,1],[1,1],[1,0]]))
