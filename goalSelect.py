#!/usr/bin/env python
import os
from utils.geometry import *
from shapely.geometry import Point, Polygon
from shapely.ops import nearest_points
from puck import *
import random

robotRadius = 35
curGoalTimeSteps = 0
minCurGoalTimeSteps = 50

PUCK_DETECTION_RADIUS = robotRadius * 5
BEST_PUCK_ANGLE_THRESH = 20
ACCEPTABLE_PUCK_ANGLE_THRESH = 70

def updateGoal(controller, robotPosition, bvcCell, sensor_data, env):
  puckPositions = list(map(lambda p: {"x": p.x, "y": p.y}, sensor_data.nearby_target_positions))
  puckPoistions = list(filter(lambda p: distanceBetween2Points(robotPosition, p) < PUCK_DETECTION_RADIUS, puckPositions))

  def getGoalFromClosestPointToEnvBounds(closestPoint):
    global curGoalTimeSteps

    log("Getting goal from closest point to env bounds")

    len = distanceBetween2Points(robotPosition, closestPoint)

    if (len == 0):
      len = 0.01

    translationVec = { \
      "x": ((closestPoint["x"] - robotPosition["x"]) * robotRadius) / (len * 10), \
      "y": ((closestPoint["y"] - robotPosition["y"]) * robotRadius) / (len * 10), \
    }

    midPoint = translatePointInDirection( \
      robotPosition["x"], \
      robotPosition["y"], \
      translationVec["x"], \
      translationVec["y"])

    delta = robotRadius * 2
    newGoal = midPoint

    newGoal = shiftPointOfLineSegInDirOfPerpendicularBisector( \
      midPoint["x"], \
      midPoint["y"], \
      robotPosition["x"], \
      robotPosition["y"], \
      closestPoint["x"], \
      closestPoint["y"], \
      delta)

    newGoalPoint = Point(newGoal["x"], newGoal["y"])
    condGoalNotReachable = newGoalPoint.distance(env.boundary) < robotRadius
    
    log("New Goal:", newGoalPoint.wkt, \
      "Env:", env.wkt, \
      "Dist from Point to new goal:", newGoalPoint.distance(env.boundary), \
      "Dist from Env bounds to new goal:", env.exterior.distance(newGoalPoint), \
      "New goal not Valid (dist < robotRadius)?", condGoalNotReachable)

    if (condGoalNotReachable):
      log("Goal Too Close to Edge Detected!")
      log("Old translation vector:", translationVec)
      translationVec["x"] *= -1
      translationVec["y"] *= -1
      log("New translation vector:", translationVec)

      midPoint = translatePointInDirection( \
        robotPosition["x"], \
        robotPosition["y"], \
        translationVec["x"], \
        translationVec["y"])

      newGoal = midPoint

      newGoal = shiftPointOfLineSegInDirOfPerpendicularBisector(
        midPoint["x"],
        midPoint["y"],
        robotPosition["x"],
        robotPosition["y"],
        closestPoint["x"],
        closestPoint["y"],
        delta)
      
      log("Modified new goal:", newGoal)

    if (not pointIsInsidePolygon(newGoal, env)):
      log("GOAL OUTSIDE ENV DETECTED:")
      log("Old Goal", newGoal)
      newGoal = {
        "x": robotPosition["x"] + 10 * (robotPosition["x"] - closestPoint["x"]),
        "y": robotPosition["y"] + 10 * (robotPosition["y"] - closestPoint["y"])
      }
      log("New Goal", newGoal)
      # exit()
      
    curGoalTimeSteps = 0

    return newGoal

  def getRandGoal():
    global curGoalTimeSteps

    log("Getting goal from Env!")

    if (controller.goal != None and \
      curGoalTimeSteps < minCurGoalTimeSteps and \
      not controller.bvcNav.reached(controller.goal)):
      curGoalTimeSteps += 10
      log("Prev goal")
      return controller.goal

    curPosPoint = Point(robotPosition["x"], robotPosition["y"])

    closestPointToEnvBounds, p2 = nearest_points(env.boundary, curPosPoint)    
    closestPoint = closestPointToEnvBounds

    # IMP: orbit static obstacles as well as env
    # closestPointToStaticObstacles = getClosesetPointOfStaticObstacles(sensor_data, controller.staticObstacles)
    # if(curPosPoint.distance(closestPointToStaticObstacles) < curPosPoint.distance(closestPointToEnvBounds)):
    #   closestPoint = closestPointToStaticObstacles
    #   print("Static Obstacles closer!", closestPointToStaticObstacles.wkt, closestPointToEnvBounds.wkt)
      # exit()

    log("closestPoint", closestPointToEnvBounds.wkt)
    newGoal = getGoalFromClosestPointToEnvBounds({"x": closestPoint.x, "y": closestPoint.y})
    log("Final selected new goal:", newGoal)

    return newGoal

  def getGoalFromPuck(puckPosition, normalizedAngle):
    if (normalizedAngle < BEST_PUCK_ANGLE_THRESH):
      return puckPosition

    closestPointInLine = getPuckManeuverGoal(robotPosition, puckPosition, 0)

    if (normalizedAngle < ACCEPTABLE_PUCK_ANGLE_THRESH):
      return closestPointInLine

    return getRandGoal()

  def selectBestNearbyPuck():
    global curGoalTimeSteps

    if (controller.goal != None and \
      curGoalTimeSteps < minCurGoalTimeSteps and \
      not controller.bvcNav.reached(controller.goal)):
      curGoalTimeSteps += 1
      # return robot.bestPuck
      return None

    angleRatings = []
    distanceRatings = []

    def puckIsValid(position, group):
      if (not puckReachedGoal(position, group)):
        # g = getGoalFromPuck(p)
        # condInRobotVorCell = pointIsInsidePolygon(p.position, robot.BVC);
        # condReachableInEnv = robot.pointIsReachableInEnvBounds(g);
        # return condInRobotVorCell and condReachableInEnv and condReachableOutOfStaticObs;
        return True
      return False

    validPucks = list(filter(lambda p: puckIsValid(p, 0), puckPositions))

    for puck in validPucks:
      ang = puckNormalizedAngle(robotPosition, puck, 0)
      if(puckDistanceTo(puck, robotPosition) < PUCK_DETECTION_RADIUS):
        if(ang < ACCEPTABLE_PUCK_ANGLE_THRESH):
          angleRatings.append([puck, ang])
        distanceRatings.append([puck, puckDistanceTo(puck, robotPosition)])

    angleRatings.sort(key=lambda x: x[1])
    distanceRatings.sort(key=lambda x: x[1])

    log("Puck Ratings:", "Angles:", angleRatings, "distance:", distanceRatings)

    angleRatsExist = len(angleRatings) > 0
    distRatsExist = len(distanceRatings) > 0

    bestPuck = None

    if (angleRatsExist):
      bestPuck = angleRatings[0]
    # elif (distRatsExist and distanceRatings[0][1]  < robotRadius * 5):
    #   bestPuck = distanceRatings[0]
    # elif (angleRatsExist):
    #   bestPuck = angleRatings[0]

    if (bestPuck != None):
      curGoalTimeSteps = 0
    
    return bestPuck

  def limitGoalWithinEnvOutsideStaicObstacles(goal):
    # while (not pointIsInsidePolygon(goal, env)):
    #   print("Goal Not inside Env, limiting goal to env bounds!", goal)
    #   goal = {"x": (goal["x"] + robotPosition["x"]) / 2, "y": (goal["y"] + robotPosition["y"]) / 2}
    #   print("New goal:", goal)
    newGoal = {"x": goal["x"], "y": goal["y"]}
    
    goalDistanceToStaticObstacles = list(map(lambda obs: Point(newGoal["x"], newGoal["y"]).distance(obs), controller.staticObstacles))
    goalInsideObs = list(filter(lambda dist: dist < robotRadius, goalDistanceToStaticObstacles))
    while (len(goalInsideObs) > 0):
      log("Goal inside static obs, moving goal further!", newGoal)
      newGoal = {"x": 2 * newGoal["x"] - robotPosition["x"], "y": 2 * newGoal["y"] - robotPosition["y"]}
      log("New goal:", newGoal)
      goalDistanceToStaticObstacles = list(map(lambda obs: Point(newGoal["x"], newGoal["y"]).distance(obs), controller.staticObstacles))
      goalInsideObs = list(filter(lambda dist: dist < robotRadius, goalDistanceToStaticObstacles))
  
    return newGoal

  log("Available Pucks:", puckPositions)
  bestPuck = selectBestNearbyPuck()
  log("bestPuck:",bestPuck)
  if (bestPuck == None):
    goal = getRandGoal()
    goal = limitGoalWithinEnvOutsideStaicObstacles(goal)
    return goal
  else:
    goal = getGoalFromPuck(bestPuck[0], bestPuck[1])
    goal = limitGoalWithinEnvOutsideStaicObstacles(goal)
    return goal

logging = False
# logging = True

def log(*msg):
    if(logging):
        print(msg)