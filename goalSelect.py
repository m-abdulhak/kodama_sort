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

    print("Getting goal from closest point to env bounds")

    len = distanceBetween2Points(robotPosition, closestPoint)

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
    
    print("New Goal:", newGoalPoint.wkt, \
      "Env:", env.wkt, \
      "Dist from Point to new goal:", newGoalPoint.distance(env.boundary), \
      "Dist from Env bounds to new goal:", env.exterior.distance(newGoalPoint), \
      "New goal not Valid (dist < robotRadius)?", condGoalNotReachable)

    if (condGoalNotReachable):
      print("Goal Too Close to Edge Detected!")
      print("Old translation vector:", translationVec)
      translationVec["x"] *= -1
      translationVec["y"] *= -1
      print("New translation vector:", translationVec)

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
      
      print("Modified new goal:", newGoal)

    if (not pointIsInsidePolygon(newGoal, env)):
      print("GOAL OUTSIDE ENV DETECTED:")
      print("Old Goal", newGoal)
      newGoal = {
        "x": robotPosition["x"] + 10 * (robotPosition["x"] - closestPoint["x"]),
        "y": robotPosition["y"] + 10 * (robotPosition["y"] - closestPoint["y"])
      }
      print("New Goal", newGoal)
      # exit()
      
    curGoalTimeSteps = 0

    return newGoal

  def getRandGoal():
    global curGoalTimeSteps

    print("Getting goal from Env!")

    if (controller.goal != None and \
      curGoalTimeSteps < minCurGoalTimeSteps and \
      not controller.bvcNav.reached(controller.goal)):
      curGoalTimeSteps += 10
      print("Prev goal")
      return controller.goal

    # environmentBounds = robot.scene.environmentBounds.map(
    #   (point) => ({ x: point[0], y: point[1] }),
    # )

    # pointsCount = environmentBounds.length
    # envRectSides = []

    # for (index = 0 index < environmentBounds.length index += 1) {
    #   nextIndx = (index + 1) % pointsCount
    #   envRectSides.push([environmentBounds[index], environmentBounds[nextIndx]])
    # }

    # closestPointsToSides = envRectSides.map(
    #   (side) => closestPointInLineSegToPoint(
    #     robotPosition["x"],
    #     robotPosition["y"],
    #     side[0]["x"],
    #     side[0]["y"],
    #     side[1]["x"],
    #     side[1]["y"],
    #   ),
    # )

    # closestPoint = closestPointsToSides.reduce((acc, cur) => {
    #   condNotReached = robot.getDistanceTo(cur) > 50 || true
    #   // condNotReached = !robot.reached(cur)
    #   condFirstCorner = acc == null
    #   condClosestThanAcc = condFirstCorner
    #   || robot.getDistanceTo(cur) < robot.getDistanceTo(acc)
    #   if (condNotReached && (condFirstCorner || condClosestThanAcc)) {
    #     return cur
    #   }
    #   return acc
    # }, null)

    # for (index = 0; index < closestPointsToSides.length; index += 1) {
    #   p = closestPointsToSides[index]
    #   if (robot.getDistanceTo(p) < 5) {
    #     closestPoint = closestPointsToSides[(index + 1) % (closestPointsToSides.length)]
    #   }
    # }

    closestPoint, p2 = nearest_points(env.boundary, Point(robotPosition["x"], robotPosition["y"]))
    print("closestPoint", closestPoint.wkt)

    newGoal = getGoalFromClosestPointToEnvBounds({"x": closestPoint.x, "y": closestPoint.y})
    print("Final selected new goal:", newGoal)

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

    print("Puck Ratings:", "Angles:", angleRatings, "distance:", distanceRatings)

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

  print("Available Pucks:", puckPositions)
  bestPuck = selectBestNearbyPuck()
  print("bestPuck:",bestPuck)
  if (bestPuck == None):
    goal = getRandGoal()
    return goal
  else:
    goal = getGoalFromPuck(bestPuck[0], bestPuck[1])
    return goal