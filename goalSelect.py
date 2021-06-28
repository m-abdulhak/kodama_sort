#!/usr/bin/env python
import os
from utils.geometry import *
from shapely.geometry import Point, Polygon
from shapely.ops import nearest_points

robotRadius = 35
curGoalTimeSteps = 0
minCurGoalTimeSteps = 100

def updateGoal(controller, robotPosition, bvcCell, sensor_data, env):
  def getGoalFromClosestPointToEnvBounds(closestPoint):
    global curGoalTimeSteps

    print("Getting goal from closest point to env bounds")

    len = distanceBetween2Points(robotPosition, closestPoint)

    translationVec = { \
      "x": ((closestPoint["x"] - robotPosition["x"]) * robotRadius) / (len * 5), \
      "y": ((closestPoint["y"] - robotPosition["y"]) * robotRadius) / (len * 5), \
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

    if(newGoalPoint.distance(env) > 0):
      cP, ___ = nearest_points(env.boundary, newGoalPoint)
      newGoal = {"x": cP.x, "y": cP.y}
    
    print("New Goal:", newGoalPoint.wkt, \
      "Env:", env.wkt, \
      "Dist from Point to new goal:", newGoalPoint.distance(env), \
      "Dist from Env bounds to new goal:", env.exterior.distance(newGoalPoint), \
      "New goal not Valid (dist < robotRadius)?", newGoalPoint.distance(env) > -1 * robotRadius)
    
    if (newGoalPoint.distance(env) > -1 * robotRadius):
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

      newGoalPoint = Point(newGoal["x"], newGoal["y"])

      if(newGoalPoint.distance(env) > 0):
        cP, ___ = nearest_points(env.boundary, newGoalPoint)
        newGoal = {"x": cP.x, "y": cP.y}
      
      print("Modified new goal:", newGoal)
      
    curGoalTimeSteps = minCurGoalTimeSteps

    return newGoal

  def getRandGoal():
    global curGoalTimeSteps

    if (controller.goal != None and \
      curGoalTimeSteps < minCurGoalTimeSteps and \
      not controller.bvcNav.reached(controller.goal)):
      curGoalTimeSteps += 1
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

  # bestPuck = selectBestNearbyPuck()
    # if (bestPuck == null):
  goal = getRandGoal()
    # else:
    #   robot.goal = getGoalFromPuck(bestPuck)
  return goal