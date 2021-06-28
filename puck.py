#!/usr/bin/env python
import os
import math
from utils.geometry import *

puckGroups = [
  {
    "goal": {"x": 20, "y": 20},
    "radius": 50
  },
  {
    "goal": {"x": 400, "y": 400},
    "radius": 50
  }
]

def puckDistanceTo(puckPosition, otherPosition):
  distance = distanceBetween2Points(puckPosition, otherPosition)
  return distance


def puckReachedGoal(puckPosition, group):
  goal = puckGroups[group]["goal"]
  radius = puckGroups[group]["radius"]
  distanceToGoal = distanceBetween2Points(puckPosition, goal)
  print("Puck Reached Goal?", \
    "puck", puckPosition, \
    "goal", goal, \
    "radius", radius, \
    "distance", distanceToGoal, \
    "reached?", distanceToGoal <= radius)
  return distanceToGoal <= radius

def puckGoalAngle(robotPosition, puckPosition, group):
  goal = puckGroups[group]["goal"]
  angle = angleBetweenThreePointsDeg(robotPosition, puckPosition, goal)
  return angle

def puckNormalizedAngle(robotPosition, puckPosition, group):
  angle = puckGoalAngle(robotPosition, puckPosition, group)
  normalizedAngle = abs(angle - 180)
  return normalizedAngle

def getPuckManeuverGoal(robotPosition, puckPosition, group):
  goal = puckGroups[group]["goal"]
  return closestPointInLineToPoint( \
      robotPosition["x"], \
      robotPosition["y"], \
      puckPosition["x"], \
      puckPosition["y"], \
      goal["x"], \
      goal["y"])
