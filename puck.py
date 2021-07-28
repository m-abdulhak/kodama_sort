#!/usr/bin/env python
import os
import math
from utils.geometry import *
import pickle

MAP_SCALE = 0.25

map_files = ['map_pickles/0_goal_map.pickle']
maps = []

for map_file in map_files: 
  with open(map_file, 'rb') as handle:
      loaded_goal_map = pickle.load(handle)
      maps.append(loaded_goal_map)
      # print(loaded_goal_map[0][0])
      # exit()

puckGroups = [
  {
    "goal": {"x": 310, "y": 250},
    "radius": 80
  },
  {
    "goal": {"x": 400, "y": 400},
    "radius": 80
  }
]

def getPuckGoal(puckPosition, group):
  x = int(puckPosition["x"] * MAP_SCALE)
  y = int(puckPosition["y"] * MAP_SCALE)
  goal = maps[group][y][x]
  return {"x": goal[0] / MAP_SCALE, "y": goal[1] / MAP_SCALE}

def puckDistanceTo(puckPosition, otherPosition):
  distance = distanceBetween2Points(puckPosition, otherPosition)
  return distance


def puckReachedGoal(puckPosition, group):
  goal = puckGroups[group]["goal"]
  radius = puckGroups[group]["radius"]
  distanceToGoal = distanceBetween2Points(puckPosition, goal)
  log("Puck Reached Goal?", \
    "puck", puckPosition, \
    "goal", goal, \
    "radius", radius, \
    "distance", distanceToGoal, \
    "reached?", distanceToGoal <= radius)
  return distanceToGoal <= radius

def puckGoalAngle(robotPosition, puckPosition, group):
  goal = getPuckGoal(puckPosition, group)
  angle = angleBetweenThreePointsDeg(robotPosition, puckPosition, goal)
  return angle

def puckNormalizedAngle(robotPosition, puckPosition, group):
  angle = puckGoalAngle(robotPosition, puckPosition, group)
  normalizedAngle = abs(angle - 180)
  return normalizedAngle

def getPuckManeuverGoal(robotPosition, puckPosition, group):
  goal = getPuckGoal(puckPosition, group)
  return closestPointInLineToPoint( \
      robotPosition["x"], \
      robotPosition["y"], \
      puckPosition["x"], \
      puckPosition["y"], \
      goal["x"], \
      goal["y"])

logging = True
# logging = True

def log(*msg):
    if(logging):
        print(msg)