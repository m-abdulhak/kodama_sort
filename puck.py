#!/usr/bin/env python
import os
import math
from utils.geometry import *
import pickle
import json

MAP_SCALE = 0.25

def loadPuckGroups(file):
  groups = []
  with open(file, 'rb') as jsonFile:
    loaded_configs = json.load(jsonFile)
    for g in loaded_configs["puckGroups"]:
      groups.append({
        "goal": {"x": g["goal"][0], "y": g["goal"][1]},
        "radius" : g["radius"]
      })
    return groups

def loadMaps(files):
  m = []
  for map_file in files: 
    with open(map_file, 'rb') as handle:
        loaded_goal_map = pickle.load(handle)
        m.append(loaded_goal_map)
  return m

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

map_files = ['map_pickles/0_goal_map.pickle']
maps = loadMaps(map_files)


config_file = "cvss_config.json"
puckGroups = loadPuckGroups(config_file)