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

# Handling Getting Stuck
lastPosition = None
durationAtCurPosition = 0
stuck = False
avoidingStuckDuration = 0
MIN_STUCK_MANEUVER_DURATION = 20
SAME_POSITION_DISTANCE_THRESHOLD = robotRadius / 50
STUCK_DURATION_THRESHOLD = 20

# Control Strategy Angles Thresholds
ANGLE_OPTIMAL_THRESHOLD = 15
ANGLE_ACCEPTABLE_THRESHOLD = 75

## Loading env orbit map
MAP_SCALE = 0.25
MAP_FILE = 'map_pickles/env_orbit_map.pickle'
ENV_MAP = None
with open(MAP_FILE, 'rb') as handle:
    ENV_MAP = pickle.load(handle)

def updateGoal(controller, robotPosition, bvcCell, sensor_data, env, oldGoal):
  puckPositions = list(map(lambda p: {"x": p.x, "y": p.y}, sensor_data.nearby_target_positions))
  puckPoistions = list(filter(lambda p: distanceBetween2Points(robotPosition, p) < PUCK_DETECTION_RADIUS, puckPositions))
  global lastPosition
  global durationAtCurPosition
  global stuck
  global avoidingStuckDuration

  def getGoalFromClosestPointToEnvBounds():
    global curGoalTimeSteps
    log("Getting goal from Orbit Map")

    x = int(robotPosition["x"] * MAP_SCALE)
    y = int(robotPosition["y"] * MAP_SCALE)
    goal = ENV_MAP[y][x]
    curGoalTimeSteps = 0
    return {"x": goal[0] / MAP_SCALE, "y": goal[1] / MAP_SCALE}

  def getGoalFromOrbit():
    global curGoalTimeSteps

    log("Getting goal from Env!")

    if (controller.goal != None and \
      curGoalTimeSteps < minCurGoalTimeSteps and \
      not controller.bvcNav.reached(controller.goal)):
      curGoalTimeSteps += 10
      log("Minimum goal seek time not reached, using prev goal.")
      return controller.goal

    curPosPoint = Point(robotPosition["x"], robotPosition["y"])

    newGoal = getGoalFromClosestPointToEnvBounds()
    log("Final selected new goal:", newGoal)

    return newGoal

  def getGoalFromStuckManeuver():
    envOrbitGoal = getGoalFromOrbit()
    envOrbitGoal = limitGoalWithinEnvOutsideStaicObstacles(envOrbitGoal)
    print ("================ Old Env Orbit Goal: ", envOrbitGoal)

    vecToEnvOrbitGoal = {
      "x": envOrbitGoal["x"] - robotPosition["x"],
      "y": envOrbitGoal["y"] - robotPosition["y"],
    }
    rotatedEnvOribtGoal = {
      "x": vecToEnvOrbitGoal["y"],
      "y": -1 * vecToEnvOrbitGoal["x"],
    }

    newG = {
      "x": robotPosition["x"] + rotatedEnvOribtGoal["x"],
      "y": robotPosition["y"] + rotatedEnvOribtGoal["y"],
    }

    print ("================ New Maneuver Goal: ", newG)
    # exit()
    return newG

  def getGoalFromPuck(puckPosition, normalizedAngle):
    if (normalizedAngle < BEST_PUCK_ANGLE_THRESH):
      return puckPosition

    closestPointInLine = getPuckManeuverGoal(robotPosition, puckPosition, 0)

    if (normalizedAngle < ACCEPTABLE_PUCK_ANGLE_THRESH):
      return closestPointInLine

    return getGoalFromOrbit()

  def selectBestNearbyPuck():
    global curGoalTimeSteps

    if (controller.goal != None and \
      curGoalTimeSteps < minCurGoalTimeSteps and \
      not controller.bvcNav.reached(controller.goal)):
      curGoalTimeSteps += 1
      # return robot.bestPuck
      log("LOCAL GOAL STILL VALID, Not Selecting a new puck!")
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
    log("Valid Pucks:", validPucks)

    for puck in validPucks:
      ang = puckNormalizedAngle(robotPosition, puck, 0)
      log("Angle to local goal:", puck, ang)
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

    if (bestPuck != None):
      curGoalTimeSteps = 0
    
    return bestPuck

  def limitGoalWithinEnvOutsideStaicObstacles(goal):
    # while (not pointIsInsidePolygon(goal, env)):
    #   print("Goal Not inside Env, limiting goal to env bounds!", goal)
    #   goal = {"x": (goal["x"] + robotPosition["x"]) / 2, "y": (goal["y"] + robotPosition["y"]) / 2}
    #   print("New goal:", goal)
    # exit()
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

  # If robot was stuck and is still recovering, do not change robot goal
  if (stuck and avoidingStuckDuration <= MIN_STUCK_MANEUVER_DURATION):
    avoidingStuckDuration += 1
    print("STILL MANEUVERING FRIM GETTING STUCK, No GOAL CHANGE!", avoidingStuckDuration, " / ", MIN_STUCK_MANEUVER_DURATION)
    return oldGoal
  
  # Else, consider maneuver over, reset counters
  stuck = False
  avoidingStuckDuration = 0

  # Calc distance to last recorded position
  distToLastPos = distanceBetween2Points(robotPosition, lastPosition) if lastPosition else None

  # If robot is close enough to be considered at same position
  if (distToLastPos != None and distToLastPos <= SAME_POSITION_DISTANCE_THRESHOLD):
    # Do not change recorded position, increment stuck timer by 1
    print("Position Not Changed For:", durationAtCurPosition)
    durationAtCurPosition += 1

  # If stuck timer, reaches threshold to be considered stuck
  if (durationAtCurPosition >= STUCK_DURATION_THRESHOLD):
    # Reset stuck timer, set state to stuck, start stuck maneuver timer and start maneuver
    durationAtCurPosition = 0
    stuck = True
    avoidingStuckDuration = 0
    print("Stuck Detected! Starting Stuck Maneuver!")
    goal = getGoalFromStuckManeuver()
    goal = limitGoalWithinEnvOutsideStaicObstacles(goal)
    return goal

  # Update last position and continuer normal operations
  lastPosition = { "x" : robotPosition["x"], "y" : robotPosition["y"] }
  log("Available Pucks:", puckPositions)
  bestPuck = selectBestNearbyPuck()
  log("bestPuck:",bestPuck)
  # exit()
  if (bestPuck == None):
    goal = getGoalFromOrbit()
    goal = limitGoalWithinEnvOutsideStaicObstacles(goal)
    return goal
  else:
    goal = getGoalFromPuck(bestPuck[0], bestPuck[1])
    goal = limitGoalWithinEnvOutsideStaicObstacles(goal)
    return goal

logging = True
# logging = True

def log(*msg):
    if(logging):
        print(msg)