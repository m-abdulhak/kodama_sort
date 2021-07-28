#!/usr/bin/env python
import socket
import time
import struct
import sys
sys.path.insert(0,'..')

import os.path
import cvss_msg_pb2
from Config import Config
from Controller import Controller

import datetime
import numpy as np
from shapely.geometry import Polygon
from shapely.ops import nearest_points
from voronoi.voronoi import get_voronoi_cells
from utils.geometry import shiftPointOfLineSegInDirOfPerpendicularBisector
from utils.geometry import getClosesetPointOfStaticObstacles

from puck import *

def client_loop(config, controller):
    """Using the supplied Config object, connect with a server and launch an
    infinite loop where the controller's update method is called with new sensor
    data from the server.  The controller has the responsibility of interfacing
    with any hardware on the robot."""

    msg_timestamp = -1
    
    # Load configurations
    robotRadius = config.robotRadius
    puckRadius = config.puckRadius
    envPoly = Polygon(config.env)
    envStaticObstacles = map(lambda x: Polygon(x), config.staticObstacles)
    defaultVoronoiPoints = getDefaultVoronoiPoints(config.env)

    # Pass environment definition to controller
    controller.setEnv(envPoly)
    controller.setStaticObstacles(envStaticObstacles)

    # Wait for connection to CVSS
    waitForconnectionToCVSS(config, config.tagID)
    log("Connection To CVSS Established!")
    time.sleep(2)

    # If a goal tag is specified in config, retrieve its CURRENT position and set it as goal 
    configGoalTag = None
    # Not beeded now since goal tags will no nolger be used to specify a goal
    # controller.setGoal(configGoal.x, configGoal.y)
    log("GOAL SET AS:", controller.goal)

    # Initialize Time
    last_timestamp = -1
    last_update = -1

    while True:
        sensorData = getSensorData(config, config.tagID)

        if(not sensorData):
            continue

        # Extract time of sensor reading 
        msg_timestamp = sensorData.timestamp.seconds + sensorData.timestamp.nanos * 1e-9

        if msg_timestamp > last_timestamp: 
            # Setup voronoi points using robot and neighbors positions    
            voronoiPoints = extractVoronoiPoints(sensorData, defaultVoronoiPoints)

            # Get static obstacles
            staticObstacles = getStaticObstacles(sensorData, envStaticObstacles, puckRadius)

            # Get closest point to static obstacles
            closestPointToStaticObs = getClosesetPointOfStaticObstacles(sensorData, staticObstacles)
            controller.setClosestPointToStaticObstacles(closestPointToStaticObs)

            # Get another point on the splitting line be shifting the closest point 
            # in direction of Perpendicular Bisector of curPosition---closestPoint line segment
            point1 = shiftPointOfLineSegInDirOfPerpendicularBisector( \
                closestPointToStaticObs.x, \
                closestPointToStaticObs.y, \
                sensorData.pose.x, \
                sensorData.pose.y, \
                closestPointToStaticObs.x, \
                closestPointToStaticObs.y, \
                1000)
                
            point2 = shiftPointOfLineSegInDirOfPerpendicularBisector( \
                closestPointToStaticObs.x, \
                closestPointToStaticObs.y, \
                closestPointToStaticObs.x, \
                closestPointToStaticObs.y, \
                sensorData.pose.x, \
                sensorData.pose.y, \
                1000)

            log("Split Points:", closestPointToStaticObs.wkt, point2, point1)
            
            # Calculate Voronoi Diagram
            bvcCells = get_voronoi_cells(voronoiPoints, envPoly, sensorData.pose, [point1, point2], buffered=True, offset=robotRadius)

            # log(voronoiPoints, bvcCells)

            # Extract this robot cell from voronoi cells
            bvcCell = bvcCells[0]

            if(controller.update(sensorData, bvcCell)):
                print 'Goal Reached'
                # return
            
            log("Controller Update Done! Cycle Time:", time.time() - last_update)
            last_timestamp = msg_timestamp
            last_update = time.time()

def waitForconnectionToCVSS(config, tagId):
    response = None
    while(response == None or True):            
        response = getSensorData(config, tagId)
        
        if response and response.pose:
            # log("Returning!")
            return
        else:
            log("Could Not Connec To CVSS, Retrying!")
            response = None
        
        time.sleep(1)

def getSensorData(config, tagID):
    server_ip = config.serverIP
    port = config.port

    # Ping host machine running CVSensorSimulator, request sensor data for this robot.
    log("Pinging host")
    requestData = cvss_msg_pb2.RequestData()
    requestData.tag_id = tagID
    # requestData.request_waypoints = False
    
    sensorSimulator = socket.socket()

    try:
        # Connect To Server
        log("Trying to connect to server!")
        sensorSimulator.connect((server_ip, port))

        # Send request for sensor data
        log("Sending request for sensor data!")
        sensorSimulator.send(requestData.SerializeToString())
        sensorData = cvss_msg_pb2.SensorData()
        msg = sensorSimulator.recv(128)
        
        # Close Connection
        log("Closing Connection!")
        sensorSimulator.close()

        # Parse Message
        log("Parsing Message!")
        sensorData.ParseFromString(msg)
        
        # log("+++++++++++++++++++++++++++++++++++++++++++++++++")
        # log("sensorData", sensorData)
        # log("+++++++++++++++++++++++++++++++++++++++++++++++++")

        return sensorData
        
    except socket.error as e:
        print 'Error connecting to CVSS.'
        return False
    except Exception as e:
        print ('Error getting sensor Data: ')
        print str(e)

def extractVoronoiPoints(sensorData, defaultVoronoiPoints):
    # Remove goal tag from neighbors if exists
    neighbors = []
    for neighbor in sensorData.nearby_robot_poses:
        neighbors.append(neighbor)

    log("NEIGHBORS:", neighbors)

    # Get number of Voronoi Points (neighbors + 1, at least 4)  
    neighborsCount = len(neighbors)
    voronoiPointsCount = neighborsCount + 1 if len(neighbors) >= 3 else 4
    
    # Initialize empty voronoi points array
    if(voronoiPointsCount <= 4 ):
        points = np.array(defaultVoronoiPoints)
    else:
        points = np.empty([voronoiPointsCount, 2])

    # Set robot position as the first element
    points[0] = [sensorData.pose.x, sensorData.pose.y]

    # Set neighbors' positions as remaining elements
    for indx, n in enumerate(neighbors):
        points[indx + 1] = [n.x, n.y]
    
    log("VORONOI POINTS:", points)

    return points

def getDefaultVoronoiPoints(env):
    # Since the voronoi diagram library supports minimum of 4 points,
    # a default 4-points list is constructed using environment boundary
    # with default points that lie outside of the environment so that 
    # for any point within the environemnt, its voronoi cell calculated
    # using this list contains the whole environment
    xList = map(lambda point: point[0], env)
    yList = map(lambda point: point[1], env)

    minX = min(xList)
    maxX = max(xList)
    minY = min(yList)
    maxY = max(yList)

    return [[minX-2*maxX, minY-2*maxY], [2*maxX, minY-2*maxY], [2*maxX, 2*maxY], [minX-2*maxX, 2*maxY]]

def getObstaclePolygon(center, radius):
    points = [
        [center["x"] - radius, center["y"] + radius],
        [center["x"] - radius, center["y"]         ],
        [center["x"] - radius, center["y"] - radius],
        [center["x"]         , center["y"] - radius],
        [center["x"] + radius, center["y"] - radius],
        [center["x"] + radius, center["y"]         ],
        [center["x"] + radius, center["y"] + radius],
        [center["x"]         , center["y"] + radius],
    ]
    return Polygon(points)


def getStaticObstacles(sensorData, envStaticObstacles, puckRadius):
    # TODO: Get puck group from sensor data
    puckGroup = 0

    
    puckPositions = list(map(lambda p: {"x": p.x, "y": p.y}, sensorData.nearby_target_positions))
    pucksInGoal = list(filter(lambda p: puckReachedGoal(p, puckGroup), puckPositions))
    puckObstacles = list(map(lambda p: getObstaclePolygon(p, puckRadius), pucksInGoal))
    obstacles = envStaticObstacles + puckObstacles

    return obstacles

logging = True
# logging = True

def log(*msg):
    if(logging):
        print(msg)