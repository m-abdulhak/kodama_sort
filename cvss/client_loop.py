#!/usr/bin/env python
import socket
import time
import struct
import sys
import os.path
import cvss_msg_pb2
from Config import Config
from Controller import Controller

import datetime
import numpy as np
from shapely.geometry import Polygon
from voronoi.voronoi import get_voronoi_cells

def client_loop(config, controller):
    """Using the supplied Config object, connect with a server and launch an
    infinite loop where the controller's update method is called with new sensor
    data from the server.  The controller has the responsibility of interfacing
    with any hardware on the robot."""

    msg_timestamp = -1
    
    # Load configurations
    robotRadius = config.robotRadius
    envPoly = Polygon(config.env)
    defaultVoronoiPoints = getDefaultVoronoiPoints(config.env)
    
    # Initialize Time
    last_timestamp = -1
    last_update = -1

    while True:
        sensorData = getSensorData(config)

        if(not sensorData):
            continue

        # Extract time of sensor reading 
        msg_timestamp = sensorData.timestamp.seconds + sensorData.timestamp.nanos * 1e-9

        if msg_timestamp > last_timestamp: 
            # Setup voronoi points using robot and neighbors positions    
            voronoiPoints = extractVoronoiPoints(sensorData, defaultVoronoiPoints)

            # Calculate Voronoi Diagram
            bvcCells = get_voronoi_cells(voronoiPoints, envPoly, buffered=True, offset=robotRadius)

            # print(cells)

            # Extract this robot cell from voronoi cells
            bvcCell = bvcCells[0]

            if(controller.update(sensorData, bvcCell)):
                print 'Goal Reached'
                return
            
            last_timestamp = msg_timestamp
            last_update = time.time()

def getSensorData(config):
    server_ip = config.serverIP
    port = config.port
    tagID = config.tagID

    # Ping host machine running CVSensorSimulator, request sensor data for this robot.
    # print("Pinging host")
    requestData = cvss_msg_pb2.RequestData()
    requestData.tag_id = tagID
    requestData.request_waypoints = False
    
    sensorSimulator = socket.socket()

    try:
        # Connect To Server
        sensorSimulator.connect((server_ip, port))

        # Send request for sensor data
        sensorSimulator.send(requestData.SerializeToString())
        sensorData = cvss_msg_pb2.SensorData()
        msg = sensorSimulator.recv(128)
        
        # Close Connection
        sensorSimulator.close()

        # Parse Message
        sensorData.ParseFromString(msg)

        return sensorData
        
    except socket.error as e:
        print 'Error connecting to CVSS.'
        return False

def extractVoronoiPoints(sensorData, defaultVoronoiPoints):
    # Get number of Voronoi Points (neighbors + 1, at least 4)  
    neighborsCount = len(sensorData.neighbors)
    voronoiPointsCount = neighborsCount + 1 if len(sensorData.neighbors) >= 3 else 4
    
    # Initialize empty voronoi points array
    if(voronoiPointsCount <= 4 ):
        points = np.array(defaultVoronoiPoints)
    else:
        points = np.empty([voronoiPointsCount, 2])

    # Set robot position as the first element
    points[0] = [sensorData.pose.x, sensorData.pose.y]

    # Set neighbors' positions as remaining elements
    for indx, n in enumerate(sensorData.neighbors):
        points[indx + 1] = [n.x, n.y]
    
    return points

def getDefaultVoronoiPoints(env):
    # Since the voronoi diagram library supports minimum of 4 points
    # a default 4-points list is constructed using environment size
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