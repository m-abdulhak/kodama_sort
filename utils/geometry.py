from __future__ import division
from math import *
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import datetime

# **************************************************
# *************** Helper Functions *****************
# **************************************************

def nxtCircIndx(i, length):
    return (i+1) % length

def closestPointInLineSegToPoint(x, y, x1, y1, x2, y2):
    A = x - x1
    B = y - y1
    C = x2 - x1
    D = y2 - y1

    dot = A * C + B * D
    len_sq = C * C + D * D
    param = -1

    # in case of 0 length line
    if (len_sq != 0):
        param = dot / len_sq

    xx = None
    yy =  None

    if (param < 0):
        xx = x1
        yy = y1
    elif (param > 1):
        xx = x2
        yy = y2
    else:
        xx = x1 + param * C
        yy = y1 + param * D

    return {"x":xx, "y":yy}

def distanceBetween2Points(pos1, pos2):
    ret =  sqrt( (pos1["x"] - pos2["x"]) * (pos1["x"] - pos2["x"]) + (pos1["y"] - pos2["y"]) * (pos1["y"] - pos2["y"]))
    return ret

def cellContains(cell, point):
    return  len(cell) > 2 and pointIsInsidePolygon(point, cell)

def pointIsInsidePolygon(point, polygon):
    point = Point(point["x"], point["y"])
    polygon = Polygon(polygon)
    return polygon.contains(point)

# print(pointIsInsidePolygon("True?", {"x":0.5, "y":0.5}, [[0,0], [0,1], [1,1], [1,0], [0,0]]))
# print(pointIsInsidePolygon("False?", {"x":1.5, "y":0.5}, [[0,0], [0,1], [1,1], [1,0], [0,0]]))
# print(pointIsInsidePolygon("False?", {"x":0.1, "y":0}, [[0,0], [0,1], [1,1], [1,0], [0,0]]))