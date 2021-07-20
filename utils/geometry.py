from __future__ import division
from math import *
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import datetime

# **************************************************
# *************** Helper Functions *****************
# **************************************************

# Ruler Function
def pointOnLineSegmentPerRatio(startPoint, endPoint, ratio):
  return {"x":(1-ratio)*startPoint["x"]+ratio*endPoint["x"], "y":(1-ratio)*startPoint["y"]+ratio*endPoint["y"]}

def nxtCircIndx(i, length):
    return (i+1) % length

def closestPointInLineToPoint(x, y, x1, y1, x2, y2):
    A = x - x1
    B = y - y1
    C = x2 - x1
    D = y2 - y1

    dot = A * C + B * D
    len_sq = C * C + D * D
    param = -1
    
    if (len_sq != 0):    # in case of 0 length line
        param = dot / len_sq

    xx = x1 + param * C
    yy = y1 + param * D

    return {"x":xx, "y":yy}

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

def distanceBetweenPointAndLine(point, point1LineSeg, point2LineSeg):
    ret =  distanceBetween2Points(point, closestPointInLineToPoint(point["x"], point["y"], point1LineSeg["x"], point1LineSeg["y"], point2LineSeg["x"], point2LineSeg["y"]))
    return ret

def distanceBetweenPointAndLineSeg(point, point1LineSeg, point2LineSeg):
  ret =  distanceBetween2Points(point, closestPointInLineSegToPoint(point["x"], point["y"], point1LineSeg["x"], point1LineSeg["y"], point2LineSeg["x"], point2LineSeg["y"]))
  return ret

def cellContains(cell, point):
    return  len(cell) > 2 and pointIsInsidePolygon(point, cell)

def pointIsInsidePolygon(point, polygon):
    point = Point(point["x"], point["y"])
    poly = Polygon(polygon)
    # return poly.contains(point)
    return point.distance(poly) < 1

def circleArea(radius):
    return radius * radius * pi

def minDistanceToLine(pointsArray, vecStart, vecEnd):
    minDist = None

    for p in pointsArray:
        curDist = distanceBetweenPointAndLine(p, vecStart, vecEnd)

        if (minDist == None):
            minDist = curDist
        else:
            if (curDist < minDist):
                minDist = curDist

    return minDist

def allPointsAreOnSameSideOfVector(pointsArray, vecStart, vecEnd):
    prevSide = None

    for p in pointsArray:
        curSide = pointIsOnRightSideOfVector(p["x"], p["y"], vecStart["x"], vecStart["y"], vecEnd["x"], vecEnd["y"])

        if(prevSide == None):
            prevSide = pointIsOnRightSideOfVector(p["x"], p["y"], vecStart["x"], vecStart["y"], vecEnd["x"], vecEnd["y"])
        else:
            if(curSide != prevSide):
                return False

    return True

def pointIsOnRightSideOfVector(x, y, x1, y1, x2, y2):
    vec1 = {"x":x-x1,"y":-y+y1}
    rot90Vec1 = {"x":-1*vec1["y"], "y":vec1["x"]}
    vec2 = {"x":x2-x1, "y":-y2+y1}

    dot2 = dotProduct(rot90Vec1, vec2) 
    return  dot2>0

def translatePointInDirection(x1, y1, xVec, yVec):
    return {"x":x1+xVec, "y":y1+yVec}

def directionOfPerpendicularBisector(x1, y1, x2, y2, scale):
    length = distanceBetween2Points({"x": x1, "y": y1},{"x": x2, "y": y2})
    if (length > 0):
        return {"x": scale*(y1-y2)/length, "y": scale*(x2-x1)/length}
    else:
        return {"x": scale*(y1-y2), "y": scale*(x2-x1)}

def shiftPointOfLineSegInDirOfPerpendicularBisector(x, y, x1, y1, x2, y2, scale):
    dir = directionOfPerpendicularBisector(x1, y1, x2, y2, scale)
    p1 = translatePointInDirection(x, y, dir["x"], dir["y"])
    return p1

def dotProduct(vec1, vec2):
    return vec1["x"]*vec2["x"] + vec1["y"]*vec2["y"]

def xyPoint(p):
    return {"x":p[0], "y":p[1]}

# l = [1,2,3,4,5]

# for index in range(len(l) - 1, 0, -1):
#     print(index, l[index])

# for index,n in enumerate():
    # print(index, n)

# Test pointIsInsidePolygon:
# print("True?", pointIsInsidePolygon({"x":0.5, "y":0.5}, [[0,0], [0,1], [1,1], [1,0], [0,0]]))
# print("False?", pointIsInsidePolygon({"x":1.5, "y":0.5}, [[0,0], [0,1], [1,1], [1,0], [0,0]]))
# print("False?", pointIsInsidePolygon({"x":0.1, "y":0}, [[0,0], [0,1], [1,1], [1,0], [0,0]]))

# Test circleArea:
# print("3.14..?", circleArea(1))
# print("12.566..?", circleArea(2))


# Test allPointsAreOnSameSideOfVector:
# print("True?", allPointsAreOnSameSideOfVector( [{"x":0, "y":1},{"x":0.5, "y":2},{"x":-2, "y":-1}], {"x":0, "y":0}, {"x":1, "y":1}) )
# print("True?", allPointsAreOnSameSideOfVector( [{"x":0, "y":1},{"x":0.5, "y":2},{"x":-2, "y":-1}], {"x":1, "y":1}, {"x":0, "y":0}) )
# print("False?", allPointsAreOnSameSideOfVector( [{"x":0, "y":1},{"x":0.5, "y":2},{"x":-2, "y":-10}], {"x":0, "y":0}, {"x":1, "y":1}) )
# print("False?", allPointsAreOnSameSideOfVector( [{"x":0, "y":1},{"x":0.5, "y":2},{"x":-2, "y":-10}], {"x":1, "y":1}, {"x":0, "y":0}) )

# print("True?", allPointsAreOnSameSideOfVector( [{"x":-1, "y":1},{"x":-0.5, "y":2},{"x":-2, "y":-1}], {"x":0, "y":0}, {"x":0, "y":1}) )
# print("True?", allPointsAreOnSameSideOfVector( [{"x":-1, "y":1},{"x":-0.5, "y":2},{"x":-2, "y":-1}], {"x":0, "y":1}, {"x":0, "y":0}) )
# print("False?", allPointsAreOnSameSideOfVector( [{"x":0, "y":1},{"x":0.5, "y":2},{"x":-2, "y":-10}], {"x":0, "y":0}, {"x":0, "y":1}) )
# print("False?", allPointsAreOnSameSideOfVector( [{"x":0, "y":1},{"x":0.5, "y":2},{"x":-2, "y":-10}], {"x":0, "y":1}, {"x":0, "y":0}) )

# New for Pucks
def radToDeg(radians):
    return radians * (180 / pi)

# /*
# * Calculates the angle ABC (in radians)
# *
# * A first point, ex: {x: 0, y: 0}
# * C second point
# * B center point
# */
def angleBetweenThreePointsRad(A, B, C):
    AB = sqrt((B["x"] - A["x"]) ** 2 + (B["y"] - A["y"]) ** 2)
    BC = sqrt((B["x"] - C["x"]) ** 2 + (B["y"] - C["y"]) ** 2)
    AC = sqrt((C["x"] - A["x"]) ** 2 + (C["y"] - A["y"]) ** 2)

    if ((2 * BC * AB) != 0):
        return acos((BC * BC + AB * AB - AC * AC) / (2 * BC * AB))
    else:
        return 0

def angleBetweenThreePointsDeg(A, B, C):
  angleRad = angleBetweenThreePointsRad(A, B, C)
  return radToDeg(angleRad)
