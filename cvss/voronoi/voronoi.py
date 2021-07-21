# coding=utf-8
import datetime
import traceback
import numpy as np
import matplotlib.pyplot as plt

from scipy.spatial import Voronoi
from shapely.geometry import Point, Polygon, LineString
from shapely.ops import split

import pyclipper

def get_voronoi_cells(points, environment, pose, splittingPoints, buffered=False, offset=10, maxDistanceToEnvCenter=20000000):
    # Compute Voronoi Diagram
    vor = Voronoi(points)

    # Reconstruct Infinite Regions To Finite Cells
    regions, vertices = voronoi_finite_polygons_2d(vor,maxDistanceToEnvCenter)

    cells = []
    first = True

    # Clip Voronoi Cells To Environment
    for index, region in enumerate(regions):
        # Retreive Voronoi Cells Points
        polygon = vertices[region]
        
        # Create Closed Polygon Object From Points
        poly = Polygon(polygon)

        # Clip Voroni Cells To Environment
        poly = poly.intersection(environment)

        # log("Voronoi Cell: ", poly.wkt)
        
        if(poly.is_empty):
            continue

        # Split BVC with closest point from static obstacles
        if(first and splittingPoints and len(splittingPoints) == 2):
            line = LineString(list(map(lambda p: Point(p["x"], p["y"]), splittingPoints)))
            splitCells = split(poly, line)
            curPos = Point(pose.x, pose.y)
            correctCells = filter(lambda poly: curPos.within(poly), splitCells)
            if(len(correctCells) >= 1):
                log("********* VC SPLIT ******", poly.wkt, "-->", correctCells[0].wkt)
                poly = correctCells[0]

        # Add To Voronoi Cells List
        if(not buffered):
            cells.append(get_voronoi_cell(poly))
        else:
            cells.append(get_buffered_voronoi_cell(points[index], poly, offset))
    
    # exit()
    log("====== New BVC =========", cells[0])
    # exit()
    return cells

def voronoi_finite_polygons_2d(vor, radius=None):
    """
    Reconstruct infinite voronoi regions in a 2D diagram to finite
    regions.
    Parameters
    ----------
    vor : Voronoi
        Input diagram
    radius : float, optional
        Distance to 'points at infinity'.
    Returns
    -------
    regions : list of tuples
        Indices of vertices in each revised Voronoi regions.
    vertices : list of tuples
        Coordinates for revised Voronoi vertices. Same as coordinates
        of input vertices, with 'points at infinity' appended to the
        end.
    """

    if vor.points.shape[1] != 2:
        raise ValueError("Requires 2D input")

    new_regions = []
    new_vertices = vor.vertices.tolist()

    center = vor.points.mean(axis=0)
    if radius is None:
        radius = vor.points.ptp().max()*2

    # Construct a map containing all ridges for a given point
    all_ridges = {}
    for (p1, p2), (v1, v2) in zip(vor.ridge_points, vor.ridge_vertices):
        all_ridges.setdefault(p1, []).append((p2, v1, v2))
        all_ridges.setdefault(p2, []).append((p1, v1, v2))

    # Reconstruct infinite regions
    for p1, region in enumerate(vor.point_region):
        vertices = vor.regions[region]

        if all(v >= 0 for v in vertices):
            # finite region
            new_regions.append(vertices)
            continue

        # reconstruct a non-finite region
        ridges = all_ridges[p1]
        new_region = [v for v in vertices if v >= 0]

        for p2, v1, v2 in ridges:
            if v2 < 0:
                v1, v2 = v2, v1
            if v1 >= 0:
                # finite ridge: already in the region
                continue

            # Compute the missing endpoint of an infinite ridge

            t = vor.points[p2] - vor.points[p1] # tangent
            t /= np.linalg.norm(t)
            n = np.array([-t[1], t[0]])  # normal

            midpoint = vor.points[[p1, p2]].mean(axis=0)
            direction = np.sign(np.dot(midpoint - center, n)) * n
            far_point = vor.vertices[v2] + direction * radius

            new_region.append(len(new_vertices))
            new_vertices.append(far_point.tolist())

        # sort region counterclockwise
        vs = np.asarray([new_vertices[v] for v in new_region])
        c = vs.mean(axis=0)
        angles = np.arctan2(vs[:,1] - c[1], vs[:,0] - c[0])
        new_region = np.array(new_region)[np.argsort(angles)]

        # finish
        new_regions.append(new_region.tolist())

    return new_regions, np.asarray(new_vertices)

def get_voronoi_cell(poly):
    return list(poly.exterior.coords)

def get_buffered_voronoi_cell(point, poly, offset):
    try:
        coordinates = list(poly.exterior.coords) # Array of lat,lng tuples 
        clipper_offset = pyclipper.PyclipperOffset()
        coordinates_scaled = pyclipper.scale_to_clipper(coordinates)

        clipper_offset.AddPath(coordinates_scaled, pyclipper.JT_ROUND, pyclipper.ET_CLOSEDPOLYGON)

        new_coordinates = clipper_offset.Execute(pyclipper.scale_to_clipper(-1 * offset))

        new_coordinates_scaled = pyclipper.scale_from_clipper(new_coordinates)
        
        # log(new_coordinates_scaled)

        new_coordinates_scaled[0].append(new_coordinates_scaled[0][0])

        return new_coordinates_scaled[0]
        
    except Exception as e:
        # log("Error Calculating BVC!")
        # log(traceback.format_exc())
        # log("----------------------")
        return [[point, point],[point, point],[point, point]]

def plot_polygons_points(cells, points):
    plt.figure()

    for cell in cells:
        xs, ys = zip(*cell) #create lists of x and y values
        plt.plot(xs,ys) 

    plt.plot(points[:, 0], points[:, 1], 'ko')
    plt.axis('equal')
    plt.xlim(min_x - .2, max_x + .2)
    plt.ylim(min_y- .2, max_y + .2)

    plt.savefig('voro.png')
    plt.show()


logging = False
# logging = True

def log(*msg):
    if(logging):
        print(msg)