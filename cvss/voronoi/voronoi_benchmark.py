# coding=utf-8
import datetime
import numpy as np
import matplotlib.pyplot as plt

from scipy.spatial import Voronoi
from shapely.geometry import Polygon

import pyclipper

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

def get_voronoi_cells(points, environment, buffered=False, offset=0.1):
    # Compute Voronoi Diagram
    vor = Voronoi(points)

    performance_step("Computed Voronoi")

    # Reconstruct Infinite Regions To Finite Cells
    regions, vertices = voronoi_finite_polygons_2d(vor,20)

    performance_step("Fixed Infinite Regions")

    cells = []

    # Clip Voronoi Cells To Environment
    for region in regions:
        # Retreive Voronoi Cells Points
        polygon = vertices[region]
        
        # Create Closed Polygon Object From Points
        poly = Polygon(polygon)

        # Clip Voroni Cells To Environment
        poly = poly.intersection(box)

        # Add To Voronoi Cells List
        if(not buffered):
            cells.append(get_voronoi_cell(poly))
        else:
            cells.append(get_buffered_voronoi_cell(poly, offset))

    return cells

def get_voronoi_cell(poly):
    return list(poly.exterior.coords)

def get_buffered_voronoi_cell(poly, offset):
    coordinates = list(poly.exterior.coords) # Array of lat,lng tuples 
    clipper_offset = pyclipper.PyclipperOffset()
    coordinates_scaled = pyclipper.scale_to_clipper(coordinates)

    clipper_offset.AddPath(coordinates_scaled, pyclipper.JT_ROUND, pyclipper.ET_CLOSEDPOLYGON)

    new_coordinates = clipper_offset.Execute(pyclipper.scale_to_clipper(-1 * offset))

    new_coordinates_scaled = pyclipper.scale_from_clipper(new_coordinates)

    new_coordinates_scaled[0].append(new_coordinates_scaled[0][0])
    
    return new_coordinates_scaled[0]

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

def performance_step(msg, last_step=False):
    global last_update_time

    if (not benchmarking):
        return

    now = datetime.datetime.now()
    step_time = now - last_update_time
    last_update_time = now

    print(msg)
    print(step_time)

    if (last_step == True):
        print("Total Time:")
        print(now - begin_time)

##################
# Start of Script#
##################

# Measure Performance
begin_time = datetime.datetime.now()
last_update_time = begin_time
benchmarking = False

# Setup Clipping Region (Environment Size)
min_x = 0
max_x = 1
min_y = 0
max_y = 1
box = Polygon([[min_x, min_y], [min_x, max_y], [max_x, max_y], [max_x, min_y]])

# make up data points
np.random.seed(1234)
points = np.random.rand(5,2)
# points = [[.21,.52],[.54,.756],[.57,.259],[.753,.55]]

cells = get_voronoi_cells(points, box, buffered=True, offset=.01)

performance_step("Clipped Cells", last_step=True)

# for cell in cells:
#     print(cell)

# plot_polygons_points(cells, points)