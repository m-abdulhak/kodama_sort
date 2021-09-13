import json
import numpy as np
import pylab as plt
import skfmm
import pickle
from math import sqrt
from shapely.geometry import Point, Polygon

def pointIsInsidePolygon(point, poly):
    try:
        return point.within(poly)
    except Exception as e:
        return False

def readConfig(filePath):
    with open(filePath) as f:
        return json.load(f)

MIN_DIRECTION = 30
ENV_ORBIT_STEEPNESS = 14
INCLUDE_OBSTACLES = True
SAVE_FIGURES = True
SHOW_FIGURES = False
MAP_SCALE = 0.25
ROBOT_RADIUS = 30
CLEARANCE = ROBOT_RADIUS * MAP_SCALE

print("Robot Radius:", ROBOT_RADIUS,"Map Scale:", MAP_SCALE, "Clearance:", CLEARANCE)

###########################################
####        Load Configurations        ####
###########################################

config = readConfig('cvss_config.json')
env = config["env"]
obs = config["staticObstacles"]

envPoly = Polygon([p[0] * MAP_SCALE, p[1] * MAP_SCALE] for p in env)
envXMax = int(max(x[0] for x in env) * MAP_SCALE)
envYMax = int(max(x[1] for x in env) * MAP_SCALE)
print("Env:", env, " Max X:", envXMax,"Max Y:", envYMax)

staticObstaclesPolygons = [
    Polygon([p[0] * MAP_SCALE, p[1] * MAP_SCALE] for p in o) for o in obs
]

print("Obs:", {ob.wkt for ob in staticObstaclesPolygons})

###########################################
#### Generate boundary and speed masks ####
###########################################

X, Y = np.meshgrid(np.linspace(0, envXMax, envXMax+1), np.linspace(0, envYMax, envYMax+1))
map_border_mask = -1*np.ones_like(X)
speed = 1*np.ones_like(X)

for x in range(envXMax):
    for y in range(envYMax):
        curPoint = Point(x,y)
        if(curPoint.within(envPoly) and envPoly.exterior.distance(curPoint) > CLEARANCE):
            map_border_mask[y][x] = 1

        if(INCLUDE_OBSTACLES):
            for o in staticObstaclesPolygons:
                if(curPoint.distance(o) < CLEARANCE):
                    map_border_mask[y][x] = -1

###########################################
####    Generate Distance Transform    ####
###########################################

t = skfmm.travel_time(map_border_mask, speed, 1)
t_filled = t

gradients_y, gradients_x = np.gradient(t_filled)
vec_to_border_x = -10 * gradients_x
vec_to_border_y = -10 * gradients_y

###########################################
####    Generate Environment Orbits    ####
###########################################
orbit_vec_x = ENV_ORBIT_STEEPNESS * -1 *  vec_to_border_y + vec_to_border_x
orbit_vec_y = ENV_ORBIT_STEEPNESS * vec_to_border_x + vec_to_border_y

def vecLength(x, y):
    return sqrt(x*x + y*y)

for x in range(envXMax):
    for y in range(envYMax):
        curPoint = Point(x,y)
        if (map_border_mask[y][x] != -1):
            orbit_vec_x[y][x] = ENV_ORBIT_STEEPNESS * vec_to_border_y[y][x] + vec_to_border_x[y][x]
            orbit_vec_y[y][x] = ENV_ORBIT_STEEPNESS * -1 * vec_to_border_x[y][x] + vec_to_border_y[y][x]

        if (orbit_vec_x[y][x] >= 0 and orbit_vec_x[y][x] < MIN_DIRECTION):
            orbit_vec_x[y][x]  = MIN_DIRECTION
        if (orbit_vec_x[y][x] <= 0 and orbit_vec_x[y][x] > -1 * MIN_DIRECTION):
            orbit_vec_x[y][x]  = -1 * MIN_DIRECTION
        if (orbit_vec_y[y][x] >= 0 and orbit_vec_y[y][x] < MIN_DIRECTION):
            orbit_vec_y[y][x]  = MIN_DIRECTION
        if (orbit_vec_y[y][x] <= 0 and orbit_vec_y[y][x] > -1 * MIN_DIRECTION):
            orbit_vec_y[y][x]  = -1 * MIN_DIRECTION


from scipy.ndimage.filters import gaussian_filter

orbit_vec_x = gaussian_filter(orbit_vec_x, sigma=2)
orbit_vec_y = gaussian_filter(orbit_vec_y, sigma=2)

countAll = 0
sumAll = 0
minAll = None

for x in range(envXMax):
    for y in range(envYMax):
        curPoint = Point(x,y)
        vLength = vecLength(orbit_vec_x[y][x], orbit_vec_y[y][x])

        if (vLength < MIN_DIRECTION / 2 and orbit_vec_x[y][x] >= 0 and orbit_vec_x[y][x] < MIN_DIRECTION):
            orbit_vec_x[y][x]  = MIN_DIRECTION / 2
        if (vLength < MIN_DIRECTION / 2 and orbit_vec_x[y][x] <= 0 and orbit_vec_x[y][x] > -1 * MIN_DIRECTION):
            orbit_vec_x[y][x]  = -1 * MIN_DIRECTION / 2
        if (vLength < MIN_DIRECTION / 2 and orbit_vec_y[y][x] >= 0 and orbit_vec_y[y][x] < MIN_DIRECTION):
            orbit_vec_y[y][x]  = MIN_DIRECTION / 2
        if (vLength < MIN_DIRECTION / 2 and orbit_vec_y[y][x] <= 0 and orbit_vec_y[y][x] > -1 * MIN_DIRECTION):
            orbit_vec_y[y][x]  = -1 * MIN_DIRECTION / 2

        vLength = vecLength(orbit_vec_x[y][x], orbit_vec_y[y][x])

        countAll += 1
        sumAll += vLength
        if minAll is None or vLength < minAll:
            minAll = vLength

print("Count", countAll)
print("Mean", sumAll / countAll)
print("Min", minAll)

orbit_goal_points_x = orbit_vec_x + X
orbit_goal_points_y = orbit_vec_y + Y

orbit_map = []

for y, row in enumerate(orbit_goal_points_x):
    orbit_map.append([])
    for x, val in enumerate(row):
        orbit_map[y].append([orbit_goal_points_x[y][x], orbit_goal_points_y[y][x]])

###############################
##########  PICKLES  ##########
###############################

file_name = 'map_pickles/env_orbit_map.pickle'

with open(file_name, 'wb') as handle:
    print('Dumping map to: ' + file_name)
    pickle.dump(orbit_map, handle, protocol=2)

###############################
##########  FIGURES  ##########
###############################

GRAPH_STEP = 4

if (SAVE_FIGURES or SHOW_FIGURES):
    plt.title('Orbit Border')
    plt.contour(X, Y, map_border_mask, [0], linewidths=(3), colors='red')
    if (SAVE_FIGURES):
        plt.savefig('images/env_orbit_1_border_mask.png', dpi=1200)
    if (SHOW_FIGURES):
        plt.show()
    else:
        plt.clf()

    plt.title('Static Obstacles (Speed Map)')
    plt.contour(X, Y, speed, [0], linewidths=(1), colors='Blue')
    plt.contourf(X, Y, speed, 1)
    plt.colorbar()
    if (SAVE_FIGURES):
        plt.savefig('images/env_orbit_2_speed_map.png', dpi=1200)
    if (SHOW_FIGURES):
        plt.show()
    else:
        plt.clf()

    plt.title('Distance To Border')
    plt.contour(X, Y, map_border_mask,[0], linewidths=(3), colors='red')
    cp = plt.contourf(X, Y, t, 50)
    plt.colorbar()
    if (SAVE_FIGURES):
        plt.savefig('images/env_orbit_3_distance_to_border.png', dpi=1200)
    if (SHOW_FIGURES):
        plt.show()
    else:
        plt.clf()

    plt.title('Gradient of Distance To Border')
    cp = plt.quiver(X[::GRAPH_STEP, ::GRAPH_STEP], Y[::GRAPH_STEP, ::GRAPH_STEP], gradients_x[::GRAPH_STEP, ::GRAPH_STEP], gradients_y[::GRAPH_STEP, ::GRAPH_STEP])
    if (SAVE_FIGURES):
        plt.savefig('images/env_orbit_4_gradients_of_distance_to_border.png', dpi=1200)
    if (SHOW_FIGURES):
        plt.show()
    else:
        plt.clf()

    plt.title('Directions To Border')
    cp = plt.quiver(X[::GRAPH_STEP, ::GRAPH_STEP], Y[::GRAPH_STEP, ::GRAPH_STEP], vec_to_border_x[::GRAPH_STEP, ::GRAPH_STEP], vec_to_border_y[::GRAPH_STEP, ::GRAPH_STEP])
    if (SAVE_FIGURES):
        plt.savefig('images/env_orbit_5_direction_to_border.png', dpi=1200)
    if (SHOW_FIGURES):
        plt.show()
    else:
        plt.clf()

    plt.title('Environment Orbit Directions')
    cp = plt.quiver(X[::GRAPH_STEP, ::GRAPH_STEP], Y[::GRAPH_STEP, ::GRAPH_STEP], orbit_vec_x[::GRAPH_STEP, ::GRAPH_STEP], orbit_vec_y[::GRAPH_STEP, ::GRAPH_STEP])
    if (SAVE_FIGURES):
        plt.savefig('images/env_orbit_6_environment_orbit_directions.png', dpi=1200)
    if (SHOW_FIGURES):
        plt.show()
    else:
        plt.clf()