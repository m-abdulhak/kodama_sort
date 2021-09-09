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
    f = open(filePath)
    data = json.load(f)
    f.close()
    return data

MIN_DIRECTION = 30
ENV_ORBIT_STEEPNESS = 2
INCLUDE_OBSTACLES = True
SAVE_FIGURES = True
SHOW_FIGURES = True
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
envXMax = int(max([x[0] for x in env]) * MAP_SCALE) 
envYMax = int(max([x[1] for x in env]) * MAP_SCALE)
print("Env:", env, " Max X:", envXMax,"Max Y:", envYMax)

staticObstaclesPolygons = []
for o in obs:
    staticObstaclesPolygons.append(Polygon([p[0] * MAP_SCALE, p[1] * MAP_SCALE] for p in o))
print("Obs:", {ob.wkt for ob in staticObstaclesPolygons})

###########################################
#### Generate boundary and speed masks ####
###########################################

X, Y = np.meshgrid(np.linspace(0, envXMax, envXMax+1), np.linspace(0, envYMax, envYMax+1))
map_border_mask = -1*np.ones_like(X)
speed = 1*np.ones_like(X)

for x in range(0, envXMax):
    for y in range(0, envYMax):
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

yG, xG = np.gradient(t_filled)
x_directions = -10 * xG
y_directions = -10 * yG

###########################################
####    Generate Environment Orbits    ####
###########################################
x_directions_shifted = ENV_ORBIT_STEEPNESS * -1 *  y_directions + x_directions
y_directions_shifted = ENV_ORBIT_STEEPNESS * x_directions + y_directions

def vecLength(x, y):
    return sqrt(x*x + y*y)

for x in range(0, envXMax):
    for y in range(0, envYMax):
        curPoint = Point(x,y)
        if (map_border_mask[y][x] != -1):
            x_directions_shifted[y][x] = ENV_ORBIT_STEEPNESS * y_directions[y][x] + x_directions[y][x]
            y_directions_shifted[y][x] = ENV_ORBIT_STEEPNESS * -1 * x_directions[y][x] + y_directions[y][x]

        if (x_directions_shifted[y][x] >= 0 and x_directions_shifted[y][x] < MIN_DIRECTION):
            x_directions_shifted[y][x]  = MIN_DIRECTION
        if (x_directions_shifted[y][x] <= 0 and x_directions_shifted[y][x] > -1 * MIN_DIRECTION):
            x_directions_shifted[y][x]  = -1 * MIN_DIRECTION
        if (y_directions_shifted[y][x] >= 0 and y_directions_shifted[y][x] < MIN_DIRECTION):
            y_directions_shifted[y][x]  = MIN_DIRECTION
        if (y_directions_shifted[y][x] <= 0 and y_directions_shifted[y][x] > -1 * MIN_DIRECTION):
            y_directions_shifted[y][x]  = -1 * MIN_DIRECTION


from scipy.ndimage.filters import gaussian_filter

x_directions_shifted = gaussian_filter(x_directions_shifted, sigma=2)
y_directions_shifted = gaussian_filter(y_directions_shifted, sigma=2)

countAll = 0
sumAll = 0
minAll = None

for x in range(0, envXMax):
    for y in range(0, envYMax):
        curPoint = Point(x,y)        
        vLength = vecLength(x_directions_shifted[y][x], y_directions_shifted[y][x])

        if (vLength < MIN_DIRECTION / 2 and x_directions_shifted[y][x] >= 0 and x_directions_shifted[y][x] < MIN_DIRECTION):
            x_directions_shifted[y][x]  = MIN_DIRECTION / 2
        if (vLength < MIN_DIRECTION / 2 and x_directions_shifted[y][x] <= 0 and x_directions_shifted[y][x] > -1 * MIN_DIRECTION):
            x_directions_shifted[y][x]  = -1 * MIN_DIRECTION / 2
        if (vLength < MIN_DIRECTION / 2 and y_directions_shifted[y][x] >= 0 and y_directions_shifted[y][x] < MIN_DIRECTION):
            y_directions_shifted[y][x]  = MIN_DIRECTION / 2
        if (vLength < MIN_DIRECTION / 2 and y_directions_shifted[y][x] <= 0 and y_directions_shifted[y][x] > -1 * MIN_DIRECTION):
            y_directions_shifted[y][x]  = -1 * MIN_DIRECTION / 2
            
        vLength = vecLength(x_directions_shifted[y][x], y_directions_shifted[y][x])

        countAll += 1
        sumAll += vLength
        if minAll == None or vLength < minAll:
            minAll = vLength

# x_directions_shifted = gaussian_filter(x_directions_shifted, sigma=1)
# y_directions_shifted = gaussian_filter(y_directions_shifted, sigma=1)

print("Count", countAll)
print("Mean", sumAll / countAll)
print("Min", minAll)

x_goals = x_directions_shifted + X
y_goals = y_directions_shifted + Y

goal_map = []

for y, row in enumerate(x_goals):
    goal_map.append([])
    for x, val in enumerate(row):
        # if(x_directions)
        goal_map[y].append([x_goals[y][x], y_goals[y][x]])

###############################
##########  PICKLES  ##########
###############################

file_name = 'map_pickles/env_orbit_map.pickle'

with open(file_name, 'wb') as handle:
    print('Dumping map to: ' + file_name)
    pickle.dump(goal_map, handle, protocol=2)

#     with open('goal_map.pickle', 'rb') as handle:
#         loaded_goal_map = pickle.load(handle)
#         print(loaded_goal_map[0][0][0] == goal_map[0][0][0])

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

    plt.title('Static Obstacles (Speed Map)')
    plt.contour(X, Y, speed, [0], linewidths=(1), colors='Blue')
    plt.contourf(X, Y, speed, 1)
    plt.colorbar()
    if (SAVE_FIGURES):
        plt.savefig('images/env_orbit_2_speed_map.png', dpi=1200)
    if (SHOW_FIGURES):
        plt.show()

    plt.title('Distance To Border')
    plt.contour(X, Y, map_border_mask,[0], linewidths=(3), colors='red')
    cp = plt.contourf(X, Y, t, 50)
    plt.colorbar()
    if (SAVE_FIGURES):
        plt.savefig('images/env_orbit_3_distance_to_border.png', dpi=1200)
    if (SHOW_FIGURES):
        plt.show()

    plt.title('Gradient of Distance To Border')
    cp = plt.quiver(X[::GRAPH_STEP, ::GRAPH_STEP], Y[::GRAPH_STEP, ::GRAPH_STEP], xG[::GRAPH_STEP, ::GRAPH_STEP], yG[::GRAPH_STEP, ::GRAPH_STEP])
    if (SAVE_FIGURES):
        plt.savefig('images/env_orbit_4_gradients_of_distance_to_border.png', dpi=1200)
    if (SHOW_FIGURES):
        plt.show()

    plt.title('Directions To Border')
    cp = plt.quiver(X[::GRAPH_STEP, ::GRAPH_STEP], Y[::GRAPH_STEP, ::GRAPH_STEP], x_directions[::GRAPH_STEP, ::GRAPH_STEP], y_directions[::GRAPH_STEP, ::GRAPH_STEP])
    if (SAVE_FIGURES):
        plt.savefig('images/env_orbit_5_direction_to_border.png', dpi=1200)
    if (SHOW_FIGURES):
        plt.show()

    plt.title('Environment Orbit Directions')
    cp = plt.quiver(X[::GRAPH_STEP, ::GRAPH_STEP], Y[::GRAPH_STEP, ::GRAPH_STEP], x_directions_shifted[::GRAPH_STEP, ::GRAPH_STEP], y_directions_shifted[::GRAPH_STEP, ::GRAPH_STEP])
    if (SAVE_FIGURES):
        plt.savefig('images/env_orbit_6_environment_orbit_directions.png', dpi=1200)
    if (SHOW_FIGURES):
        plt.show()