import json
import numpy as np
import pylab as plt
import skfmm
import pickle
from shapely.geometry import Point, Polygon

SAVE_FIGURES = True
SHOW_FIGURES = True
MAP_SCALE = 0.25

def pointIsInsidePolygon(point, poly):
    try:
        return point.within(poly)
    except Exception as e:
        return False

def readConfig(filePath):
    # Opening JSON file
    f = open(filePath)
    data = json.load(f)
    f.close()
    return data

###########################################
####        Load Configurations        ####
###########################################

config = readConfig('cvss_config.json')
env = config["env"]
obs = config["staticObstacles"]
goals = list(map(lambda g: g["goal"], config["puckGroups"]))

envPoly = Polygon([p[0] * MAP_SCALE, p[1] * MAP_SCALE] for p in env)
envXMax = int(max([x[0] for x in env]) * MAP_SCALE) 
envYMax = int(max([x[1] for x in env]) * MAP_SCALE)
print("Env:", env, " Max X:", envXMax,"Max Y:", envYMax)

staticObstaclesPolygons = []
for o in obs:
    staticObstaclesPolygons.append(Polygon([p[0] * MAP_SCALE, p[1] * MAP_SCALE] for p in o))
print("Obs:", {ob.wkt for ob in staticObstaclesPolygons})


for indx, raw_goal in enumerate(goals):
    goal = [int(raw_goal[0] * MAP_SCALE), int(raw_goal[1] * MAP_SCALE)]
    print("Goal:", raw_goal, "=>", int(goal[0] * MAP_SCALE), int(goal[1] * MAP_SCALE))

    X, Y = np.meshgrid(np.linspace(0, envXMax, envXMax+1), np.linspace(0, envYMax, envYMax+1))
    map_goal_mask = -1*np.ones_like(X)
    speed = 1*np.ones_like(X)

    map_goal_mask[np.logical_and(X==goal[0], Y==goal[1])] = 1
    speed[np.logical_or(X>=envXMax, Y>=envYMax)] = 0
    speed[np.logical_or(X==0, Y==0)] = 0
    
    for x in range(0, envXMax):
        for y in range(0, envYMax):
            curPoint = Point(x,y)
            for o in staticObstaclesPolygons:
                if(pointIsInsidePolygon(curPoint, o)):
                    speed[y][x] = 0

    t = skfmm.travel_time(map_goal_mask, speed, 1)
    
    t.fill_value = np.NaN
    t_filled = t.filled()
    
    t_filled[X==0] = t_filled[X==1] + 1
    t_filled[Y==0] = t_filled[Y==1] + 1
    t_filled[X==envXMax] = t_filled[X==envXMax-1] + 1
    t_filled[Y==envYMax] = t_filled[Y==envYMax-1] + 1
    
    yG, xG = np.gradient(t_filled)
    x_directions = -10 * xG
    y_directions = -10 * yG
    
    x_goals = x_directions + X
    y_goals = y_directions + Y
    
    goal_map = []
    
    for y, row in enumerate(x_goals):
        goal_map.append([])
        for x, val in enumerate(row):
            goal_map[y].append([x_goals[y][x], y_goals[y][x]])
        
    ###############################
    ##########  PICKLES  ##########
    ###############################
    
    file_name = 'map_pickles/' + str(indx) + '_goal_map.pickle'
    
    with open(file_name, 'wb') as handle:
        print('Dumping goal map to: ' + file_name)
        pickle.dump(goal_map, handle, protocol=2)

#     with open('goal_map.pickle', 'rb') as handle:
#         loaded_goal_map = pickle.load(handle)
#         print(loaded_goal_map[0][0][0] == goal_map[0][0][0])

    ###############################
    ##########  FIGURES  ##########
    ###############################
    
    if (SAVE_FIGURES or SHOW_FIGURES):
        plt.title('Goal Location')
        plt.contour(X, Y, map_goal_mask, [0], linewidths=(3), colors='red')
        if (SAVE_FIGURES):
            plt.savefig('images/' + str(indx) + '_1_map_goal_mask.png', dpi=1200)
        if (SHOW_FIGURES):
            plt.show()

        plt.title('Static Obstacles (Speed Map)')
        plt.contour(X, Y, speed, [0], linewidths=(1), colors='Blue')
        plt.contourf(X, Y, speed, 1)
        plt.colorbar()
        if (SAVE_FIGURES):
            plt.savefig('images/' + str(indx) + '_2_speed_map.png', dpi=1200)
        if (SHOW_FIGURES):
            plt.show()

        plt.title('Distance To Goal')
        plt.contour(X, Y, map_goal_mask,[0], linewidths=(3), colors='red')
        # cp = plt.contour(X, Y, t, 15)
        cp = plt.contourf(X, Y, t, 50)
        # plt.clabel(cp, inline=1, fontsize=10)
        plt.colorbar()
        if (SAVE_FIGURES):
            plt.savefig('images/' + str(indx) + '_3_distance_to_goal.png', dpi=1200)
        if (SHOW_FIGURES):
            plt.show()

        plt.title('Gradients of Distance To Goal')
        cp = plt.quiver(X[::5, ::5], Y[::5, ::5], xG[::5, ::5], yG[::5, ::5])
        if (SAVE_FIGURES):
            plt.savefig('images/' + str(indx) + '_4_gradients_of_distance_to_goal.png', dpi=1200)
        if (SHOW_FIGURES):
            plt.show()
        
        plt.title('Directions To Goal')
        cp = plt.quiver(X[::5, ::5], Y[::5, ::5], x_directions[::5, ::5], y_directions[::5, ::5])
        if (SAVE_FIGURES):
            plt.savefig('images/' + str(indx) + '_5_direction_to_goal.png', dpi=1200)
        if (SHOW_FIGURES):
            plt.show()