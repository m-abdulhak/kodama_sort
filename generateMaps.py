import json
import numpy as np
import pylab as plt
import skfmm
import pickle

SAVE_FIGURES = True
SHOW_FIGURES = True
MAP_SCALE = 0.25

def readConfig(filePath):
    # Opening JSON file
    f = open(filePath)
    data = json.load(f)
    f.close()
    return data

config = readConfig('cvss_config.json')
env = config["env"]
obs = config["staticObstacles"]
goals = config["goals"]

envXMax = int(max([x[0] for x in env]) * MAP_SCALE) 
envYMax = int(max([x[1] for x in env]) * MAP_SCALE)

print("Env: Max X:", envXMax,"Max Y:", envYMax)
staticObstacles = []

for o in obs:
    xMin = int(min([x[0] for x in o]) * MAP_SCALE)
    yMin = int(min([x[1] for x in o]) * MAP_SCALE)
    xMax = int(max([x[0] for x in o]) * MAP_SCALE)
    yMax = int(max([x[1] for x in o]) * MAP_SCALE)
    print("Obstacle:", o, "=>", {"xMin": xMin, "xMax": xMax, "yMin": yMin, "yMax": yMax})
    staticObstacles.append({"xMin": xMin, "xMax": xMax, "yMin": yMin, "yMax": yMax})

print("Obs:", staticObstacles)

for indx, raw_goal in enumerate(goals):
    goal = [int(raw_goal[0] * MAP_SCALE), int(raw_goal[1] * MAP_SCALE)]
    print("Goal:", raw_goal, "=>", int(goal[0] * MAP_SCALE), int(goal[1] * MAP_SCALE))

    X, Y = np.meshgrid(np.linspace(0, envXMax, envXMax+1), np.linspace(0, envYMax, envYMax+1))
    map_goal_mask = -1*np.ones_like(X)

    map_goal_mask[np.logical_and(X==goal[0], Y==goal[1])] = 1
    
    
    speed = 1*np.ones_like(X)

    for o in staticObstacles:
        speed[np.logical_and.reduce([X>o["xMin"], X<o["xMax"], Y>o["yMin"], Y<o["yMax"]])] = 0
    
    t = skfmm.travel_time(map_goal_mask, speed, 1)
    
    t.fill_value = np.NaN
    t_filled = t.filled()
    
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
