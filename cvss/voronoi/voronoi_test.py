# coding=utf-8
import datetime
import numpy as np
from shapely.geometry import Polygon

from voronoi import get_voronoi_cells

##################
# Start of Script#
##################

# Measure Performance
begin_time = datetime.datetime.now()

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

print(datetime.datetime.now() - begin_time)

for cell in cells:
    print(cell)