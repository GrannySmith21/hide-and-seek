"""
Class used as an interface to computes path between a starting and an ending location using an RTT based algorithm
"""

import copy
import json
import time

"""import numpy as np
from scipy import interpolate"""

from . rtt_base import *

class PathPlanner:

    def __init__(self, map_uri, biais=9, dmax=30, obstacle_growth=7):
        """
        Initialize the PathPlanner class
        :Args:
            | map_uri: uri to the json file location containing map data (String)
            | biais: 1/biais is the % of biais wanted. i.e the % of nodes created towards the goal to accelerate the search
            | dmax: maximun accepted distance between two nodes (int)
            | obstacle_growth: the number of points by which to increase the size of the obstacles, so the path is not
            close to the walls (int)
        """

        self.map_uri = map_uri
        self.biais = biais
        self.dmax = dmax
        my_json = json.load(open(map_uri, 'r'))
        self.dimensions = (my_json["x_length"], my_json["y_length"])
        self.map_data = my_json["data"]
        self.grow_map(obstacle_growth)


    def path_planner(self, start=(50, 380), goal=(380, 50)):
        """
        Computes a path between a starting point and an ending point
        :Args:
            | start: tuples coordinates of the starting point (int, int)
            | goal: tuples coordinates of the ending point (int, int)
        :Return:
            | return: an array of the location on the path between the start location and the goal location,
            starting from the start location ([(x,y), ...])
        """

        graph = RRTGraph(start, goal, self.map_data, self.dimensions[0], self.dimensions[1])

        iteration = 0
        t1 = time.time()

        while (not graph.path_to_goal()):

            # timeout
            elapsed = time.time() - t1
            if elapsed > 1:
                raise

            if iteration % self.biais == 0:
                graph.bias(goal, self.dmax)
            else:
                graph.expand(self.dmax)
            iteration += 1
    
        return graph.get_path_coords()


    """def basic_smoothing(self, path):

        result = []
        x_list = []
        y_list = []

        for points in path:
            x_list.append(points[0])
            y_list.append(points[1])

        tck, *rest = interpolate.splprep([x_list, y_list])
        all_points = np.linspace(0, 1, num=500)

        x_temp, y_temp = interpolate.splev(all_points, tck)
        for x, y in zip(x_temp, y_temp):
            x_int = int(x)
            y_int = int(y)
            result.append((x_int, y_int))

        return result"""


    def grow_map(self, factor):
        """
        Increase the size of the obstacles, so the robot's path is not close to the walls
        :Args:
            | factor: the number of points by which to increase the size of the obstacles, so the path is not
            close to the walls (int)
        """

        size_x = len(self.map_data[0])
        size_y = len(self.map_data)
        map_data_copy = copy.deepcopy(self.map_data)
        for x in range(size_x):
            for y in range(size_y):
                if map_data_copy[y][x] == 100:
                    reset_range_x = x - factor
                    reset_range_y = y - factor
                    if reset_range_x < 0:
                        reset_range_x = 0
                    elif reset_range_y < 0:
                        reset_range_y = 0
                    for i in range(reset_range_x, x+factor+1):
                        for j in range(reset_range_y, y+factor+1):
                            if not (i >= size_x or j >= size_y):
                                self.map_data[j][i] = 100