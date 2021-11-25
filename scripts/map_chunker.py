import copy
import json

from PIL import Image
import numpy as np


class PathFinder():

    def __init__(self, map_uri):

        self.map_uri = map_uri
        obstacle_growth = 5
        self.map_data = json.load(open(map_uri, 'r'))["data"]
        
        self.grow_map(obstacle_growth)


    def grow_map(self, factor):

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
        """for test in range(size_y):
            print(self.map_data[test])"""
        x, y = 400, 70
        size = 10
        for i in range(x,x + size):
            for j in range(y, y + size):

                self.map_data[i][j] = 1
        img = Image.fromarray((np.array(self.map_data) * 255).astype(np.uint8))
        
        img.show()
        f = open("zommed_map.json", "w")
        json_data = {"x_length" : size_x, "y_length" : size_y, "data" : self.map_data}
        json.dump(json_data, f)


if __name__ == '__main__':
    # --- Main Program  ---
    node = PathFinder("map_data_2d")