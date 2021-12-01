  
import os
from . import path_planner
def generate_path(start,end):
    absolute_path = os.path.abspath(__file__)
    path_finder = path_planner.PathPlanner(os.path.dirname(absolute_path) + "/map_data_2d")
    result = False
    print(start, end)
    while not result:
        try:
            #print("here")
            return path_finder.path_planner(start, end)
        except:
            print("searching solution..")


