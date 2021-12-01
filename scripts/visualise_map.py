#!/usr/bin/env python3
import rospy
import json
import PIL.Image as pil
from nav_msgs.msg import OccupancyGrid
import numpy as np
def callback(data):
    # Create Json file of map. This static file will be used to create paths.
    row = data.info.width
    col = data.info.height
    data=data.data
    
    data2d = [data[i:i+row] for i in range(0,row*col,row)]
    f = open("map_data_2d", "w")
    json_data = {"x_length" : row, "y_length" : col,"data":data2d }
    json.dump(json_data,f)


data = [
    [100,100,100,100,100,100],
    [100,  0,  0,100,  0,100],
    [100,  0,  0,100,  0,100],
    [100,  0,  0,  0,  0,100],
    [100,100,100,100,100,100]
    ]

def heurstic_dist(posx, posy, target_x, target_y):
    return (posx - target_x)**2 + (posy - target_y)**2
    
def astar(data, target_x, target_y, curr_x, curr_y, prev_pos = []):
    if curr_x == target_x and curr_y == target_y:
        return True, prev_pos
    next_pos = []
    
    for i in range(-1,2):
        for j in range(-1,2):
            if i == 0 and j == 0:
                continue
            _x = curr_x + i
            _y = curr_y + j
            p = (_x, _y)
            if p in prev_pos:
                continue
            if _y > len(data) or _y < 0 or _x < 0 or _x > len(data[0]):
                continue
            if data[_y][_x] ==0:  
                next_pos.append(p)
    
    next_pos = next_pos.sort(key = lambda x: heurstic_dist(x[0], x[1], target_x, target_y))
    for i in range(len(next_pos)):
        new_prev_pos = prev_pos.append(next_pos[i])
        result, solution = astar(data,target_x,target_y, next_pos[i][0], next_pos[i][1], new_prev_pos)
        if result:
            return True, solution
    return False, []
                

def pixel_mapify(data):
    import PIL.Image as pil
    import numpy as np
    for i in range(len(data)):
        data[i] = map(lambda x: 1.0 if x == 100 else 0.0, data[i])
    data = np.array(data)
    img = pil.fromarray(data)
    img.show()

pixel_mapify(json.load(open("map_data_2d", )["data"])

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
     