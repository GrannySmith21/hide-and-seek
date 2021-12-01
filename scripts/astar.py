data = [
    [100,100,100,100,100,100],
    [100,  0,  0,100,  0,100],
    [100,  0,  0,100,  0,100],
    [100,  0,  0,  0,  0,100],
    [100,100,100,100,100,100]
    ]
import math
def heurstic_dist(posx, posy, target_x, target_y):
    return math.sqrt((posx - target_x)**2 + (posy - target_y)**2)
def F(p, tp, sp):
    g = heurstic_dist(p[0],p[1], sp[0],sp[1])
    h = heurstic_dist(p[0],p[1], tp[0],tp[1])
    return h + g
def astar(data, target_x, target_y, prev_pos):
    curr_x = prev_pos[-1][0]
    curr_y = prev_pos[-1][1]
    
    if curr_x == target_x and curr_y == target_y:
        return True, prev_pos
    next_pos = []
    
    for i in range(-1,2):
        for j in range(-1,2):
            if i == 0 and j == 0:
                continue # curr pos
            _x = curr_x + i#(i*10)
            _y = curr_y + j#(j*10)
            p = (_x, _y)
            if ((i == -1 or i == 1) and not j == 0) or ((j == -1 or j ==1) and not i ==0):
                continue # diagonal movement disallowed
            if p in prev_pos:
                continue # stop infinite loop retracing steps
            if _y > len(data) or _y < 0 or _x < 0 or _x > len(data[0]):
                continue # out of bounds
            if data[_y][_x] ==0:  
                next_pos.append(p) 
    
    next_pos.sort(key = lambda x: F(x, (target_x,target_y), (prev_pos[0][0], prev_pos[0][1])))
    print(next_pos)
    for i in range(len(next_pos)):
        
        new_prev_pos = prev_pos.copy()
        new_prev_pos.append(next_pos[i])
        
        result, solution = astar(data,target_x,target_y, new_prev_pos)
        if result:
            return True, solution
        
    return False, []

def print_data(data):
    for y in data:
        print(list(map(lambda x : "  0" if x == 0 else "100",y)))

#import json
#data = json.load(open("zommed_map.json", "r"))["data"]
print_data(data)
_, solution = astar(data, 1, 1,[(4,1)])
print(solution)

