import random
import math

class RRTGraph:
    def __init__(self, start, goal, map, width, height):
        (x,y) = start
        self.goal = goal
        self.width = width
        self.height = height
        self.map = map

        # tree
        self.x = []
        self.y = []
        self.parent = []
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)

        self.goalFlag = False

        # path
        self.goalstate = None
        self.path = []

    def add_node(self, n, x, y):
        self.x.insert(n,x)
        self.y.insert(n,y)

    def remove_node(self, n):
        self.x.pop(n)
        self.y.pop(n)

    def add_edge(self, parent, child):
        self.parent.insert(child, parent)

    def remove_edge(self, n):
        self.parent.pop(n)

    def number_of_nodes(self):
        return len(self.x)

    def distance(self, n1, n2):
        a = [self.x[n1], self.y[n1]]
        b = [self.x[n2], self.y[n2]]
        return math.dist(a, b)

    def sample_envir(self):
        x = int(random.uniform(0, self.width))
        y = int(random.uniform(0, self.height))
        return x, y

    def nearest(self, n):
        dmin = self.distance(0, n)
        nnear = 0
        for i in range(0, n):
            if self.distance(i, n) < dmin:
                dmin = self.distance(i, n)
                nnear = i
        return nnear

    def isFree(self):
        n = self.number_of_nodes() - 1
        point_value = self.map[self.y[n]][self.x[n]]
        if point_value == 0:
            return True
        return False

    def crossObstacle(self, x1, x2, y1, y2):
        for i in range(0,101):
            u = i/100
            x = (int)(x1 * u + x2 * (1 - u))
            y = (int)(y1 * u + y2 * (1 - u))
            if not self.map[y][x] == 0:
                return True
        return False

    def connect(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        if self.crossObstacle(x1, x2, y1, y2):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1, n2)
            return True

    def step(self, nnear, nrand, dmax=20):
        d = self.distance(nnear, nrand)
        if d > dmax:
            u = dmax / d
            (xnear, ynear) = (self.x[nnear], self.y[nnear])
            (xrand, yrand) = (self.x[nrand], self.y[nrand])
            (px, py) = (xrand - xnear, yrand - ynear)
            theta = math.atan2(py, px)
            (x, y) = (int(xnear + dmax * math.cos(theta)),
                      int(ynear + dmax * math.sin(theta)))
            self.remove_node(nrand)
            if abs(x - self.goal[0]) <= dmax and abs(y - self.goal[1]) <= dmax:
                self.add_node(nrand, self.goal[0], self.goal[1])
                self.goalstate = nrand
                self.goalFlag = True
            else:
                self.add_node(nrand, x, y)

    def path_to_goal(self):
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalstate)
            newpose = self.parent[self.goalstate]
            while (newpose != 0):
                self.path.append(newpose)
                newpose = self.parent[newpose]
            self.path.append(0)
        return self.goalFlag

    def getPathCoords(self):
        pathCoords = []
        for node in self.path:
            x, y = (self.x[node], self.y[node])
            pathCoords.append((x,y))
        return pathCoords

    def bias(self, ngoal, dmax):
        n = self.number_of_nodes()
        self.add_node(n, ngoal[0], ngoal[1])
        nnear = self.nearest(n)
        self.step(nnear, n, dmax)
        self.connect(nnear, n)
        return self.x, self.y, self.parent

    def expand(self, dmax):
        n = self.number_of_nodes()
        x, y = self.sample_envir()
        self.add_node(n, x, y)
        if self.isFree():
            xnearest = self.nearest(n)
            self.step(xnearest, n, dmax)
            self.connect(xnearest, n)
        else:
            self.remove_node(n)
        return self.x, self.y, self.parent
