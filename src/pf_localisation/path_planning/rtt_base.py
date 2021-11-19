"""
Class implementing a Rapidly expoloring Random Tree
Adapted from the python RTT path planning videos series of the youtube channel "Algobotics",
the article from Justin Svegliato published on Towards Data Science https://towardsdatascience.com/how-does-a-robot-plan-a-path-in-its-environment-b8e9519c738b,
and the article from Tim Chin published on his blog https://theclassytim.medium.com/robotic-path-planning-rrt-and-rrt-212319121378
"""

import random
import math

class RRTGraph:
    def __init__(self, start, goal, map, width, height):
        """
        Initialize the RRT

        :Args:
            | start: the tuple of the starting point coordinates (int, int)
            | goal: the tuple of the ending point coordinates (int, int)
            | map: the matrix defining the map
            | width: the width of the map in number of points
            | height: the height of the map in number of points
        """
        (x,y) = start
        self.goal = goal
        self.width = width
        self.height = height
        self.map = map

        # adding the starting state to the graph
        self.x = []
        self.y = []
        self.parent = []
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)

        self.goalFlag = False

        # path
        self.goalstate = None # we haven't reach the goal state yet
        self.path = []

    def add_node(self, n, x, y):
        """
        Add a node to the graph
        :Args:
            | n: index of the node (int)
            | x: x coordinate of the node (int)
            | y: y coordinate of the node (int)
        """
        self.x.insert(n,x)
        self.y.insert(n,y)

    def remove_node(self, n):
        """
        Remove a node to the graph
        :Args:
            | n: index of the node (int)
        """
        self.x.pop(n)
        self.y.pop(n)

    def add_edge(self, parent, child):
        """
        Construct an edge between two nodes
        :Args:
            | parent: index of the parent node (int)
            | child: index of the child node (int)
        """
        self.parent.insert(child, parent)

    def remove_edge(self, n):
        """
        Remove an edge between two nodes
        :Args:
            | parent: index of the child node (int)
        """
        self.parent.pop(n)

    def number_of_nodes(self):
        """
        Computes the number of nodes currently in the graph
        :Return:
            | return: number of nodes currently in the graph (int)
        """
        return len(self.x)

    def distance(self, n1, n2):
        """
        Computes the distance between two nodes
        :Args:
            | n1: index of the first node (int)
            | n2: index of the second node (int)
        :Return:
            | return: the distance between n1 and n2 (float)
        """
        a = [self.x[n1], self.y[n1]]
        b = [self.x[n2], self.y[n2]]
        return math.dist(a, b)

    def sample_envir(self):
        """
        Computes a random point coordinates within the map
        :Return:
            | return: coordinates x and y of the random point
        """
        x = int(random.uniform(0, self.width))
        y = int(random.uniform(0, self.height))
        return x, y

    def nearest(self, n):
        """
        Finds the nearest node to node n
        :Args:
            | n: index of the node n (int)
        :Return:
            | return: index of the closest node from node n (int)
        """
        dmin = self.distance(0, n)
        nnear = 0
        for i in range(0, n):
            if self.distance(i, n) < dmin:
                dmin = self.distance(i, n)
                nnear = i
        return nnear

    def is_free(self):
        """
        Computes if a the latest node in the graph is within the free space.
        i.e: not in a wall of outside the borders
        :Return:
            | return: true if the node is in the free space else false (bool)
        """
        n = self.number_of_nodes() - 1
        point_value = self.map[self.y[n]][self.x[n]]
        if point_value == 0:
            return True
        return False

    def cross_obstacle(self, x1, x2, y1, y2):
        """
        Search for obstacles on the path between two nodes
        :Args:
            | x1: x coordinate of node 1
            | x2: x coordinate of node 2
            | y1: y coordinate of node 1
            | y2: y coordinate of node 2
        :Return:
            | return: true if there is an obstacle between the two nodes else false (bool)
        """
        for i in range(0,101):
            u = i/100 # generates 100 intermediates locations between the two nodes to check the presence of an obstacle
            x = (int)(x1 * u + x2 * (1 - u))
            y = (int)(y1 * u + y2 * (1 - u))
            if not self.map[y][x] == 0:
                return True
        return False

    def connect(self, n1, n2):
        """
        Add an edge between two nodes if there is no obstacles between them
        :Args:
            | n1: index of node 1 (int)
            | n2: index of node 2 (int)
        :Return:
            | return : true if an edge is crated else false (bool)
        """
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        if self.cross_obstacle(x1, x2, y1, y2):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1, n2)
            return True

    def step(self, nnear, nrand, dmax=20):
        """
        Add a node located between nneat and nrand to the graph.
        nnear is the nearest node to nrand which location is random.
        If the distance between nnear and nrand is more than dmax, we add a node to the graph wich is located on the
        nneat to nrand axis at a distance of dmax from nnear.
        :Args:
            | nnear: index of the node nnear (int)
            | nrand: index of the node nread (int)
            | dmax: maximun accepted distance between two nodes (int)
        """
        d = self.distance(nnear, nrand)
        if d > dmax:
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
        """
        Computes if the goal location has been reached or not.
        If so, computes the indexes of the nodes on the path to the goal.
        :Return:
            | return: if the goal has been reached (bool)
        """
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalstate)
            newpose = self.parent[self.goalstate]
            while (newpose != 0):
                self.path.append(newpose)
                newpose = self.parent[newpose]
            self.path.append(0)
        return self.goalFlag

    def get_path_coords(self):
        """
        Construct the location of the nodes on the path to the goal.
        :Return:
            | return: an array of the location on the path between the start location and the goal location,
            starting from the start location ([(x,y), ...])
        """
        pathCoords = []
        for node in self.path:
            x, y = (self.x[node], self.y[node])
            pathCoords.append((x,y))
        return pathCoords

    def bias(self, ngoal, dmax):
        """
        Try to add the goal node to the path. So an intermediate node on the axis between the goal node and the nearest
        one can be created at a distance of dmax from each other if there is no obstacle between them.
        :Args:
            | ngoal: tuple of the coordinates of the goal node (int, int)
            | dmax: maximun accepted distance between two nodes (int)
        """
        n = self.number_of_nodes()
        self.add_node(n, ngoal[0], ngoal[1])
        nnear = self.nearest(n)
        self.step(nnear, n, dmax)
        self.connect(nnear, n)
        #return self.x, self.y, self.parent

    def expand(self, dmax):
        """
        Try to add a node to the graph, finds the closest node to the random node, if it is in the free space add an
        intermediate node at a distance dmax from the closest node, and if this node is still in the free space and if
        there is no obstacles between the temporary node and the closest node, create an egde between those two.
        :Args:
            | dmax: maximun accepted distance between two nodes (int)
        """
        n = self.number_of_nodes()
        x, y = self.sample_envir()
        self.add_node(n, x, y)
        if self.is_free():
            xnearest = self.nearest(n)
            self.step(xnearest, n, dmax)
            self.connect(xnearest, n)
        else:
            self.remove_node(n)
        #return self.x, self.y, self.parent
