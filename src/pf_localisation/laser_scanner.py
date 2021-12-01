#!/usr/bin/env python

import rospy, numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
class laser_scanner:
    def __init__(subscriber = "base_scan"):
        rospy.init_node('navigator', anonymous=True)

        rospy.Subscriber(subscriber, LaserScan, self.callback)
        self.angle = 0
        self.reverse = False
    def callback(self, data):
        split_data = self.split(data.ranges, 5)
        l,lm,m,rm,r = split_data[0],split_data[1],split_data[2],split_data[3],split_data[4]
        LARGE_SPACE = 3

        CLOSE_SPACE = 0.4
        WALL_SPACE = 0.5
        self.reverse = m < CLOSE_SPACE or lm < CLOSE_SPACE or rm < CLOSE_SPACE
        if m > LARGE_SPACE or (r < m and l < m): #lots of space in front or more space in front than on sides
            if r < WALL_SPACE:
                self.angle = 1.57/2
            elif l < WALL_SPACE:
                self.angle = -1.57/2
        else:
            self.angle = 0

    def split(self, a, n):
        k, m = divmod(len(a), n)
        return list((a[i*k+min(i, m):(i+1)*k+min(i+1, m)] for i in range(n)))
    