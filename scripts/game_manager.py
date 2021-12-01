#!/usr/bin/python3
# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from geometry_msgs/Point.msg. Do not edit."""
import codecs
import random
import sys
import genpy
import struct
import rospy
from geometry_msgs.msg import Twist, Pose
from node import RobotNode
from nav_msgs.msg import OccupancyGrid
class GameManager():

    def __init__(self, hidingSpots =[(1.984, -11.89), (3.05, -7.99),( 10.62, 0.11),(6.56, -1.50), (1.96, -5.11), (1.02, -1.76), (1.94, 5.77), (-3.74, 1.00),
                                   (-4.1, 3.63), (-5.78, 5.05), (-8.773, 5.57), (-17.09, 5.854), (-12.80, 7.377),
                                   (-6.14, 11.21), (-9.35, 12.85), (-4.31, 15.46)], 
                                   no_of_hiders = 2, no_of_seekers = 1):
        
        self.hiders_list: list = []
        self.active_hiders: list = []
        self.seekers_list = []
        self.no_of_seekers: int = no_of_seekers
        self.no_of_hiders: int = no_of_hiders
        self.hiding_spots: list = hidingSpots
        self.occupied_hiding_spots: list = []

        

    def _get_list_of_active_players(self):
        return self.active_hiders

    def _setup_list_of_players(self):
        rospy.loginfo("Waiting for a map...")
        try:
            ocuccupancy_map = rospy.wait_for_message("/map", OccupancyGrid, 20)
        except Exception as e:
            rospy.logerr(f"Problem getting a map. Check that you have a map_server"
                     f" running: rosrun map_server map_server <mapname> {e}" )
            sys.exit(1)
        for i in range(self.no_of_hiders):
            chosen = self.find_position_to_hide()
            rob_hiding_spot = self.stage_ros_to_map_conversion(chosen)
            print(rob_hiding_spot, chosen)
            self.hiders_list.append(RobotNode(i, ocuccupancy_map, goal = rob_hiding_spot))
            
        #To do: set up seekers, make Robot seeker
            
        self.active_hiders = self.hiders_list
    def stage_ros_to_map_conversion(self, coords):
        x,y = coords
        coords_l = 620
        map_l = 30.0
        c = coords_l/map_l
        print(coords)
        return (round(c * (x+15 ) ),round(c * (y+15 )))
    def _player_caught(self, playerId: int):
        for robot in self.active_hiders:
            if playerId == robot.id:
                print("Player", playerId, "Caught At")
                print(robot.hiding_spot.coordinate.x, ",", robot.hiding_spot.coordinate.y)
                self.active_hiders.pop(self.active_hiders.index(robot))
                break
                ## rose topic player eliminated
        self._check_if_game_over()

    def _check_if_game_over(self):
        if len(self.active_hiders) == 1:
            print("Game Over")
        else:
            print(len(self.active_hiders), "Players Still Left")

    def find_position_to_hide(self):
        hiding_spot = self.hiding_spots.pop(random.randrange(0, len(self.hiding_spots)))
        self.occupied_hiding_spots.append(hiding_spot)
        return hiding_spot

    def restart_game(self):
        self.no_of_players
    def run(self):
        for robot in self.hiders_list:
            p = Pose()
            x,y = 23, 7
            p.position.x = x
            p.position.y = y
            robot.begin(p)
            print(robot)
        # subscribe to all robots

if __name__=="__main__":
    rospy.init_node("game")
    game= GameManager(hidingSpots= [(-3,2),(-13, 3), (12,-3),(-3, 12) , (4,-12) ])
    game._setup_list_of_players()
    game.run()
    rospy.spin()

'''
## Debug Code for Game Manager
hidingspots = []
for i in range(15):
    hidingspots.append(HidingSpot(Cord(random.randrange(0,602), random.randrange(0,602)), False))
print(len(hidingspots))
a = GameManger(hidingspots)
a._setup_list_of_players()
print("Seeker", a.seeker.id)
a._player_caught(4)
a._player_caught(6)
a._player_caught(2)

'''

## 602
