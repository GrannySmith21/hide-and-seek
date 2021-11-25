#!/usr/bin/python3

"""
This is the main entry point for the particle filter exercise node. It
subscribes to laser, map, and odometry and creates an instance of
pf.PFLocaliser() to do the localisation.
"""
from geometry_msgs.msg import Twist, Pose, PoseArray
import rospy
import pf_localisation
import pf_localisation.pf
from pf_localisation.util import *
from pf_localisation import navigator
from geometry_msgs.msg import ( PoseStamped, PoseWithCovarianceStamped,
                                PoseArray, Quaternion )
from tf.msg import tfMessage
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry

from threading import Lock

import sys
from copy import deepcopy

class ParticleFilterLocalisationNode(object):

    def __init__(self, goal):
        self.goal = goal
        self.path_made = False
        
        self.path = []

        # ----- Minimum change (m/radians) before publishing new particle cloud and pose
        self._PUBLISH_DELTA = rospy.get_param("publish_delta", 0.1)  
        
        self._particle_filter = pf_localisation.pf.PFLocaliser()
        self.path_pub = rospy.Publisher("/path", PoseArray, queue_size=100)
                
        self._latest_scan = None
        self._last_published_pose = None
        self._initial_pose_received = False

        self._pose_publisher = rospy.Publisher("/estimatedpose", PoseStamped)
        self._amcl_pose_publisher = rospy.Publisher("/amcl_pose", PoseWithCovarianceStamped)
        self._cloud_publisher = rospy.Publisher("/particlecloud", PoseArray, queue_size=10)
        self._tf_publisher = rospy.Publisher("/tf", tfMessage)
        self._movement_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=100)
        rospy.loginfo("Waiting for a map...")
        try:
            ocuccupancy_map = rospy.wait_for_message("map", OccupancyGrid, 20)
        except:
            rospy.logerr("Problem getting a map. Check that you have a map_server"
                     " running: rosrun map_server map_server <mapname> " )
            sys.exit(1)
        rospy.loginfo("Map received. %d X %d, %f px/m." %
                      (ocuccupancy_map.info.width, ocuccupancy_map.info.height,
                       ocuccupancy_map.info.resolution))
        self._particle_filter.set_map(ocuccupancy_map)
        
        self._laser_subscriber = rospy.Subscriber("/base_scan", LaserScan,
                                                  self._laser_callback,
                                                  queue_size=1)
        self._initial_pose_subscriber = rospy.Subscriber("/initialpose",
                                                         PoseWithCovarianceStamped,
                                                         self._initial_pose_callback)
        self._odometry_subscriber = rospy.Subscriber("/odom", Odometry,
                                                     self._odometry_callback,
                                                     queue_size=1)

    def _initial_pose_callback(self, pose):
        """ called when RViz sends a user supplied initial pose estimate """
        self._particle_filter.set_initial_pose(pose)
        self._last_published_pose = deepcopy(self._particle_filter.estimatedpose)
        self._initial_pose_received = True
        self._cloud_publisher.publish(self._particle_filter.particlecloud)

    def _odometry_callback(self, odometry):
        """
        Odometry received. If the filter is initialised then execute
        a filter predict step with odeometry followed by an update step using
        the latest laser.
        """
        if self._initial_pose_received:
            pose = self._particle_filter.estimatedpose.pose.pose
            if not self.path_made:
                pose_array = PoseArray()
                print(pose.position)
                self.path = navigator.generate_path(self.convert_map_coords(pose.position.x, pose.position.y), self.goal)  
                self.path.append((0,620))
                self.path.append((0,0))
                for p in self.path:
                    pos = Pose()
                    x,y = self.convert_coords(p)
                    
                    pos.position.x = x
                    pos.position.y = y
                    pose_array.poses.append(pos)
                pose_array.header = self._particle_filter.particlecloud.header
                print(self.path)
                #print(self._particle_filter.particlecloud.poses)
                self.path_pub.publish(pose_array)
                self.path.reverse()
                self.path_made = True  
                input()
                
            if len(self.path) > 0:
                c = self.convert_coords(self.path[0])
                arrived = self.follow_path(pose, c)
                if arrived:
                    self.path = self.path[1:]
            
            t_odom = self._particle_filter.predict_from_odometry(odometry)
            t_filter = self._particle_filter.update_filter(self._latest_scan)
            if t_odom + t_filter > 0.1:
                rospy.logwarn("Filter cycle overran timeslot")
                rospy.loginfo("Odometry update: %fs"%t_odom)
                rospy.loginfo("Particle update: %fs"%t_filter)
        
    def _laser_callback(self, scan):
        """
        Laser received. Store a ref to the latest scan. If robot has moved
        much, republish the latest pose to update RViz
        """
        self._latest_scan = scan
        if self._initial_pose_received:
            if  self._sufficientMovementDetected(self._particle_filter.estimatedpose):
                # ----- Publish the new pose
                self._amcl_pose_publisher.publish(self._particle_filter.estimatedpose)
                estimatedpose =  PoseStamped()
                estimatedpose.pose = self._particle_filter.estimatedpose.pose.pose
                estimatedpose.header.frame_id = "map"
                self._pose_publisher.publish(estimatedpose)
                
                # ----- Update record of previously-published pose
                self._last_published_pose = deepcopy(self._particle_filter.estimatedpose)
        
                # ----- Get updated particle cloud and publish it
                self._cloud_publisher.publish(self._particle_filter.particlecloud)
        
                # ----- Get updated transform and publish it
                self._tf_publisher.publish(self._particle_filter.tf_message)
    
    def _sufficientMovementDetected(self, latest_pose):
        """
        Compares the last published pose to the current pose. Returns true
        if movement is more the self._PUBLISH_DELTA
        """
        # ----- Check that minimum required amount of movement has occurred before re-publishing
        latest_x = latest_pose.pose.pose.position.x
        latest_y = latest_pose.pose.pose.position.y
        prev_x = self._last_published_pose.pose.pose.position.x
        prev_y = self._last_published_pose.pose.pose.position.y
        location_delta = abs(latest_x - prev_x) + abs(latest_y - prev_y)

        # ----- Also check for difference in orientation: Take a zero-quaternion,
        # ----- rotate forward by latest_rot, and rotate back by prev_rot, to get difference)
        latest_rot = latest_pose.pose.pose.orientation
        prev_rot = self._last_published_pose.pose.pose.orientation

        q = rotateQuaternion(Quaternion(w=1.0),
                             getHeading(latest_rot))   # Rotate forward
        q = rotateQuaternion(q, -getHeading(prev_rot)) # Rotate backward
        heading_delta = abs(getHeading(q))
        #rospy.loginfo("Moved by %f"%location_delta)
        return (location_delta > self._PUBLISH_DELTA or
                heading_delta > self._PUBLISH_DELTA)

    def angle_of_line(x1, y1, x2, y2):
        return math.degrees(math.atan2(y1-y2, x2-x1))
    
    def compute_angle(self, pose, coords):
       

        x, y = pose.position.x, pose.position.y
        return math.atan2(coords[1] - y, coords[0] - x)
        
    
    def follow_path(self,pose, path):
        direction_to_turn = self.pointToTarget(self.compute_angle(pose, path), pose.orientation)
        if self.collision_inbound():
            pass


        elif  direction_to_turn is not 0:
            self.move_robot(0 , direction_to_turn *.5)
                    
        elif not self.approximate_same_position(pose, path):
            self.move_robot(2, 0)
        else:
            return True
        return False
    import math
    def convert_coords(self,coords):
        x,y = coords
        coords_l = 620
        map_l = 30.0
        c = coords_l / map_l
        
        return ((y / c), (x/c))
    def convert_map_coords(self, x, y):
        coords_l = 620
        map_l = 30
        c = coords_l / map_l
        return (int(c * (y  )), int(c * (x ) ))
    def pointToTarget(self, angle, rot):
        #theta = math.acos(rot.w) * 2
        siny_cosp = 2 * (rot.w * rot.z + rot.x * rot.y)
        cosy_cosp = 1 - 2 * (rot.y * rot.y + rot.z * rot.z)
        radian_z = math.atan2(siny_cosp, cosy_cosp)
        
        r = int(10 * radian_z)
        a = int(10 * angle)
    
        d = a - r
        if d < 0:
            return -1
        if d > 0:
            return 1
        return 0

    def approximate_same_position(self, pose, coords):
        noise = 0.5
        x, y = pose.position.x, pose.position.y
        d = math.dist([x, y], [coords[0],coords[1]])
        print( x, coords[0], y, coords[1])
        print(d)
        if d < noise:
            return True
        
        return False
    def move_robot(self, vel, angle):
        base_data = Twist()
        base_data.linear.x = vel
        base_data.angular.z = angle
        self._movement_publisher.publish(base_data)
if __name__ == '__main__':
    # --- Main Program  ---
    rospy.init_node("pf_localisation")
    node = ParticleFilterLocalisationNode((380, 50))
    

    rospy.spin()