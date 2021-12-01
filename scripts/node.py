#!/usr/bin/python3

"""
This is the main entry point for the particle filter exercise node. It
subscribes to laser, map, and odometry and creates an instance of
pf.PFLocaliser() to do the localisation.
"""
from tf.transformations import euler_from_quaternion    
import math
from geometry_msgs.msg import Twist, Pose, PoseArray
import rospy
import pf_localisation
import pf_localisation.pf
from pf_localisation.util import *
from navigation import navigator
from geometry_msgs.msg import (PoseStamped, PoseWithCovarianceStamped,
                                PoseArray, Quaternion)
from tf.msg import tfMessage
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry

from threading import Lock

import sys
from copy import deepcopy

class RobotNode(object):
    def begin(self, pose):
        _pose = self._particle_filter.estimatedpose
        _pose.pose.pose = pose 
        self._particle_filter.set_initial_pose(_pose)
        self._last_published_pose = deepcopy(self._particle_filter.estimatedpose)
        self._initial_pose_received = True
        self._cloud_publisher.publish(self._particle_filter.particlecloud)
    def __init__(self, ID,ocuccupancy_map, **kwargs):
        self.id = ID
        #rospy.init_node(f"robot_{ID}")
        if "goal" in kwargs.keys():
            self.goal = kwargs['goal']
            self.hider = True
        else:
            self.hider = False
            
        
        self.path_made = False
        self.reverse = False
        self.angle = 0
        self.path = []
        
        # ----- Minimum change (m/radians) before publishing new particle cloud and pose
        self._PUBLISH_DELTA = rospy.get_param("publish_delta", 0.1)  
        
        self._particle_filter = pf_localisation.pf.PFLocaliser()
                
        self._latest_scan = None
        self._last_published_pose = None
        self._initial_pose_received = False

        
        self.path_pub = rospy.Publisher(f"/robot_{ID}/path", PoseArray, queue_size=100)
        self._pose_publisher = rospy.Publisher(f"/robot_{ID}/estimatedpose", PoseStamped)
        self._amcl_pose_publisher = rospy.Publisher(f"/robot_{ID}/amcl_pose", PoseWithCovarianceStamped)
        self._cloud_publisher = rospy.Publisher(f"/robot_{ID}/particlecloud", PoseArray, queue_size=10)
        self._tf_publisher = rospy.Publisher(f"/tf", tfMessage)
        self._movement_publisher = rospy.Publisher(f"/robot_{ID}/cmd_vel", Twist, queue_size=100)
        
        rospy.loginfo("Map received. %d X %d, %f px/m." %
                      (ocuccupancy_map.info.width, ocuccupancy_map.info.height,
                       ocuccupancy_map.info.resolution))
        self._particle_filter.set_map(ocuccupancy_map)
        
        self._laser_subscriber = rospy.Subscriber(f"/robot_{ID}/base_scan", LaserScan,
                                                  self._laser_callback,
                                                  queue_size=1)
        self._initial_pose_subscriber = rospy.Subscriber(f"/robot_{ID}/initialpose",
                                                         PoseWithCovarianceStamped,
                                                         self._initial_pose_callback)
        self._odometry_subscriber = rospy.Subscriber(f"/robot_{ID}/odom", Odometry,
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
                self.path = navigator.generate_path(self.convert_map_coords(pose.position.x, pose.position.y), self.goal)  
                self.path = list(map(self.convert_coords, self.path))
                self.path.reverse()
                #self.path.append((15,0))
                
                
                self.public_path()
                self.path_made = True
                print(self.path)
                #input()
                
                
            if len(self.path) > 0:
                c = self.path[0]
                arrived = self.follow_path(pose, c)
                if arrived:
                    self.path = self.path[1:]
                    self.public_path()

            t_odom = self._particle_filter.predict_from_odometry(odometry)
            t_filter = self._particle_filter.update_filter(self._latest_scan)
            if t_odom + t_filter > 0.1:
                rospy.logwarn("Filter cycle overran timeslot")
                rospy.loginfo("Odometry update: %fs"%t_odom)
                rospy.loginfo("Particle update: %fs"%t_filter)

    
    
    def public_path(self):#, path):
        pose_array = PoseArray()
        #print(self.id, path)
        for p in self.path:
            pos = Pose()
            x,y = p
            
            pos.position.x = x
            pos.position.y = y
            pose_array.poses.append(pos)
        
        pose_array.header = self._particle_filter.particlecloud.header
        self.path_pub.publish(pose_array)
    def _laser_callback(self, data):
        """
        Laser received. Store a ref to the latest scan. If robot has moved
        much, republish the latest pose to update RViz
        """
        self._latest_scan = data
        
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
        avg = lambda x: sum(x)/len(x)
        split_data = self.split(data.ranges, 5)
        l,lm,m,rm,r = split_data[0],split_data[1],split_data[2],split_data[3],split_data[4]
        min_lm,min_m, min_rm = min(lm),min(m),min(rm)
        l,lm,m,rm,r = avg(l), avg(lm), avg(m), avg(rm), avg(r)
        LARGE_SPACE = 3

        CLOSE_SPACE = 0.5
        WALL_SPACE = 0.5
        self.reverse = (m < CLOSE_SPACE or lm < CLOSE_SPACE or 
        rm < CLOSE_SPACE)
        
        if lm < CLOSE_SPACE:
            self.angle = 1.57
            #print(self.reverse)
        if rm < CLOSE_SPACE:
            self.angle = -1.57
            #print(self.reverse)
        
        if not self.reverse:
            if m > LARGE_SPACE or (r < m and l < m): #lots of space in front or more space in front than on sides
                if r < WALL_SPACE:
                    self.angle = 1.57/2
                elif l < WALL_SPACE:
                    self.angle = -1.57/2
            else:
                self.angle = 0
        #print(self.angle, self.reverse)
    def split(self, a, n):
        k, m = divmod(len(a), n)
        return list((a[i*k+min(i, m):(i+1)*k+min(i+1, m)] for i in range(n)))

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

   
    def compute_angle(self, pose, coords):
        x, y = pose.position.x,pose.position.y 
        
        coords = round(coords[0],2), round(coords[1],2)
        angle = math.atan2((y -coords[1]), (x - coords[0]))
        normalise = lambda x : (x + (math.pi*2))%(math.pi*2)
        return angle
    
    
    def follow_path(self,pose, path):
        turn, angle = self.pointToTarget(self.compute_angle(pose, path), pose.orientation)
        arrived, distance = self.approximate_same_position(pose, path)
        if self.reverse:
            self.move_robot(-1 , self.angle)
            return
        if not arrived:
            if turn:
                self.move_robot(0 , angle)
                print("turning")
            else:
                angle = self.angle
                print("moving")
                self.move_robot(distance, angle)
        
        else:
            return True
        return False
    
    def convert_coords(self,coords):
        x,y = coords
        coords_l = 620
        map_l = 30.0
        c = coords_l/map_l
        t = 0.5
        return ((x/c)+ t,(y / c)+t)   
    def convert_map_coords(self, x, y):
        coords_l = 620
        map_l = 30.0
        c = coords_l/map_l
        return (round(c * (x ) ),round(c * (y )))

    def pointToTarget(self, angle, rot):
        #theta = math.acos(rot.w) * 2
        rotlist = [rot.x, rot.y, rot.z, rot.w]
        
        _,_,radian_z = euler_from_quaternion(rotlist)
        r = radian_z + math.pi
        angle = (angle+ (math.pi*2))%(math.pi*2)
        r = round(r,2)
        a = round(angle,2)
        #print("radian z", r, "angle", a)
        d = a - r
        threshold = 0.4
        if d < threshold and d > -threshold:
            return False, 0
        
        return True, d
    def approximate_same_position(self, pose, coords):
        noise = 0.1
        x, y = pose.position.x, pose.position.y
        d = math.dist([x, y], [coords[0],coords[1]])
        if d < noise:
            return True, 0
        return False, d
    def move_robot(self, vel, angle):
        base_data = Twist()
        base_data.linear.x = vel
        base_data.angular.z = angle
        self._movement_publisher.publish(base_data)
if __name__ == '__main__':
    # --- Main Program  ---
    rospy.init_node("robot")
    rospy.loginfo("Waiting for a map...")
    try:
        ocuccupancy_map = rospy.wait_for_message("/map", OccupancyGrid, 20)
    except Exception as e:
        rospy.logerr(f"Problem getting a map. Check that you have a map_server"
                    f" running: rosrun map_server map_server <mapname> {e}" )
        sys.exit(1)
    node = RobotNode(0,ocuccupancy_map, goal = (393, 62))
    rospy.spin()