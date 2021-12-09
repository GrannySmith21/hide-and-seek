#!/usr/bin/python3

"""
This is the main entry point for the particle filter exercise node. It
subscribes to laser, map, and odometry and creates an instance of
pf.PFLocaliser() to do the localisation.
"""
from std_msgs.msg import String
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
from sensor_msgs.msg import Image
from threading import Lock
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys
from copy import deepcopy

class RobotNode(object):
    """begin method
    return: void
    args: 
        self
        pose - a pose estimate for the starting position of the robot 
        (needs to be accurate, this pose is used to path find)
    This method initialises attributes of Robot, and lets it begin with
        path finding/ going to path/ particle filtering 
    """
    def begin(self, pose):
        _pose = self._particle_filter.estimatedpose
        _pose.pose.pose = pose 
        self._particle_filter.set_initial_pose(_pose)
        self._last_published_pose = deepcopy(self._particle_filter.estimatedpose)
        self._initial_pose_received = True
        self._cloud_publisher.publish(self._particle_filter.particlecloud)

    """__init__ constructor method
    args:
        self
        ID - must be a unique identifier for robot- must match up with Robot's ROS Topics
        (e.g. /robot_*ID*/cmd_vel)
        occupancy_map - map of world
        kwargs - optional params :
            goal - sets "goal" of path finder to this value
    """
    def __init__(self, ID,ocuccupancy_map, **kwargs):
        self.ID = ID
        self.is_seeker = False
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

        # init publishers and subscribers
        self.path_pub = rospy.Publisher(f"/robot_{ID}/path", PoseArray, queue_size=100)
        self._pose_publisher = rospy.Publisher(f"/robot_{ID}/estimatedpose", PoseStamped)
        self._amcl_pose_publisher = rospy.Publisher(f"/robot_{ID}/amcl_pose", PoseWithCovarianceStamped)
        self._cloud_publisher = rospy.Publisher(f"/robot_{ID}/particlecloud", PoseArray, queue_size=10)
        self._tf_publisher = rospy.Publisher(f"/tf", tfMessage)
        self._movement_publisher = rospy.Publisher(f"/robot_{ID}/cmd_vel", Twist, queue_size=100)
        self._robot_status = rospy.Publisher(f"/robot_{ID}/status", String, queue_size=100)
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
    def start_seeker(self, locations):
        self._seeker_camera_subscriber = rospy.Subscriber(f'/robot_{self.ID}/image',Image, self._camera_callback, queue_size=1)
        self.is_seeker = True
        self.goals = locations
        self.seeker_publisher = rospy.Publisher(f'/robot_{self.ID}/seeker', String)
    """ initial pose callback function
    handles callback for initial pose subscriber
    """
    def _initial_pose_callback(self, pose):
        """ called when RViz sends a user supplied initial pose estimate """
        self._particle_filter.set_initial_pose(pose)
        self._last_published_pose = deepcopy(self._particle_filter.estimatedpose)
        self._initial_pose_received = True
        self._cloud_publisher.publish(self._particle_filter.particlecloud)
    
    """ odometry callback function
    handles callback for odometry

    Also where path finding + path following algorithms are called
    """
    def _odometry_callback(self, odometry):
        """
        Odometry received. If the filter is initialised then execute
        a filter predict step with odeometry followed by an update step using
        the latest laser, and attempt path finding/ path following
        """
        if self._initial_pose_received:
            pose = self._particle_filter.estimatedpose.pose.pose
            if self.is_seeker:
                if len(self.path) == 0:
                    if len(self.goals) > 0:
                        self.set_path(self.goals.pop(0))
                    else:
                        print ("End of game - Seeker lost")
                        quit()
                        sys.exit(0)

                    
            # path is a stack of nodes that the robot will attempt to reach. When no nodes are left in path, 
            # robot has reached destination
            if len(self.path) > 0:
                # get next node
                c = self.path[0]
                # move robot nearer node
                arrived = self.follow_path(pose, c)
                # arrived = if we have reached node
                if arrived:
                    # pop item off list
                    self.path = self.path[1:]
                    # publish path on rostopic - for RVIZ purposes
                    self.public_path()
                    # if we have reached target, publish on rostopic
                    if len(self.path) == 0:
                        self._robot_status.publish(f"{self.ID};arrived")

            # odometry code 
            t_odom = self._particle_filter.predict_from_odometry(odometry)
            t_filter = self._particle_filter.update_filter(self._latest_scan)
            

    """ set_path function
    args:
        self
        goal - tuple of coordinates (must be integer coords between {0, 620})
        sets the .path attribute to a list of pose coordinates which will direct
        the robot to its goal with no obstacles 
    """
    def set_path(self, goal):
        pose = self._particle_filter.estimatedpose.pose.pose
        if not self.path_made:
            # TODO: sanity check on goal values, convert appropriately
            self.path = navigator.generate_path(self.convert_map_coords(pose.position.x, pose.position.y), goal)  
            # convert index coords into pose coord system
            self.path = list(map(self.convert_coords, self.path))
            # path is currently goal to start, needs to be reversed
            self.path.reverse()        
            # publish path (for rviz)
            self.public_path()
    """ public path
    publishes path attribute to path topic, for visualisation
    """
    def public_path(self):#, path):
        pose_array = PoseArray()
        for p in self.path:
            pos = Pose()
            x,y = p
            pos.position.x = x
            pos.position.y = y
            pose_array.poses.append(pos)
        pose_array.header = self._particle_filter.particlecloud.header
        self.path_pub.publish(pose_array)

    """ laser callback function 
    callback function when robot receives laser data
    handles obstacle detection
    """
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
        # Obstacle detection
        """ Basic idea: independantly of followpath aglorithm, 
        we set the angle and whether or not the robot should reverse.
        We set .angle and .reverse to do this. 
        Note: these values are only used when the robot is already pointing towards node, and is ready to move
        """
        # helper script to find average of list
        avg = lambda x: sum(x)/len(x)
        #divide laser data into 5 evenly spaced sections
        split_data = self.split(data.ranges, 5)
        # laser data recieved is a semicircle infront of robot
        # each section of laser data (left, left-mid, middle, right-mid, right) are each one fifth of
        # full 180 degrees 
        l,lm,m,rm,r = split_data[0],split_data[1],split_data[2],split_data[3],split_data[4]
        # get minimum for rm, m and lm
        min_lm,min_m, min_rm = min(lm),min(m),min(rm)
        # get average of each section
        l,lm,m,rm,r = avg(l), avg(lm), avg(m), avg(rm), avg(r)
        # define some constants for what constitutes as "too close"
        LARGE_SPACE = 3
        CLOSE_SPACE = 0.4
        WALL_SPACE = 0.5

        # reverse boolean, when true, will make robot reverse
        self.reverse = (m < CLOSE_SPACE or lm < CLOSE_SPACE or 
        rm < CLOSE_SPACE)
        # if either of these if statements are true, so is self.reverse
        # lets robot swerve away from wall
        if lm < CLOSE_SPACE:
            self.angle = -1.57
        if rm < CLOSE_SPACE:
            self.angle = 1.57
        # if we are continuing forward..
        if not self.reverse:
            if r < WALL_SPACE:
                self.angle = -1.57/4
            elif l < WALL_SPACE:
                self.angle = 1.57/4
            else:
                self.angle = 0
    """split
    args:
        a - list
        n - nuimber of section
    returns: list
    simple function that divides a list into n parts evenly
    """
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
        return (location_delta > self._PUBLISH_DELTA or
                heading_delta > self._PUBLISH_DELTA)

    """compute_angle
    args:
        pose - pose to compare
        coords - coordinate tuple to compare
    returns: 
        angle between pose and coords
    this function compares pose to coordinates and finds the angle between them, normalised between {0 - 2pi}
    """
    def compute_angle(self, pose, coords):
        x, y = pose.position.x,pose.position.y 
        
        coords = round(coords[0],2), round(coords[1],2)
        angle = math.atan2((y -coords[1]), (x - coords[0]))
        normalise = lambda x : (x + (math.pi*2))%(math.pi*2)
        return angle
    
    """ follow_path
    args:
        pose - pose of robot
        path - tuple of next coordinate
    returns:
        boolean - True if pose ~= path, else False
    
    followpath turns or moves the robot towards the next node, making use of the 
    .angle and .reverse attributes to avoid obstacles
    returns True if the robot is sufficiently close to the node.
    """
    def follow_path(self, pose, path):
        # if we need to reverse, ignore any turn or movement towards node
        if self.reverse:
            self.move_robot(-0.5 , self.angle)
            return False
        # get angle to turn, and whether the robot should turn
        turn, angle = self.pointToTarget(self.compute_angle(pose, path), pose.orientation)
        # get distance to path, and whether robot is approximately arrived
        arrived, distance = self.approximate_same_position(pose, path)
        
        # if we havent already arrived, we should then check if we need to turn
        if not arrived:
            # first ensure we are pointing in the right direction
            if turn:
                self.move_robot(0 , angle)
            # then we can start marching forwards
            else:
                #accound for obstacles
                angle = self.angle
                self.move_robot(distance, angle)
        else:
            # We've arrived!
            return True
        # We havent arrived..
        return False
    """convert_coords
    args:
        coords - tuple of coordinates in the 620 index coordinate system used in path finding
    returns:
        tuple - coresponding coords in pose coordinate system
    """
    def convert_coords(self,coords):
        x,y = coords
        coords_l = 620
        map_l = 30.0
        c = coords_l/map_l
        t = 0.5
        return ((x/c)+ t,(y / c)+t)   

    """convert_map_coords
    args: 
        x, y - x and y index values of index coordinate system
    returns:
        tuple of coords in pose coordinate system
    """
    def convert_map_coords(self, x, y):
        coords_l = 620
        map_l = 30.0
        c = coords_l/map_l
        t = 0.5
        return (round(c * (x-t)), round(c * (y-t)))
    
    def move_robot(self, vel, angle):
        base_data = Twist()
        base_data.linear.x = vel
        base_data.angular.z = angle
        self._movement_publisher.publish(base_data)
     
    """ pointToTarget
    args: 
        angle - angle between position and desired location
        rot - Quartenion orientation of pose
    returns:

        if angle is ~= rot: 
            False, 0 
        else: 
            True, angular velocity
    This function converts Quartenions into euler angles, then computes the angle to turn to 
    match the robots orientation with the desired angle
    """
    def pointToTarget(self, angle, rot):
        # convert Quartenion to euler
        rotlist = [rot.x, rot.y, rot.z, rot.w]
        _,_,radian_z = euler_from_quaternion(rotlist)
        # adjust this angle to match same rotation system as "angle" parameter
        r = radian_z + math.pi
        # compute difference
        angle = (angle + (math.pi*2))%(math.pi*2)
        
        d = angle - r
        # compute other difference in angles
        other_d_abs = 2*math.pi - abs(d)
        
        if other_d_abs < abs(d):
            if d < 0:
                d = other_d_abs
            else:
                d = -other_d_abs

        approx_correct_angle = 0.4
        
        if d < approx_correct_angle and d > -approx_correct_angle:
            return False, 0
        return True, d
    """approximate_same_position
    args: 
        pose - position of robot
        coords - coords of node
    returns:
        if distance is small:
            True, 0
        else:
            False, move speed
    """
    def approximate_same_position(self, pose, coords):
        noise = 0.2
        x, y = pose.position.x, pose.position.y
        # compute distance between points
        d = math.dist([x, y], [coords[0],coords[1]])
        # if distance is sufficiently small
        if d < noise:
            return True, 0
        # if distance is particularly large, we can set a constant high movespeed
        large_distance = 1
        if large_distance < d:
            return False, 3
        # if the distance is small, the movespeed is proportional to the distance, allowing 
        # the robot to smoothly reach its goal without overshooting
        return False, d
    """camera callback
    handles robot detection
    """
    def _camera_callback(self, ros_data):
        

        rgb_image = CvBridge().imgmsg_to_cv2(ros_data, desired_encoding="bgr8")
        

        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
        
        # define range wanted color in HSV
        lower_val_red = np.array([0, 102, 0])
        upper_val_red = np.array([52, 255, 255])

        lower_val_purple = np.array([131, 167, 68])
        upper_val_purple = np.array([179, 224, 255])

        lower_val_green = np.array([52, 62, 0])
        upper_val_green = np.array([110, 255, 255])

        lower_val_blue = np.array([111, 50, 0])
        upper_val_blue = np.array([130, 255, 255])

        # Threshold the HSV image - any green color will show up as white
        mask_red = cv2.inRange(hsv, lower_val_red, upper_val_red)
        mask_purple = cv2.inRange(hsv, lower_val_purple, upper_val_purple)
        mask_green = cv2.inRange(hsv, lower_val_green, upper_val_green)
        mask_blue = cv2.inRange(hsv, lower_val_blue, upper_val_blue)
        # if there are any white pixels on mask, sum will be > 0
        hasred = np.sum(mask_red)
        haspurple = np.sum(mask_purple)
        hasgreen = np.sum(mask_green)
        hasblue = np.sum(mask_blue)
        found_str = "FOUND "
        msg = lambda x: f"{self.ID};{found_str}{x}"
        if hasred > 0:
            self.seeker_publisher.publish(msg("red"))

        if haspurple > 0:
            self.seeker_publisher.publish(msg("purple"))

        if hasgreen > 0:
            self.seeker_publisher.publish(msg("green"))

        if hasblue > 0:
            self.seeker_publisher.publish(msg("blue"))
         
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