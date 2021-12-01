import math

from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point
from . pf_base import PFLocaliserBase
import rospy
import numpy as np
import os

from . util import rotateQuaternion, getHeading
from random import random, gauss
import time, statistics




class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        self.number_of_particles = 200
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # ----- Set motion model parameters
        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict


    def init_random_pose(self, mean_pose, sig):
        pose = Pose()
        pose.position = self.init_random_position(mean_pose.position, sig)
        pose.orientation = self.init_random_orientation(mean_pose.orientation, sig)
        return pose

    def init_random_position(self, mean_pos, sig):
        p = Point()
        p.x = gauss(mean_pos.x, sig)
        p.y = gauss(mean_pos.y, sig)
        return p
    def init_random_orientation(self,mean_ori, sig):
        q = Quaternion()
        q.z = gauss(mean_ori.z,sig)
        q.w = gauss(mean_ori.w, sig)
        return q
        
        
    
    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise

        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
        N = self.number_of_particles #number of particles
        sig =0.5  # sigma of noise gaussian
        pose_array = PoseArray()
        for i in range(N):
            pose_array.poses.append(self.init_random_pose(initialpose.pose.pose, sig))
        
       
        pose_array.header = initialpose.header
        
        
        return pose_array

    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
        
        desired_particles_num = self.number_of_particles
        weight_data = []
        # ----- Compute the likelihood weighting for each of a set of particles
        

        #self.particlecloud.poses.sort(key= lambda p: self.sensor_model.get_weight(scan, p) )
        for p in self.particlecloud.poses:
            weight_data.append(self.sensor_model.get_weight(scan, p))

        normalised_weight_data = self.normalise(weight_data)
        cdf = [normalised_weight_data[0]]
        
        for i in range(1,len(normalised_weight_data)):
            cdf.append(cdf[i-1] + normalised_weight_data[i])
        if statistics.mean(weight_data) > 4:
            desired_particles_num = 200
        else:
            desired_particles_num = 300
        threshold = random()* math.pow(desired_particles_num,-1)
        i=1
        new_particle_cloud = PoseArray()

        for j in range(0, desired_particles_num):
            while(threshold > cdf[i]):
                i+=1

                # make sigma prop to the inverse of the weight,

            if weight_data[i] < 4:
                #robot kidnapped/particle is far away from robot pos
                sigma = 3
            else:
                sigma = 1.1/weight_data[i]
            new_particle_cloud.poses.append(self.init_random_pose(self.particlecloud.poses[i],sigma))

            threshold = threshold + math.pow(desired_particles_num,-1)

        self.particlecloud = new_particle_cloud
        
    def normalise(self, _l):
        l = _l.copy()
        total = 0
        for i in range(len(l)):
            total +=l[i]
        
        for a in range(len(l)):
            l[a] = l[a]/ total
        return l

    
    
    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """

        # Implementing bottom-up hierarchical clustering

        """
        Computes the mean of a cluster
        :Args:
            | cluster ([geometry_msgs.msg.Pose]): an array of poses representing the cluster
        :Return:
            | (float, float, int) the mean of the x and y coordinates of each particules within the cluster plus the
            number of particules within the cluster.
        """
        def calculate_mean(cluster):
            mean_x = 0
            mean_y = 0
            for i in cluster:
                mean_x += i.position.x
                mean_y += i.position.y
            len_cluster = len(cluster)
            mean_x /= len_cluster
            mean_y /= len_cluster
            return mean_x, mean_y, len_cluster

        # Firstly, each pose is a cluster itself
        cluster_list = [] # list of clusters, a cluster is a list of poses [particule_1, ..., particule_n]
        cluster_data_list = [] # list of the means of the clusters [(mean_x, mean_y, len_cluster), ...]
        number_clusters = 0

        # Filling the lists
        for p in self.particlecloud.poses:
            cluster_list.append([p])
            cluster_data_list.append((p.position.x, p.position.y, 1))
            number_clusters += 1

        """
        Matrix stocking the distances between each and every clusters
        For n=5 clusters for instance:
                    0          1          2          3          4
        0  [       inf        inf        inf        inf        inf]
        1  [0.75692557        inf        inf        inf        inf]
        2  [1.21914767 1.11622158        inf        inf        inf]
        3  [1.59767441 1.15018727 1.20443125        inf        inf]
        4  [0.81266185 0.99565461 0.50851403 1.0390767         inf]
        Where distance between cluster 2 and 1 is 1.11622158 units.
        """
        distance_matrix = np.empty(shape=(number_clusters,number_clusters))
        # Initializing the matrix with the maximum float value which is handy for computing the lowest distance
        distance_matrix.fill(float('inf'))
        # Filling the matrix the distances between each and every clusters
        for i in range(number_clusters):
            for j in range(number_clusters):
                if i > j:
                    distance_matrix[i][j] = math.dist(cluster_data_list[i][:2], cluster_data_list[j][:2])

        # We keep merging the two nearest clusters until with have only 2 of them or the shortest distance between
        # them is long enough
        while number_clusters > 2:

            # Finding the two closest clusters in the matrix
            cluster_i = np.where(distance_matrix == np.amin(distance_matrix))[0][0]
            cluster_j = np.where(distance_matrix == np.amin(distance_matrix))[1][0]

            # If the distance between the two closest clusters is more than 1 unit we consider that our clusters are
            # good enough so we end the while loop
            if distance_matrix[cluster_i][cluster_j] > 1:
                break

            # We merge the two clusters together (by merging the cluster j into the cluster i
            cluster_list[cluster_i].extend(cluster_list[cluster_j])
            cluster_data_list[cluster_i] = calculate_mean(cluster_list[cluster_i])

            # We computes the new distances between the new formed cluster and all the other clusters
            for k in range(number_clusters):
                if cluster_i > k:
                    distance_matrix[cluster_i][k] = math.dist(cluster_data_list[cluster_i][:2], cluster_data_list[k][:2])
                elif k > cluster_i:
                    distance_matrix[k][cluster_i] = math.dist(cluster_data_list[cluster_i][:2], cluster_data_list[k][:2])

            # We delete the cluster j since he is now in the cluster i
            cluster_list.pop(cluster_j)
            cluster_data_list.pop(cluster_j)
            distance_matrix = np.delete(np.delete(distance_matrix, cluster_j, axis=0), cluster_j, axis=1)
            number_clusters -= 1

        # Computes the tallest cluster
        tallest_cluster_len = -42
        tallest_cluster_index = 0
        for c in range(len(cluster_data_list)):
            if cluster_data_list[c][2] > tallest_cluster_len:
                tallest_cluster_len = cluster_data_list[c][2]
                tallest_cluster_index = c

        # Computes the estimate pose
        pose = Pose()
        p = Point()
        p.x = cluster_data_list[tallest_cluster_index][0]
        p.y = cluster_data_list[tallest_cluster_index][1]

        tallest_cluster = cluster_list[tallest_cluster_index]
        sum_zr = sum_wr = 0
        for c in  tallest_cluster:
            sum_zr+= c.orientation.z
            sum_wr+= c.orientation.w
        q = Quaternion()
        len_tallest_cluster = len(tallest_cluster)
        q.z = sum_zr/len_tallest_cluster
        q.w = sum_wr/len_tallest_cluster
        pose.position = p
        pose.orientation = q

        return pose
