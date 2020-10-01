#!/usr/bin/env python

from __future__ import division
from astar import AStar
from nav_msgs.msg import OccupancyGrid
import sys
import math
import cv2
import random
import rospy
import numpy as np
import matplotlib.pyplot as plt


class MazeSolver(AStar):

    """sample use of the astar algorithm. In this exemple we work on a maze made of ascii characters,
    and a 'node' is just a (x,y) tuple that represents a reachable position"""

    def __init__(self, maze):
        self.lines = maze.strip().split('\n')
        self.width = len(self.lines[0])
        self.height = len(self.lines)

    def heuristic_cost_estimate(self, n1, n2):
        """computes the 'direct' distance between two (x,y) tuples"""
        (x1, y1) = n1
        (x2, y2) = n2
        return math.hypot(x2 - x1, y2 - y1)

    def distance_between(self, n1, n2):
        """this method always returns 1, as two 'neighbors' are always adajcent"""
        return 1

    def neighbors(self, node):
        """ for a given coordinate in the maze, returns up to 4 adjacent(north,east,south,west)
            nodes that can be reached (=any adjacent coordinate that is not a wall)
        """
        x, y = node
        return[(nx, ny) for nx, ny in[(x, y - 1), (x, y + 1), (x - 1, y), (x + 1, y)]if 0 <= nx < self.width and 0 <= ny < self.height and self.lines[ny][nx] == '0']


class CoordSysTransf(object):
    """ This object is used to transform from cartesian to matrix coordinates
    and back.
    
    Arguments:
        scale_factor {[float]} -- Scale percentage of matrix coordiante system.
    """        

    def __init__(self, scale_factor=100):
        self.scale_factor = scale_factor
        map_topic = '/robot_0/move_base/global_costmap/costmap'
        msg = rospy.wait_for_message(map_topic, OccupancyGrid)
        self.resolution = msg.info.resolution
        # self.width = msg.info.width
        # self.height = msg.info.height
        self.off_x = (msg.info.origin.position.x) / self.resolution
        self.off_y = (msg.info.origin.position.y) / self.resolution
        
    def array_to_matrix_coord(self, poses):
        """ Transforms a tuple with two (x, y) arrays from cartesian to matrix
        coordiantes.
        
        Arguments:
            poses {[tuple]} -- Tuple with two arrays (x, y) in cartesian
            coordinates.
        
        Returns:
            [tuple] -- Tuple with two arrays (x, y) in matrix coordinates.
        """        
        x_poses, y_poses = poses
        x_poses_mat = np.array([])
        y_poses_mat = np.array([])
        for robot in range(poses[0].size):
            x_pos_mat = int((-self.off_x + x_poses[robot] / self.resolution) * (self.scale_factor / 100))
            y_pos_mat = int((-self.off_y - y_poses[robot] / self.resolution) * (self.scale_factor / 100))
            x_poses_mat = np.append(x_poses_mat, x_pos_mat)
            y_poses_mat = np.append(y_poses_mat, y_pos_mat)
        return x_poses_mat, y_poses_mat

    def point_to_matrix_coord(self, goal):
        """ Transforms a tuple, which represents a point in cartesian
        coordiantes into a point in matrix coordiantes.
        
        Arguments:
            goal {[tuple]} -- X, y point in cartesian coordiantes.
        
        Returns:
            [tuple] -- X, y point in matrix coordiantes.
        """        
        x_goal, y_goal = goal
        x_goal_mat = int((-self.off_x + x_goal / self.resolution) * (self.scale_factor / 100))
        y_goal_mat = int((-self.off_y - y_goal / self.resolution) * (self.scale_factor / 100))
        return x_goal_mat, y_goal_mat

    def array_to_cartesian_coord(self, poses_mat):
        """ Transforms a tuple with two (x, y) arrays from matrix to cartesian
        coordiantes.
        
        Arguments:
            poses {[tuple]} -- Tuple with two arrays (x, y) in matrix
            coordinates.
        
        Returns:
            [tuple] -- Tuple with two arrays (x, y) in cartesian coordinates.
        """       
        x_poses_mat, y_poses_mat = poses_mat
        x_poses = np.array([])
        y_poses = np.array([])
        for robot in range(poses_mat[0].size):
            x_pos = self.resolution * (x_poses_mat[robot] * (100 / self.scale_factor) + self.off_x)
            y_pos = self.resolution * (-y_poses_mat[robot] * (100 / self.scale_factor) - self.off_y)
            x_poses = np.append(x_poses, x_pos)
            y_poses = np.append(y_poses, y_pos)
        return x_poses, y_poses

    def point_to_cartesian_coord(self, goal_mat):
        """ Transforms a tuple, which represents a point in matrix
        coordiantes into a point in cartesian coordiantes.
        
        Arguments:
            goal {[tuple]} -- X, y point in matrix coordiantes.
        
        Returns:
            [tuple] -- X, y point in cartesian coordiantes.
        """   
        x_goal_mat, y_goal_mat = goal_mat
        x_goal = self.resolution * (x_goal_mat * (100 / self.scale_factor) + self.off_x)
        y_goal = self.resolution * (-y_goal_mat * (100 / self.scale_factor) - self.off_y)
        return x_goal, y_goal


class AstarPathGenerator(object):
    """ A* path generator.
    
    Arguments:
        scale_factor {[float]} -- Costmap resize percentage.
        threshold {[int]} -- Cost threshold value, between 0 and 100. Used 
        when creating a downscaled costmap.
        show_costmap_rsz {[bool]} -- True if you wish to inspect resized
        costmap.

    """
    def __init__(self, scale_factor, threshold, show_costmap_rsz=False):
        self.costmap = self.get_costmap(scale_factor, threshold)
        self.costmap_string = self.create_costmap_string(self.costmap)
        if show_costmap_rsz == True:
            plt.figure()
            plt.imshow(self.costmap, cmap='gray')
            plt.show()

    def get_costmap_size(self):
        """ Returns the dimensions of resized and thresholded costmap.
        
        Returns:
            [tuple] -- Rows, columns
        """        
        return self.costmap.shape

    def save_costmap(self, path):
        """ Saves costmap matrix as .npy in "path" direcotry.
        
        Arguments:
            path {[str]} -- Direcotry to save to.
        """        
        np.save(path, self.costmap)

    def get_costmap(self, scale_factor, threshold):
        """ Subscribes to ROS global costmap, resizes and thresholds it.
        
        Arguments:
            scale_factor {[float]} -- Costmap resize percentage.
            threshold {[int]} -- Cost threshold value, between 0 and 100. Used 
        when creating a downscaled costmap.
        
        Returns:
            [array] -- Resized and thresholded costmap.
        """        
        # get costmap
        print("Attention: A ros node must be created and an environment with working move_base must be eastablished.")
        map_topic = '/robot_0/move_base/global_costmap/costmap'
        msg = rospy.wait_for_message(map_topic, OccupancyGrid)
        width = msg.info.width
        height = msg.info.height
        data = msg.data
        costmap = np.reshape(data, (height, width))
        costmap = np.rot90(costmap, k=2)
        costmap = np.flip(costmap, axis=1)
        costmap = costmap.astype(np.float32)
        # resize
        # dims_rsz = (int(costmap.shape[0] * scale_factor / 100), int(costmap.shape[1] * scale_factor / 100)) !!!!!!
        dims_rsz = (int(costmap.shape[1] * scale_factor / 100), int(costmap.shape[0] * scale_factor / 100))
        costmap_rsz = cv2.resize(costmap, dims_rsz)
        # threshold
        for i in range(costmap_rsz.shape[0]):
            for j in range(costmap_rsz.shape[1]):
                if costmap_rsz[i, j] < threshold:
                    costmap_rsz[i, j] = 0
                else:
                    costmap_rsz[i, j] = 1
        costmap = costmap_rsz.astype(np.int8)
        return costmap

    def create_costmap_string(self, costmap_thr):
        """ Creates a string from an array. Used in A* algorithm.
        
        Arguments:
            costmap_thr {[array} -- Matrix array.
        
        Returns:
            [str] -- Matrix string.
        """        
        costmap_str = ''
        rows = costmap_thr.shape[0]
        cols = costmap_thr.shape[1]
        for r in range(rows):
            for c in range(cols):
                costmap_str = costmap_str + str(costmap_thr[r, c])
                if (c == cols - 1) and (r != rows - 1):
                    costmap_str = costmap_str + '\n'
                else:
                    pass
        return costmap_str

    def get_distances(self, goal, poses):
        """ Returns A* distances from robot positions to goal
        according to a costmap.
        
        Arguments:
            goal {[tuple]} -- X, y goal.
            poses {[tuple]} -- Tuple with two (x, y) arrays of positons.
        
        Returns:
            [array] -- Distances from positions to goal.
        """        
        distances = np.array([])
        x_goal, y_goal = goal
        goal = int(x_goal), int(y_goal)
        x_poses, y_poses = poses
        # print("poses:", poses)
        for i in range(len(x_poses)):
            x_pos = x_poses[i]
            y_pos = y_poses[i]
            start = int(x_pos), int(y_pos)
            dist = len(self.generate_astar_path(start, goal))
            distances = np.append(distances, dist)
        return distances

    def generate_astar_path_incl_robots(self, start, goal, (x_poses, y_poses)):
        costmap = np.copy(self.costmap)
        for robot in range(x_poses.size):
            if start != (int(x_poses[robot]), int(y_poses[robot])):
                costmap[int(y_poses[robot]), int(x_poses[robot])] = 1
            else:
                pass
        costmap_str = self.create_costmap_string_from_costmap(costmap)
        path = list(MazeSolver(costmap_str).astar(start, goal))
        return path

    def generate_astar_path(self, start, goal):
        path = list(MazeSolver(self.costmap_string).astar(start, goal))
        return path
    
    def get_astar_path_length(self, start, goal):
        path = list(MazeSolver(self.costmap_string).astar(start, goal))
        return len(path)

    def show_path(self, path, colour=[255, 0, 0]):
        costmap = self.costmap.astype(np.float32)
        rgb_image = cv2.cvtColor(costmap, cv2.COLOR_GRAY2RGB)

        plt.figure()
        for i in range(len(path) - 1):
            point = path[i]
            rgb_image[point[1], point[0], :] = colour
        point = path[-1]
        rgb_image[point[1], point[0]] = [0, 255, 0]
        plt.imshow(rgb_image)
        plt.show()
    
    def show_path_and_robots(self, path, poses, colour=[255, 0, 0]):
        x_poses, y_poses = poses
        costmap = self.costmap.astype(np.float32)
        rgb_image = cv2.cvtColor(costmap, cv2.COLOR_GRAY2RGB)

        plt.figure()
        for i in range(len(path) - 1):
            point = path[i]
            rgb_image[point[1], point[0], :] = colour
        point = path[-1]
        rgb_image[point[1], point[0]] = [0, 255, 0]

        for robot in range(x_poses.size):
            rgb_image[int(y_poses[robot]), int(x_poses[robot])] = [0, 0, 255]
        plt.imshow(rgb_image)
        plt.show()

    def show_robots(self, poses, colour=[255, 0, 0]):
        x_poses, y_poses = poses
        costmap = self.costmap.astype(np.float32)
        rgb_image = cv2.cvtColor(costmap, cv2.COLOR_GRAY2RGB)

        plt.figure()
        for robot in range(x_poses.size):
            rgb_image[int(y_poses[robot]), int(x_poses[robot])] = [0, 0, 255]
        plt.imshow(rgb_image)
        plt.show()

    def generate_random_goal(self, poses):
        x_poses, y_poses = poses
        while(1):
            x_goal = random.randint(0, self.costmap.shape[1] - 1)
            y_goal = random.randint(0, self.costmap.shape[0] - 1)
            if self.costmap[y_goal, x_goal] == 0:
                if np.any(x_poses != x_goal) and np.any(y_poses != y_goal):
                    return x_goal, y_goal
                else:
                    pass
            else:
                pass
            
    def init_robot_positions(self, no_robots):
        x_poses = np.array([])
        y_poses = np.array([])
        for robot in range(no_robots):
            while(1):
                x_pos = random.randint(0, self.costmap.shape[1] - 1)
                y_pos = random.randint(0, self.costmap.shape[0] - 1)
                if self.costmap[y_pos, x_pos] == 0:
                    x_poses = np.append(x_poses, x_pos)
                    y_poses = np.append(y_poses, y_pos)
                    break
                else:
                    pass
        return x_poses, y_poses
