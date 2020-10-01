#!/usr/bin/env python

from __future__ import division
import random
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid


# class CoordSysTransf(object):
#     """ This object is used to transform from cartesian to matrix coordinates
#     and back.
    
#     Arguments:
#         scale_factor {[float]} -- Scale percentage of matrix coordiante system.
#     """        

#     def __init__(self, scale_factor=100):
#         self.scale_factor = scale_factor
#         map_topic = '/robot_0/move_base/global_costmap/costmap'
#         msg = rospy.wait_for_message(map_topic, OccupancyGrid)
#         self.resolution = msg.info.resolution
#         # self.width = msg.info.width
#         # self.height = msg.info.height
#         self.off_x = (msg.info.origin.position.x) / self.resolution
#         self.off_y = (msg.info.origin.position.y) / self.resolution
        
#     def array_to_matrix_coord(self, poses):
#         """ Transforms a tuple with two (x, y) arrays from cartesian to matrix
#         coordiantes.
        
#         Arguments:
#             poses {[tuple]} -- Tuple with two arrays (x, y) in cartesian
#             coordinates.
        
#         Returns:
#             [tuple] -- Tuple with two arrays (x, y) in matrix coordinates.
#         """        
#         x_poses, y_poses = poses
#         x_poses_mat = np.array([])
#         y_poses_mat = np.array([])
#         for robot in range(poses[0].size):
#             x_pos_mat = int((-self.off_x + x_poses[robot] / self.resolution) * (self.scale_factor / 100))
#             y_pos_mat = int((-self.off_y - y_poses[robot] / self.resolution) * (self.scale_factor / 100))
#             x_poses_mat = np.append(x_poses_mat, x_pos_mat)
#             y_poses_mat = np.append(y_poses_mat, y_pos_mat)
#         return x_poses_mat, y_poses_mat

#     def point_to_matrix_coord(self, goal):
#         """ Transforms a tuple, which represents a point in cartesian
#         coordiantes into a point in matrix coordiantes.
        
#         Arguments:
#             goal {[tuple]} -- X, y point in cartesian coordiantes.
        
#         Returns:
#             [tuple] -- X, y point in matrix coordiantes.
#         """        
#         x_goal, y_goal = goal
#         x_goal_mat = int((-self.off_x + x_goal / self.resolution) * (self.scale_factor / 100))
#         y_goal_mat = int((-self.off_y - y_goal / self.resolution) * (self.scale_factor / 100))
#         return x_goal_mat, y_goal_mat

#     def array_to_cartesian_coord(self, poses_mat):
#         """ Transforms a tuple with two (x, y) arrays from matrix to cartesian
#         coordiantes.
        
#         Arguments:
#             poses {[tuple]} -- Tuple with two arrays (x, y) in matrix
#             coordinates.
        
#         Returns:
#             [tuple] -- Tuple with two arrays (x, y) in cartesian coordinates.
#         """       
#         x_poses_mat, y_poses_mat = poses_mat
#         x_poses = np.array([])
#         y_poses = np.array([])
#         for robot in range(poses_mat[0].size):
#             x_pos = self.resolution * (x_poses_mat[robot] * (100 / self.scale_factor) + self.off_x)
#             y_pos = self.resolution * (-y_poses_mat[robot] * (100 / self.scale_factor) - self.off_y)
#             x_poses = np.append(x_poses, x_pos)
#             y_poses = np.append(y_poses, y_pos)
#         return x_poses, y_poses

#     def point_to_cartesian_coord(self, goal_mat):
#         """ Transforms a tuple, which represents a point in matrix
#         coordiantes into a point in cartesian coordiantes.
        
#         Arguments:
#             goal {[tuple]} -- X, y point in matrix coordiantes.
        
#         Returns:
#             [tuple] -- X, y point in cartesian coordiantes.
#         """   
#         x_goal_mat, y_goal_mat = goal_mat
#         x_goal = self.resolution * (x_goal_mat * (100 / self.scale_factor) + self.off_x)
#         y_goal = self.resolution * (-y_goal_mat * (100 / self.scale_factor) - self.off_y)
#         return x_goal, y_goal


class GoalGenerator(object):

    def __init__(self, scale_factor, threshold, show_costmap_rsz=False):
        self.scale_factor = scale_factor
        self.threshold = threshold
        # self.coord_transf = CoordSysTransf(self.scale_factor)
        self.get_costmap()
        print(self.costmap_rsz.shape)
        if show_costmap_rsz == True:
            plt.figure()
            plt.imshow(self.costmap_rsz, cmap='gray')
            plt.show()
        else:
            pass

    def save_costmap(self, path):
        np.save(path, self.costmap_rsz)

    def get_costmap(self):
        print("WARNING: A ros node must be created and an environment with working move_base must be eastablished.")
        map_topic = '/robot_0/move_base/global_costmap/costmap'
        msg = rospy.wait_for_message(map_topic, OccupancyGrid)
        width = msg.info.width # x
        height = msg.info.height # y
        data = msg.data
        self.res = msg.info.resolution
        self.x_limit = 0.9 * width * self.res / 2
        self.y_limit = 0.9 * height * self.res / 2
        self.off_x = (msg.info.origin.position.x) / self.res
        self.off_y = (msg.info.origin.position.y) / self.res
        # print(self.res, self.off_x, self.off_y)
        cmap = np.reshape(data, (height, width))
        cmap = np.rot90(cmap, k=2)
        cmap = np.flip(cmap, axis=1)
        cmap = cmap.astype(np.float32)
        # costmap full size
        self.costmap_full = cmap
        # plt.figure()
        # plt.imshow(self.costmap_full, cmap='gray')
        # plt.show()

        # costmap resize
        # resize
        # dims_rsz = (int(cmap.shape[0] * self.scale_factor / 100), int(cmap.shape[1] * self.scale_factor / 100))
        # x -> col, y -> row !!!!!
        dims_rsz = (int(cmap.shape[1] * self.scale_factor / 100), int(cmap.shape[0] * self.scale_factor / 100))
        costmap_rsz = cv2.resize(cmap, dims_rsz)
        # threshold
        for i in range(costmap_rsz.shape[0]):
            for j in range(costmap_rsz.shape[1]):
                if costmap_rsz[i, j] < self.threshold:
                    costmap_rsz[i, j] = 0
                else:
                    costmap_rsz[i, j] = 1
        self.costmap_rsz = costmap_rsz.astype(np.int8)

    def generate_random_goal(self):
        task_difficulty = random.choice([0, 1, 2])

        while(1):
            x_goal_f = random.uniform(-self.x_limit, self.x_limit)
            y_goal_f = random.uniform(-self.y_limit, self.y_limit)
            # transf to matrix full
            x_goal_ft = int((-self.off_x + x_goal_f / self.res))
            y_goal_ft = int((-self.off_y - y_goal_f / self.res))

            # resize check (to alleviate rounding error)
            x_goal_rsz_1 = int((-self.off_x + x_goal_f / self.res) * (self.scale_factor / 100))
            y_goal_rsz_1 = int((-self.off_y - y_goal_f / self.res) * (self.scale_factor / 100))
            # trans to cartesian
            x_goal_rsz_t = self.res * (x_goal_rsz_1 * (100 / self.scale_factor) + self.off_x)
            y_goal_rsz_t = self.res * (-y_goal_rsz_1 * (100 / self.scale_factor) - self.off_y)
            # trans to matrix
            x_goal_rsz_2 = int((-self.off_x + x_goal_rsz_t / self.res) * (self.scale_factor / 100))
            y_goal_rsz_2 = int((-self.off_y - y_goal_rsz_t / self.res) * (self.scale_factor / 100))
            if x_goal_rsz_1 == x_goal_rsz_2 and y_goal_rsz_1 == y_goal_rsz_2:
                if self.costmap_rsz[y_goal_rsz_1, x_goal_rsz_1] == 0 and self.costmap_full[y_goal_ft, x_goal_ft] == 0:
                    return x_goal_f, y_goal_f, task_difficulty
                else:
                    pass
            else:
                pass

