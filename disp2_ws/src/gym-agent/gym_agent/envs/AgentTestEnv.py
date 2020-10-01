#!/usr/bin/env python

from __future__ import division

import gym
import numpy as np
import dispenvlib as dl
import time
import cv2
import matplotlib.pyplot as plt
import AstarPath as ap
import rewards as rw

from gym import spaces
from gym.utils import seeding


class AgentTestEnv(gym.Env):

    def __init__(self):
        super(AgentTestEnv, self).__init__()
        costmap = np.load('/home/martin/disp2_ws/src/gym-agent/gym_agent/envs/costmap.npy')
        self.x_limit = costmap.shape[1] - 1 # array indexing
        self.y_limit = costmap.shape[0] - 1
        self.no_robots = 4
        self.task_id = 0
        self.tasks_buffer = dl.TaskBuffers(self.no_robots)
        self.astar = ap.AstarPathGenerator(costmap=costmap, scale_factor=12.5, threshold=99, show_costmap=False)
        self.max_occupancy = 5
        self.dl_levels = 3 # -> 3 levels of task importance ("deadline")
        # Action space, action -> robot_id
        self.action_space = spaces.Discrete(self.no_robots)
        # Observation space, observation -> [x_robots, x_goal, y_robots, y_goal, buffer_sizes, task_level]
        x_space_high = self.x_limit * np.ones(self.no_robots + 1) # +1 goal coordinate
        y_space_high = self.y_limit * np.ones(self.no_robots + 1)
        occ_low = np.zeros(self.no_robots * self.dl_levels)
        occ_high = np.ones(self.no_robots * self.dl_levels) * self.max_occupancy
        x_space_low = np.zeros_like(x_space_high)
        y_space_low = np.zeros_like(y_space_high)
        importance_low = np.zeros(self.dl_levels) # task importance flags, index 0 most important, last index least important
        importance_high = np.ones(self.dl_levels)
        low_space = np.concatenate((x_space_low, y_space_low, occ_low, importance_low))
        high_space = np.concatenate((x_space_high, y_space_high, occ_high, importance_high))
        self.observation_space = spaces.Box(low=low_space, high=high_space, dtype=np.float64)

    def step(self, action):
        # print(" ")
        # 1. action
        # 2. reward
        # 3. observation
        done = False

        # print("state:", self.state)

        # State unpacking -> for purposes of rewarding
        # x_goal, y_goal = self.state[self.no_robots], self.state[2 * self.no_robots + 1]
        # x_poses = self.state[0:self.no_robots]
        # y_poses = self.state[self.no_robots + 1:2 * self.no_robots + 1]
        x_poses = self.state[0:self.no_robots]
        y_poses = self.state[self.no_robots:2 * self.no_robots]
        x_goal = self.state[2*self.no_robots]
        y_goal = self.state[2*self.no_robots + 1]
        task_imp = self.state[2*self.no_robots + 2: 2*self.no_robots + 2 + self.dl_levels]
        occ = self.state[2*self.no_robots + 2 + self.dl_levels:]
        occ_0 = occ[:self.no_robots]
        occ_1 = occ[self.no_robots:2*self.no_robots]
        occ_2 = occ[2*self.no_robots:]
        occ_rew = [occ_0, occ_1, occ_2]
        for i in range(task_imp.size): # first index most important, last is least important
            if task_imp[i] == 1.0:
                dl_level = i
            else:
                pass

        # Action
        # robot_occ = self.tasks_buffer.get_weighted_buffer_sizes()
        task = np.array([self.task_id, x_goal, y_goal, 0.0, dl_level]) # ACTION EXECUTION
        self.tasks_buffer.add_task(action, task)
        # self.tasks_buffer.sort_buffers() # neumno

        goal = x_goal, y_goal
        poses = x_poses, y_poses
        distances = self.astar.get_distances(goal, poses)

        # print(" ")
        # print("dl level:", dl_level)
        # print("goal:", goal)
        # print("poses:", poses)
        # print("distances:", distances)
        # print("action: ", action)
        # print("occ rew", occ_rew)
        # print("state:", self.state)

        # Reward
        # 1. distance part
        rew_dist = rw.general_reward_continuous(action, distances, power=3)

        # 2. occupancy part

        rew_occ = rw.weighted_occupancy_reward(action, occ_rew[int(dl_level)], dl_level)

        rew = rew_dist + (rew_occ * 0.85)

        # print("reward:", rew)
        # time.sleep(5)

        # New observation
        # old (tko k bi dispatcher delu)
        last_task = self.tasks_buffer.check_last_task(action) # fifo logika
        new_x_poses = np.copy(x_poses)
        new_y_poses = np.copy(y_poses)
        new_x_poses[action] = last_task[1]
        new_y_poses[action] = last_task[2]
        new_poses = new_x_poses, new_y_poses

        new_x_goal, new_y_goal, new_task_level = self.astar.generate_random_goal(new_poses)
        new_task_imp = np.zeros(self.dl_levels)
        new_task_imp[new_task_level] = 1 # importance flag
        all_occs = self.tasks_buffer.get_buffer_sizes()
        # new_robot_occ = all_occs[int(new_task_level)]
        new_occ_0 = all_occs[0]
        new_occ_1 = all_occs[1]
        new_occ_2 = all_occs[2]
 
        self.state = np.concatenate((new_x_poses, new_y_poses, np.array([new_x_goal]), np.array([new_y_goal]), new_task_imp, new_occ_0, new_occ_1, new_occ_2))

        if self.tasks_buffer.get_max_occ() >= self.max_occupancy:
            done = True
        else:
            pass

        return self.state, rew, done, {}

    def reset(self):
        x_init_poses, y_init_poses = self.astar.init_robot_positions(self.no_robots)
        init_poses = x_init_poses, y_init_poses
        x_goal, y_goal, task_level = self.astar.generate_random_goal(init_poses)
        occ_init = np.zeros(self.no_robots * self.dl_levels)
        task_imp_init = np.zeros(self.dl_levels) # task importance init
        task_imp_init[int(task_level)] = 1 # flag representing task imporatence
        # self.state = np.concatenate((x_init_poses, np.array([x_goal]), y_init_poses, np.array([y_goal]), occ_init, task_imp_init))
        self.state = np.concatenate((x_init_poses, y_init_poses, np.array([x_goal]), np.array([y_goal]), task_imp_init, occ_init))
        self.tasks_buffer = dl.TaskBuffers(self.no_robots)
        self.task_id = 0
        # print("reset:", self.state)
        return self.state

    def render(self, mode='human', close=False):
        return
