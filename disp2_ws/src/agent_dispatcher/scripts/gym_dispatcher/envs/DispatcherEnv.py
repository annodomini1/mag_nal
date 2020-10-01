#!/usr/bin/env python

import gym
import rospy
import numpy as np
import time
import actionlib
import cv2
import astarlib as al
import dispenvlib as dl

from gym import spaces
from gym.utils import seeding
from taskgen.msg import Task
from std_srvs.srv import Empty
from std_msgs.msg import Empty as empty
from std_msgs.msg import String


class DispatcherEnv(gym.Env):

    def __init__(self):
        super(DispatcherEnv, self).__init__()
        # Node initialisation
        rospy.init_node('dispatcher_env')
        time.sleep(1.0)

        # astar rabmo samo zarad observation space-a
        # (ne za ocenjevanje razdalj)
        scale_factor = 12.5
        self.astar = al.AstarPathGenerator(scale_factor=scale_factor, threshold=99, show_costmap_rsz=False)
        cmap_dims = self.astar.get_costmap_size()

        self.x_limit = cmap_dims[1] - 1
        self.y_limit = cmap_dims[0] - 1
        self.no_robots = 4
        self.prev_msg = None
        self.new_msg = None
        self.assignment_check_interval = 1.0
        self.reassignment_check_interval = 3.0
        self.prev_occ = np.zeros(self.no_robots)
        self.max_occupancy = 5
        self.dl_levels = 3 # 3 levels of task importance, 0 - most important, 2 - least important
        self.assignment_flag = 0 # 0 -> if assignment, 1 -> if reassignmnet
        self.reassignment_task = []
        self.reasignment_robot_id = None
        self.reas_sweep_id = 0
        self.sweep_done = True

        self.coord_transform = al.CoordSysTransf(scale_factor=scale_factor)
        self.in_task_buffer = dl.TaskBuffer() # buffer for incoming tasks
        self.robot_task_buffers = dl.TaskBuffers(self.no_robots) # robot buffers
        self.disp = dl.DispatcherTools(self.no_robots)
        self.active_task_buffer = dl.ActiveTaskBuffers(self.no_robots)

        # Action space, action -> chosen robot_id
        self.action_space = spaces.Discrete(self.no_robots)
        # Observation space, observation -> robot poses, goal pose, importnace flags, weighted buffer occupancies
        x_space_high = self.x_limit * np.ones(self.no_robots + 1) # +1 goal coordinate
        y_space_high = self.y_limit * np.ones(self.no_robots + 1)
        occ_low = np.zeros(self.no_robots * self.dl_levels)
        occ_high = np.ones(self.no_robots * self.dl_levels) * self.max_occupancy
        x_space_low = np.zeros_like(x_space_high)
        y_space_low = np.zeros_like(y_space_high)
        importance_low = np.zeros(self.dl_levels)
        importance_high = np.ones(self.dl_levels)
        low_space = np.concatenate((x_space_low, y_space_low, importance_low, occ_low))
        high_space = np.concatenate((x_space_high, y_space_high, importance_high,  occ_high))
        self.observation_space = spaces.Box(low=low_space, high=high_space, dtype=np.float64)

        # Publishers
        self.pub_reset = rospy.Publisher('/reset_time', empty, queue_size=1)
        self.pub_test_info = rospy.Publisher('/test_info', Task, queue_size=10)

        # Subscribers
        rospy.Subscriber('/task', Task, self.task_callback)

        # Services
        self.reset_simulation_proxy = rospy.ServiceProxy(
            '/gazebo/reset_world', Empty)
        self.pause_simulation_proxy = rospy.ServiceProxy(
            '/gazebo/pause_physics', Empty)
        self.unpause_simulation_proxy = rospy.ServiceProxy(
            '/gazebo/unpause_physics', Empty)

    # Auxilliary functions/Callbacks
    def simulation_reset(self):
        print("simulation reset")
        rospy.wait_for_service('/gazebo/reset_simulation')
        self.reset_simulation_proxy()
        self.pub_reset.publish(empty())
        time.sleep(1.0)

    def publish_info(self, task_id, stage, robot, task_level): # TODO: dodaj delaye
        info = Task() # task is repurposed for sharing test info
        info.task_id = task_id
        info.x_position = stage
        # stages:
        # 0 -> task is placed in robots' buffer
        # 1 -> task is started
        # 2 -> task is completed
        info.y_position = robot
        info.z_orientation = task_level
        info.deadline = float(rospy.Time.now().to_sec())
        self.pub_test_info.publish(info)
        rospy.sleep(0.3)

    def task_callback(self, msg):
        # tuki se fila vohdni buffer
        self.new_msg = msg
        if self.new_msg != self.prev_msg:
            task = np.array([msg.task_id, msg.x_position, msg.y_position, msg.z_orientation, msg.deadline])
            self.in_task_buffer.add_task(task)
            self.prev_msg = self.new_msg
        else:
            pass

    def execute_tasks(self):
        # loop za izvrsevanje nalog
        
        # najprej premik v aktivne bufferje
        # tu so pasivni bufferji ze sortani
        for robot in range(self.no_robots):
            if self.robot_task_buffers.is_buffer_empty_for_robot(robot) == False and \
                self.active_task_buffer.is_buffer_empty_for_robot(robot) == True:
                task = self.robot_task_buffers.get_first_task(robot)
                self.active_task_buffer.add_task(robot, task)
            else:
                pass

        # izvajanje nalog
        curr_occ = self.disp.get_robot_occupancy()
        for robot in range(self.no_robots):
            if self.active_task_buffer.is_buffer_empty_for_robot(robot) == False:
                # naloga se bo izvajala
                if curr_occ[robot] == 0 and self.prev_occ[robot] == 0:
                    # set goal
                    task = self.active_task_buffer.check_task(robot)
                    self.disp.set_goal(robot, task)
                    # pub test data
                    self.publish_info(int(task[0]), 1, robot, task[-1])
                    self.prev_occ[robot] = 1
                    curr_occ[robot] = 1
                elif curr_occ[robot] == 0 and self.prev_occ[robot] == 1:
                    # robot has reached the goal
                    # task is removed from active task buffer
                    task = self.active_task_buffer.get_task(robot)
                    self.publish_info(int(task[0]), 2, robot, 0)
                    self.prev_occ[robot] = 0
                else:
                    pass
            else:
                pass

    def next_observation(self, assignment):
        # assignment: 0 -> assignment, 1 -> reassignment
        state = np.zeros(2 * self.no_robots + 2) # 4 pozicije + cilj
        # new goal
        if assignment == 0:
            new_task = self.in_task_buffer.check_first_task()
            self.assignment_flag = 0
        else:
            new_task = self.robot_task_buffers.check_first_task(int(self.reasignment_robot_id))
            self.reassignment_task = new_task
            self.assignment_flag = 1
        # print("new task:", new_task)
        new_goal_c = new_task[1], new_task[2]
        new_task_level = new_task[-1]
        new_goal_m = self.coord_transform.point_to_matrix_coord(new_goal_c)
        # print("new goal m", new_goal_m)
        state[2 * self.no_robots] = new_goal_m[0]
        state[2 * self.no_robots + 1] = new_goal_m[1]
        task_imp = np.zeros(self.dl_levels)
        task_imp[int(new_task_level)] = 1 # task level flag
        state = np.concatenate((state, task_imp)) # pripnemo array flagov

        # new poses
        new_poses_c = self.disp.get_robot_poses()
        new_poses_m = self.coord_transform.array_to_matrix_coord(new_poses_c)
        new_x_poses_m, new_y_poses_m = new_poses_m
        for robot in range(self.no_robots):
            if self.robot_task_buffers.is_buffer_empty_for_robot(robot) == False and \
                self.active_task_buffer.is_buffer_empty_for_robot(robot) == False:
                # robot je v stanju opravljanja naloge ter ima naloge v pasivnem bufferju
                # observacija pozicije je pozicija naslednje naloge iste ali tezje tezavnosti
                task = self.robot_task_buffers.check_task_by_dl(robot, new_task_level) # BUG: razvrstit mors tezjo nalogo v bufferju pa le lazje
                if task == []:
                    task = self.active_task_buffer.check_task(robot)
                pos_c = task[1], task[2]
                pos_m = self.coord_transform.point_to_matrix_coord(pos_c)
                state[robot] = pos_m[0]
                state[self.no_robots + robot] = pos_m[1]
            elif self.robot_task_buffers.is_buffer_empty_for_robot(robot) == True and \
                self.active_task_buffer.is_buffer_empty_for_robot(robot) == False:
                # robot ima nalogo le v aktivnem bufferju
                # observacija pozicije je koncna lega trenutne naloge
                task = self.active_task_buffer.check_task(robot)
                pos_c = task[1], task[2]
                pos_m = self.coord_transform.point_to_matrix_coord(pos_c)
                state[robot] = pos_m[0]
                state[self.no_robots + robot] = pos_m[1]
            elif self.robot_task_buffers.is_buffer_empty_for_robot(robot) == True and \
                self.active_task_buffer.is_buffer_empty_for_robot(robot) == True:
                # robot je brez nalog
                # observacija je trenutna lega
                state[robot] = new_x_poses_m[robot]
                state[self.no_robots + robot] = new_y_poses_m[robot]
            elif self.robot_task_buffers.is_buffer_empty_for_robot(robot) == False and \
                self.active_task_buffer.is_buffer_empty_for_robot(robot) == True:
                # ta sictuacija se zgodi redko
                task = self.robot_task_buffers.check_task_by_dl(robot, new_task_level) # BUG: razvrstit mors tezjo nalogo v bufferju pa le lazje
                if task == []:
                    state[robot] = new_x_poses_m[robot]
                    state[self.no_robots + robot] = new_y_poses_m[robot]
                else:
                    pos_c = task[1], task[2]
                    pos_m = self.coord_transform.point_to_matrix_coord(pos_c)
                    state[robot] = pos_m[0]
                    state[self.no_robots + robot] = pos_m[1]
            else:
                pass
                # print("cudn bug - task_reassignment_obs")
                # print(self.robot_task_buffers.is_buffer_empty_for_robot(robot)) # False
                # print(self.active_task_buffer.is_buffer_empty_for_robot(robot)) # True
                # print(state)

        # current buffer occupancy
        new_occ_waiting = self.robot_task_buffers.get_buffer_sizes()
        new_occ_active = self.active_task_buffer.get_buffer_sizes() # active tasks must also be counted in
        new_occ_0 = new_occ_waiting[0] + new_occ_active[0] # hard tasks
        new_occ_1 = new_occ_waiting[1] + new_occ_active[1] # medium tasks
        new_occ_2 = new_occ_waiting[2] + new_occ_active[2] # easy tasks
        # new_occ_0 = np.zeros(self.no_robots)
        # new_occ_1 = np.zeros(self.no_robots)
        # new_occ_2 = np.zeros(self.no_robots)

        state = np.concatenate((state, new_occ_0, new_occ_1, new_occ_2))

        return state

    def step(self, action):
        # 1. executing action
        # 2. reward
        # 3. observation

        print("action:", action)

        done = False
        # Action
        if self.assignment_flag == 0:
            # task assignmnet
            task = self.in_task_buffer.get_first_task()
            self.robot_task_buffers.add_task(action, task)
            self.robot_task_buffers.sort_buffers()
            self.publish_info(int(task[0]), 0, action, task[-1]) # belezenje casa ko task pride v sistem
        else:
            if int(action) != int(self.reasignment_robot_id):
                print("REASSINGNMENT")
                print("From robot " + str(self.reasignment_robot_id) + " to robot " + str(action) + "!")
                self.robot_task_buffers.delete_task_by_id(int(self.reasignment_robot_id), self.reassignment_task[0])
                self.robot_task_buffers.add_task(action, self.reassignment_task)
                self.robot_task_buffers.sort_buffers()
            else:
                pass

        # Reward - not needed for execution
        rew = 0

        # New state observation
        if self.sweep_done == False:
            while(1):
                if self.reas_sweep_id < self.no_robots:

                    print("reassignment sweep")
                    print("reas sweep id:", self.reas_sweep_id)
                    # check for tasks
                    self.reasignment_robot_id = self.reas_sweep_id
                    self.reas_sweep_id += 1
                    if self.robot_task_buffers.is_buffer_empty_for_robot(int(self.reasignment_robot_id)) == False:
                        # create a reas obs
                        # state = self.task_reassignment_obs2()
                        state = self.next_observation(assignment=1)

                        return state, rew, done, {}
                    else:
                        pass
                else:
                    self.reas_sweep_id = 0
                    self.sweep_done = True
                    break

        start_time_a = rospy.Time.now().to_sec()
        start_time_r = rospy.Time.now().to_sec()
        if self.sweep_done == True:
            while(1):
                self.execute_tasks()
                if rospy.Time.now().to_sec() - start_time_a >= self.assignment_check_interval:
                    start_time_a = rospy.Time.now().to_sec()

                    # print("task for reassignemnt")
                    # print(self.robot_task_buffers.check_reassignment_task())

                    self.robot_task_buffers.print_all_buffers()
                    self.active_task_buffer.print_all_active_buffers()
                    if self.in_task_buffer.is_buffer_empty() == False:
                        # if assignment
                        # state = self.task_assignment_obs()
                        state = self.next_observation(assignment=0)
                        print("task assignment")
                        print("state:", state)
                        return state, rew, done, {}
                    else:
                        pass
                elif rospy.Time.now().to_sec() - start_time_r >= self.reassignment_check_interval:
                    start_time_r = rospy.Time.now().to_sec()

                    self.sweep_done = False
                else:
                    pass

    def reset(self):
        self.simulation_reset()
        self.prev_msg = None
        self.new_msg = None
        self.prev_occ = np.zeros(self.no_robots)
        self.in_task_buffer = dl.TaskBuffer()
        self.robot_task_buffers = dl.TaskBuffers(self.no_robots)
        # Reset observation
        init_poses_c = self.disp.get_robot_poses()
        init_poses_m = self.coord_transform.array_to_matrix_coord(init_poses_c)
        while(1):
            if self.in_task_buffer.is_buffer_empty() == False:
                task = self.in_task_buffer.check_first_task() # task is only checked, not taken from the buffer in "step"
                init_goal_c = task[1], task[2]
                init_task_level = task[-1]
                init_goal_m = self.coord_transform.point_to_matrix_coord(init_goal_c)
                init_occ = np.zeros(self.no_robots * self.dl_levels)
                init_task_imp = np.zeros(self.dl_levels)
                init_task_imp[int(init_task_level)] = 1
                state = np.concatenate((init_poses_m[0], init_poses_m[1], np.array([init_goal_m[0]]), np.array([init_goal_m[1]]), init_task_imp, init_occ))
                print("init goal m:", init_goal_m)
                print("state", state)
                return state
            else:
                pass

    def render(self, mode='human', close=False):
        return
