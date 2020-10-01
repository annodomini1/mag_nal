#!/usr/bin/env python

from __future__ import division
import numpy as np
import rospy
import time

from actionlib_msgs.msg import GoalStatusArray
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal
from tf.transformations import quaternion_from_euler


class ActiveTaskBuffers():
    def __init__(self, no_robots):
        self.no_robots = no_robots
        self.buffers = [[] for _ in range(self.no_robots)]

    def add_task(self, robot_id, task):
        individual_buffer = self.buffers[robot_id]
        individual_buffer = task
        self.buffers[robot_id] = individual_buffer

    def get_task(self, robot_id):
        individual_buffer = self.buffers[robot_id]
        self.buffers[robot_id] = []
        return individual_buffer
    
    def get_buffer_sizes(self):
        # this also needs to be counted in when generating an observation
        states_hard = np.zeros(self.no_robots)
        states_medium = np.zeros(self.no_robots)
        states_easy = np.zeros(self.no_robots)
        for robot in range(self.no_robots):
            task = self.buffers[robot]
            if task != []:
                # not empty
                if task[-1] == 0.0:
                    states_hard[robot] = 1
                elif task[-1] == 1.0:
                    states_medium[robot] = 1
                elif task[-1] == 2.0:
                    states_easy[robot] = 1
                else:
                    print("weird error")
            else:
                pass
        return states_hard, states_medium, states_easy
    
    def is_buffer_empty_for_robot(self, robot_id):
        if self.buffers[robot_id] == []:
#             print("buffer empty")
            return True
        else:
#             print("not empty")
            return False

    def check_task(self, robot_id):
        return self.buffers[robot_id]
    
    def print_all_active_buffers(self):
        print("active buffers:")
        print("________________________________________________________")
        for robot in range(self.no_robots):
            print("Buffer for robot: " + str(robot) + ":")
            print("Task ids:      X, Y goal:              Z orientation:   Deadline:")
            individual_buffer = self.buffers[robot]
            if individual_buffer == []:
                print("Buffer is empty for robot " + str(robot) + "!")
            else:
                task = individual_buffer
                print("%d              %f, %f      %f         %f" % (int(task[0]), task[1], task[2], task[3], task[4]))
        print("________________________________________________________")


class TaskBuffers(object):
    # each robot has its own buffer
    """ This object contains buffers for each robot.

    Arguments:
        no_robots {[int]} -- Number of robots.

    """

    def __init__(self, no_robots):
        # data = [task_id, x_pos, y_pos, z_ori, deadline_level]
        # deadline_level -> 0 short deadline, 1 intermediate, 2 long deadline
        # deadline weights -> importance is represented as weights
        # short deadlines have higher weight
        
        self.no_robots = no_robots
        self.buffer_array = np.empty((0, 5))
        self.all_buffers = np.zeros(no_robots)
        self.all_buffers = self.all_buffers.tolist()
        self.prev_tasks = np.zeros(no_robots)
        self.prev_tasks = self.prev_tasks.tolist()
        self.task_levels = 3 # easy, medium, hard

    def add_task(self, robot_id, data):
        """ Add task to robot according to it's id.

        Arguments:
            robot_id {[int]} -- Robot's id
            data {[array]} -- [task_id, x_goal, y_goal, z_orientation, deadline]
        """
        if isinstance(self.all_buffers[robot_id], np.float): # mesto je prazno
            individual_buffer = np.vstack((self.buffer_array, data))
            self.all_buffers[robot_id] = individual_buffer
        else:
            individual_buffer = self.all_buffers[robot_id]
            individual_buffer = np.vstack((individual_buffer, data))
            self.all_buffers[robot_id] = individual_buffer

    def get_first_task(self, robot_id):
        """ Returns first task in the individual robot buffer.
        Task is deleted.
        
           Arguments:
            robot_id {[int]} -- Robot's id.
        
        Returns:
            [array] -- [task_id, x_goal, y_goal, z_orientation, deadline]
        """     
        individual_buffer = self.all_buffers[robot_id]
        task = individual_buffer[0]
        individual_buffer = np.delete(individual_buffer, 0, 0)
        self.all_buffers[robot_id] = individual_buffer
        return task

    def check_first_task(self, robot_id):
        """ Check first task info without deletion.
        
        Arguments:
            robot_id {[int]} -- Robot's id.
        
        Returns:
            [array] -- [task_id, x_goal, y_goal, z_orientation, deadline]
        """        
        individual_buffer = self.all_buffers[robot_id]
        return individual_buffer[0]

    def get_last_task(self, robot_id):
        """ Returns last task in the individual robot buffer.
        Task is deleted.
        
           Arguments:
            robot_id {[int]} -- Robot's id.
        
        Returns:
            [array] -- [task_id, x_goal, y_goal, z_orientation, deadline]
        """     
        individual_buffer = self.all_buffers[robot_id]
        data = individual_buffer[-1]
        individual_buffer = np.delete(individual_buffer, -1, 0)
        self.all_buffers[robot_id] = individual_buffer
        return data

    def check_last_task(self, robot_id):
        """ Check last task info without deletion.
        
        Arguments:
            robot_id {[int]} -- Robot's id.
        
        Returns:
            [array] -- [task_id, x_goal, y_goal, z_orientation, deadline]
        """  
        individual_buffer = self.all_buffers[robot_id]
        return individual_buffer[-1]

    def is_buffer_empty_for_robot(self, robot_id):
        """ For checking emptiness of robot's buffer.
        
        Arguments:
            robot_id {[int]} -- Robot's id.
        
        Returns:
            [bool] -- True if empty, False if not.
        """        
        if isinstance(self.all_buffers[robot_id], float):
            return True
        elif isinstance(self.all_buffers[robot_id], np.ndarray):
            if self.all_buffers[robot_id].size == 0:
                return True
            else:
                return False
        else:
            return False

    def delete_task_by_id(self, robot_id, task_id):
        """ Deletes task from robot's buffer by task id.
        
        Arguments:
            robot_id {[int]} -- Robot's id.
            task_id {[int]} -- Task id.
        """        
        individual_buffer = self.all_buffers[robot_id]
        task_ids = individual_buffer[:, 0]
        task_idx = np.where(task_ids == task_id)
        if task_idx[0].size == 0:
            print("ERROR: Task was already deleted of was never in the buffer.")
        individual_buffer = np.delete(individual_buffer, task_idx, 0)
        self.all_buffers[robot_id] = individual_buffer

    def print_all_buffers(self):
        """ Prints the contents of all buffers.
        """        
        print("waiting buffers:")
        print("________________________________________________________")
        for robot in range(self.no_robots):
            print("Buffer for robot: " + str(robot) + ":")
            print("Task ids:      X, Y goal:              Z orientation:   Deadline:")
            individual_buffer = self.all_buffers[robot]
            if isinstance(individual_buffer, np.float) or individual_buffer == []:
                print("Buffer is empty for robot " + str(robot) + "!")
            else:
                for buff_row in range(individual_buffer.shape[0]):
                    task = individual_buffer[buff_row]
                    print("%d              %f, %f      %f         %f" % (int(task[0]), task[1], task[2], task[3], task[4]))
        print("________________________________________________________")

    def print_buffer_for_robot(self, robot_id):
        """ Prints robot's buffer.
        
        Arguments:
            robot_id {[int]} -- Robot's id.
        """        
        individual_buffer = self.all_buffers[robot_id]
        print("Buffer for robot: " + str(robot_id) + ".:")
        print("Task ids:      X, Y goal:              Z orientation:   Deadline:")
        if isinstance(individual_buffer, np.float) or individual_buffer == []:
            print("Buffer is empty for robot " + str(robot_id) + "!")
        else:
            for buff_row in range(individual_buffer.shape[0]):
                task = individual_buffer[buff_row]
                print("%d              %f, %f      %f         %f" % (int(task[0]), task[1], task[2], task[3], task[4]))

    def are_buffers_empty(self):
        """ If all buffers are empty True is returned.
        
        Returns:
            [bool] -- True if empty, False if not.
        """        
        i = 0
        for i in range(self.no_robots):
            if self.is_buffer_empty_for_robot(i) is True:
                i += 1
            else:
                return False
        if i >= self.no_robots:
            return True
        else:
            pass

    def sort_buffers(self):
        for robot in range(self.no_robots):
            individual_buffer = self.all_buffers[robot]
            if isinstance(individual_buffer, np.ndarray):
                # buffer not empty
                # sort buffer (1. by level, 2. by "arrival time" (id))
                ind_buffer_sorted_lvl = individual_buffer[individual_buffer[:, -1].argsort()]
                ind_buffer_sorted = np.zeros_like(ind_buffer_sorted_lvl)
                for i in range(self.task_levels):
                    idxs = np.where(ind_buffer_sorted_lvl[:, -1] == i)
                    idxs = idxs[0]
                    if idxs.size > 1:
                        buffer_part = ind_buffer_sorted_lvl[idxs[0]:idxs[-1] + 1]
                        buffer_part_sorted = buffer_part[buffer_part[:, 0].argsort()]
                        ind_buffer_sorted[idxs[0]:idxs[-1] + 1] = buffer_part_sorted
                    elif idxs.size == 1:
                        # ni sortiranja po casu prihoda
                        buffer_part = ind_buffer_sorted_lvl[idxs[0]]
                        ind_buffer_sorted[idxs[0]] = buffer_part
                    else:
                        pass
                self.all_buffers[robot] = ind_buffer_sorted
            else:
                # buffer empty
                pass
    
    def get_buffer_sizes(self):
        occ_easy = np.zeros(self.no_robots) # task_level 2
        occ_medium = np.zeros(self.no_robots) # task_level 1
        occ_hard = np.zeros(self.no_robots) # taks_level 0
        for robot in range(self.no_robots):
            individual_buffer = self.all_buffers[robot]
            if isinstance(individual_buffer, np.ndarray):
                # buffer not empty
                task_lvls = individual_buffer[:, -1]
                task_lvl0_idx = np.where(task_lvls == 0.0)
                task_lvl1_idx = np.where(task_lvls == 1.0)
                task_lvl2_idx = np.where(task_lvls == 2.0)
                task_lvl0_num = len(task_lvl0_idx[0])
                task_lvl1_num = len(task_lvl1_idx[0])
                task_lvl2_num = len(task_lvl2_idx[0])
                occ_hard[robot] = task_lvl0_num
                occ_medium[robot] = task_lvl1_num
                occ_easy[robot] = task_lvl2_num
            else:
                # buffer empty
                pass
#                 print("buffer empty")
        return occ_hard, occ_medium, occ_easy

    def get_max_occ(self):
        occ0, occ1, occ2 = self.get_buffer_sizes()
        maxes = [np.max(occ0), np.max(occ1), np.max(occ2)]
        return np.max(maxes)

    def check_task_by_dl(self, robot_id, task_level):
        individual_buffer = self.all_buffers[robot_id]
        # print("indi buff", individual_buffer, task_level)
        task_levels = individual_buffer[:, -1]
        idxs = np.where(task_levels <= task_level)
        idxs = idxs[0]
        if idxs.size >= 1:
            return individual_buffer[idxs[-1]]
        elif idxs.size == 0:
            # print("error, no such tasks")
            # print("primerjas z aktivnim taskom")
            return []
    
    def check_reassignment_task(self):
        all_occs = self.get_buffer_sizes()
        for i in range(self.task_levels):
            if np.max(all_occs[i] > 0):
                idxs = np.where(all_occs[i] == np.max(all_occs[i]))
                idxs = idxs[0]
                # print("idxs", idxs)
                if idxs.size == 1:
                    return self.check_first_task(int(idxs)), int(idxs)
                elif idxs.size > 1:
                    # najd tistega, ki ima nizji id
                    tids = []
                    for robot in idxs:
                        task = self.check_first_task(robot)
                        tids.append(task[0])
                    min_tid_idx = np.where(tids == np.min(tids))
                    min_tid_idx = min_tid_idx[0]
                    chosen_robot_id = idxs[min_tid_idx]
                    return self.check_first_task(int(chosen_robot_id)), int(chosen_robot_id)
                elif i == self.task_levels - 1 and idxs.size == 0:
                    # print("no tasks for reassignment 1")
                    return [], None
                else:
                    pass
            elif i == self.task_levels - 1:
                # print("no tasks for reassignment 2")
                return [], None


class TaskBuffer(object):
    """ This class is a buffer for incoming tasks.

    """    

    def __init__(self):
        self.buffer = np.empty((0, 5))

    def add_task(self, task):
        """ Add task to buffer.
        
        Arguments:
            task {[array]} -- [task_id, x_goal, y_goal, z_orientation, deadline]
        
        Returns:
            [array] -- Buffer contents.
        """        
        self.buffer = np.vstack((self.buffer, task))
        return self.buffer

    def get_first_task(self):
        """ Returns first task from buffer. Task is deleted.
        
        Returns:
            [array] -- [task_id, x_goal, y_goal, z_orientation, deadline]
        """        
        task = self.buffer[0]
        self.buffer = np.delete(self.buffer, 0, 0)
        return task

    def check_first_task(self):
        """ Returns first task from buffer, without deletion.
        
        Returns:
            [array] -- [task_id, x_goal, y_goal, z_orientation, deadline]
        """        
        return self.buffer[0]

    def is_buffer_empty(self):
        """ For checking if buffer is empty.
        
        Returns:
            [bool] -- True if empty, False if not.
        """        
        if self.buffer.shape == (0, 5):
            return True
        else:
            return False
        
    def is_buffer_full(self, limit=10):
        """ For checking if buffer is empty.
        
        Returns:
            [bool] -- True if empty, False if not.
        """        
        if self.buffer.shape[0] >= limit:
            # self.sort_buffer()
            return True
        else:
            return False

    def get_buffer(self):
        """ Returns buffer contents.
        
        Returns:
            [array] -- Buffer.
        """        
        return self.buffer

    def print_buffer(self):
        """ Prints buffer contents.
        """        
        print(self.buffer)
        
    def sort_buffer(self):
        sorted_buffer_level = self.buffer[self.buffer[:,-1].argsort()] #first stage
        sorted_buffer = np.zeros_like(sorted_buffer_level)
        for i in range(3): # 3 levels of difficulty
            idxs = np.where(sorted_buffer_level[:, -1] == i)
            idxs = idxs[0]
            if idxs.size > 1:
                buffer_part = sorted_buffer_level[idxs[0]:idxs[-1] + 1]
                buffer_part_sorted = buffer_part[buffer_part[:, 0].argsort()]
                sorted_buffer[idxs[0]:idxs[-1] + 1] = buffer_part_sorted
            elif idxs.size == 1:
                # tuki ne rabis sortanja
                # print("ni sortanja")
                buffer_part = sorted_buffer_level[idxs[0]]
                sorted_buffer[idxs[0]] = buffer_part
            else:
                pass
        self.buffer = sorted_buffer

    def clear_buffer(self):
        self.buffer = np.empty((0, 5))

    def set_buffer(self, new_buffer):
        self.buffer = new_buffer

    # def delete_task_by_index(self, index):
    #     """ Deletes task by task index.
        
    #     Arguments:
    #         index {[int]} -- Tasks index.
    #     """        
    #     self.buffer = np.delete(self.buffer, index, 0)


class DispatcherTools(object):
    """ Tools needed for robot dispatching.
    
    Arguments:
        no_robots {[int]} -- Number of robots.
    """    

    def __init__(self, no_robots):
        self.no_robots = no_robots
        self.goal_pubs = self.create_set_goal_pubs()

    def create_set_goal_pubs(self):
        """ Creates a dictionary with ROS publisher objects used for 
        publishing goals.
        
        Returns:
            [dict] -- Contains goal publisher objects.
        """        
        pubs = {}
        for i in range(self.no_robots):
            pub_topic = '/robot_' + str(i) + '/move_base/goal'
            # pub_name = 'send_goal_robot_' + str(i) # key
            pub_name = str(i) # key
            pubs[pub_name] = rospy.Publisher(pub_topic, MoveBaseActionGoal, queue_size=10)
            # time.sleep(1.0)
        return pubs

    def get_robot_occupancy(self):
        """ Returns current robot occupancy. If robot isn't moving -> 0,
        if robot is moving -> 1.
        
        Returns:
            [array] -- Occupancy array.
        """        
        occupancy = np.zeros(self.no_robots)
        for i in range(self.no_robots):
            status_topic = '/robot_' + str(i) + '/move_base/status'
            msg = rospy.wait_for_message(status_topic, GoalStatusArray)
            msg_list = msg.status_list
            if msg_list == []:
                occupancy[i] = 0
            else:
                if len(msg_list) > 1:
                    robot_status = msg_list[-1].status
                else:
                    robot_status = msg_list[0].status

                if (robot_status == 1) or (robot_status == 0) or (robot_status == 7): # BUG pazi tuki je lahko se kaksna fora ker je teh statusov like 10
                    occupancy[i] = 1 # robot on move
                else:
                    occupancy[i] = 0 # robot on goal
        return occupancy

    def relative_occupancy(self, occupancy):
        # 1 -> relatively occupied
        # 1 -> relatively free
        rel_occ = np.zeros(occupancy.size)
        occ = np.copy(occupancy)
        occ = occ - np.min(occ)
        occ_max = float(np.max(occ))
        if occ_max == 0:
            return np.zeros(occupancy.size)
        
        for i in range(occ.size):
            rel_occ[i] = occ[i] / occ_max
        return rel_occ

    def set_goal(self, robot_id, task):
        """ Sets goal for a robot according to it's id.
        
        Arguments:
            robot_id {[int]} -- Robot's id.
            task {[array]} -- [task_id, x_goal, y_goal, z_orientation, deadline]
            pub_msg {[obj]} -- ROS object for publishing message strings.
        """        
        pub_names = self.goal_pubs.keys()
        pub_objs = self.goal_pubs.values()
        for i in range(len(pub_names)):
            if robot_id == int(pub_names[i]):
                Goal = MoveBaseActionGoal()
                Goal.header.stamp = rospy.Time.now()
                Goal.header.frame_id = ''
                Goal.goal_id.stamp = rospy.Time.now()
                Goal.goal_id.id = str(int(task[0]))
                Goal.goal.target_pose.header.stamp = rospy.Time.now()
                Goal.goal.target_pose.header.frame_id = 'map'
                Goal.goal.target_pose.pose.position.x = task[1]
                Goal.goal.target_pose.pose.position.y = task[2]
                z_rot_rad = task[3] * np.pi / 180
                q = quaternion_from_euler(0, 0, z_rot_rad)
                Goal.goal.target_pose.pose.orientation.z = q[2]
                Goal.goal.target_pose.pose.orientation.w = q[3]
                pub_obj = pub_objs[i]
                pub_obj.publish(Goal)
                break
            else:
                pass

    # def set_goal(self, robot_id, task, pub_msg):
    #     """ Sets goal for a robot according to it's id.
        
    #     Arguments:
    #         robot_id {[int]} -- Robot's id.
    #         task {[array]} -- [task_id, x_goal, y_goal, z_orientation, deadline]
    #         pub_msg {[obj]} -- ROS object for publishing message strings.
    #     """        
    #     pub_names = self.goal_pubs.keys()
    #     pub_objs = self.goal_pubs.values()
    #     for i in range(len(pub_names)):
    #         if robot_id == int(pub_names[i]):
    #             Goal = MoveBaseActionGoal()
    #             Goal.header.stamp = rospy.Time.now()
    #             Goal.header.frame_id = ''
    #             Goal.goal_id.stamp = rospy.Time.now()
    #             Goal.goal_id.id = str(int(task[0]))
    #             Goal.goal.target_pose.header.stamp = rospy.Time.now()
    #             Goal.goal.target_pose.header.frame_id = 'map'
    #             Goal.goal.target_pose.pose.position.x = task[1]
    #             Goal.goal.target_pose.pose.position.y = task[2]
    #             z_rot_rad = task[3] * np.pi / 180
    #             q = quaternion_from_euler(0, 0, z_rot_rad)
    #             Goal.goal.target_pose.pose.orientation.z = q[2]
    #             Goal.goal.target_pose.pose.orientation.w = q[3]
    #             pub_obj = pub_objs[i]
    #             pub_obj.publish(Goal)
    #             print("Goal set for robot " + str(robot_id) + ". Task id: " + str(int(task[0])) + ".")
    #             msg_str = "Goal set for robot " + str(robot_id) + ". Task id: " + str(int(task[0])) + ". Time: %s" % rospy.Time.now().to_sec()
    #             pub_msg.publish(msg_str)
    #             break
    #         else:
    #             pass

    def get_robot_poses(self):
        """ Returns a tuple which contains arrays current robot positions.
        
        Returns:
            [tuple] -- Array of x positions and array of y positions.
        """        
        x_poses = np.array([])
        y_poses = np.array([])
        for i in range(self.no_robots):
            odom_topic = '/robot_' + str(i) + '/odom'
            msg = rospy.wait_for_message(odom_topic, Odometry)
            x_pos = msg.pose.pose.position.x
            y_pos = msg.pose.pose.position.y
            x_poses = np.append(x_poses, x_pos)
            y_poses = np.append(y_poses, y_pos)
        return x_poses, y_poses
