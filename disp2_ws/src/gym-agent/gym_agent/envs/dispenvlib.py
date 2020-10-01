#!/usr/bin/env python

import numpy as np
import rospy
import time

from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal


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


class PrevGoalsBuffer(object):
    # goal shape -> (1, 2)
    def __init__(self, max_buffer_size):
        self.buffer = np.empty((0, 2))
        self.max_buffer_size = max_buffer_size

    def add_goal(self, goal):
        if self.buffer.shape[0] < self.max_buffer_size:
            self.buffer = np.vstack((self.buffer, goal))
        else:
            self.buffer = np.delete(self.buffer, 0, 0)
            self.buffer = np.vstack((self.buffer, goal))
    
    def get_goal(self):
        # TODO elemente zacnes brisat ko je dosezen max_buffer_size
        goal = self.buffer[0]
        self.buffer = np.delete(self.buffer, 0, 0)
        return goal

    def get_buffer(self):
        return self.buffer
    
    def get_buffer_size(self):
        return self.buffer.shape[0]

    def is_buffer_full(self):
        if self.buffer.shape[0] == self.max_buffer_size:
            return True
        else:
            return False

    def print_buffer(self):
        print(self.buffer)


class TaskBuffer(object):
    # one buffer for all robots

    def __init__(self):
        self.buffer = np.empty((0, 5))

    def add_task(self, task):
        self.buffer = np.vstack((self.buffer, task))
        return self.buffer
    
    def get_task(self):
        task = self.buffer[0]
        self.buffer = np.delete(self.buffer, 0, 0)
        return task

    def check_task(self):
        # returns task without taking it out of the buffer
        return self.buffer[0]

    def is_buffer_empty(self):
        if self.buffer.shape == (0, 5):
            return True
        else:
            return False

    def get_buffer(self):
        return self.buffer

    def print_buffer(self):
        print(self.buffer)

    def delete_task_by_index(self, index):
        self.buffer = np.delete(self.buffer, index, 0)


class Occupancy(object):
    # occ_size -> no_robots
    # 1 -> occupied
    # 0 -> free
    def __init__(self, occupancy_size):
        self.occupancy = np.zeros(occupancy_size)

    def set_occupancy(self, robot_id):
        self.occupancy[robot_id] = 1

    def get_occupancy(self):
        return self.occupancy

    def check_occupancy(self):
        # False -> en ali vec robotov je prostih
        return self.occupancy.all()

    def check_occupancy_by_robot_id(self, robot_id):
        return self.occupancy[robot_id]
    
    def update_occupancy(self, task_buffer, margin=0.1):

        def no_active_robots(occupancy):
            cntr = 0
            for i in range(len(occupancy)):
                if occupancy[i] > 0:
                    cntr += 1
                else:
                    pass
            return cntr

        lar = no_active_robots(self.occupancy)
        # buff = task_buffer.get_buffer()
        # # print("pre buffer:", buff)
        # active_buff = buff[-lar:, :]
        # last_ids = active_buff[:, 0]
        # last_goals = active_buff[:, 1:3]

        for i in range(lar - 1):
            buff = task_buffer.get_buffer()
            # print("pre buffer:", buff)
            active_buff = buff[-lar:, :]
            last_ids = active_buff[:, 0]
            last_goals = active_buff[:, 1:3]

            rid = int(last_ids[i])
            x_pos, y_pos = get_robot_pose_by_id(rid)
            x_goal, y_goal = last_goals[i, 0], last_goals[i, 1]
            dist = np.sqrt((x_pos - x_goal)**2 + (y_pos - y_goal)**2)
            # print("rid:", rid, "dist:", dist)
            # print("goal:", x_goal, y_goal)
            # print("pos:", x_pos, y_pos)
            if dist <= margin:
                self.occupancy[rid] = 0
                # print("i:", i)
                # print("pre task buffer:")
                # task_buffer.print_buffer()
                task_buffer.delete_task_by_index(i)
                # print("post task buffer:")
                # task_buffer.print_buffer()
                print("Robot " + str(rid) + " je prispel na cilj.")
            else:
                pass
            # lar = no_active_robots(self.occupancy)
     
        return self.occupancy

def create_cancel_goal_pubs(no_robots):
    pubs = {}
    for i in range(no_robots):
        pub_topic = '/robot_' + str(i) + '/move_base/cancel'
        # pub_name = 'cancel_goal_robot_' + str(i) # key
        pub_name = 'cancel' + str(i)
        pubs[pub_name] = rospy.Publisher(pub_topic, GoalID, queue_size=10)
        # time.sleep(1.0)
    return pubs

def create_set_goal_pubs(no_robots):
    pubs = {}
    for i in range(no_robots):
        pub_topic = '/robot_' + str(i) + '/move_base/goal'
        # pub_name = 'send_goal_robot_' + str(i) # key
        pub_name = str(i) # key
        pubs[pub_name] = rospy.Publisher(pub_topic, MoveBaseActionGoal, queue_size=10)
        # time.sleep(1.0)
    return pubs

def create_set_goal_pubs2(no_robots):
    pubs = {}
    for i in range(no_robots):
        pub_topic = '/robot_' + str(i) + '/move_base_simple/goal'
        # pub_name = 'send_goal_robot_' + str(i) # key
        pub_name = str(i) # key
        pubs[pub_name] = rospy.Publisher(pub_topic, PoseStamped, queue_size=10)
        # time.sleep(1.0)
    return pubs

def get_last_goal_id(robot_id):
    status_topic = '/robot_' + str(robot_id) + '/move_base/status'
    msg = rospy.wait_for_message(status_topic, GoalStatusArray)
    msg_list = msg.status_list
    if msg_list == []:
        # print("no prior goals")
        return 0
    else:
        if len(msg_list) > 1:
            last_goal_id = msg_list[-1].goal_id.id
            return last_goal_id
        else:
            last_goal_id = msg_list[0].goal_id.id
            return last_goal_id

def get_goal_ids(robot_id):
    status_topic = '/robot_' + str(robot_id) + '/move_base/status'
    msg = rospy.wait_for_message(status_topic, GoalStatusArray)
    msg_list = msg.status_list
    goal_ids = np.array([])
    if msg_list == []:
        # print("no prior goals")
        return 0
    else:
        # print("len of msg list for robot" + str(robot_id))
        # print(len(msg_list))
        if len(msg_list) > 1:
            for i in range(len(msg_list)):
                goal_ids = np.append(goal_ids, msg_list[i].goal_id.id)
            return goal_ids
        else:
            goal_ids = np.append(goal_ids, msg_list[0].goal_id.id)
            return goal_ids

def cancel_all_goals(pubs):
    print("canceling all goals")
    pub_names = pubs.values() # list
    for i in range(len(pub_names)):
        goal_ids = get_goal_ids(i)
        pub_name = pub_names[i]
        if isinstance(goal_ids, np.ndarray):
            print("msg len:", len(goal_ids))
            if len(goal_ids) > 1:
                for j in range(len(goal_ids)):
                    Cancel = GoalID()
                    Cancel.stamp = rospy.Time.now()
                    Cancel.id = goal_ids[j]
                    # pub_name = pub_names[i]
                    pub_name.publish(Cancel)
                    # print("cancel:", Cancel)
                    time.sleep(0.2)
            else:      
                Cancel = GoalID()
                Cancel.stamp = rospy.Time.now()
                Cancel.id = goal_ids[0]
                # pub_name = pub_names[i]
                pub_name.publish(Cancel)
                # print("cancel:", Cancel)
                time.sleep(0.2)
            # print("goal canceled for robot with id " + last_goal_id)
        else:
            pass

def cancel_all_goals_simple(pubs): # TODO
    pub_names = pubs.values() # list
    for i in range(len(pub_names)):
        pub_name = pub_names[i]
        Cancel = GoalID()
        # Cancel.stamp = (0.0, 0.0)
        # Cancel.id = ''
        # pub_name = pub_names[i]
        pub_name.publish(Cancel)
        # print("cancel:", Cancel)
        time.sleep(0.2)


def cancel_all_goals2(pubs):
    print("canceling all goals")
    pub_names = pubs.values() # list
    for i in range(len(pub_names)):
        last_goal_id = get_last_goal_id(i)
        if last_goal_id != 0:
            Cancel = GoalID()
            Cancel.stamp = rospy.Time.now()
            Cancel.id = last_goal_id
            pub_name = pub_names[i]
            pub_name.publish(Cancel)
            print("goal canceled for robot with id " + last_goal_id)
        else:
            pass

def set_goal(robot_id, pos, pubs):
    pub_names = pubs.keys()
    pub_objs = pubs.values()
    for i in range(len(pub_names)):
        if robot_id == int(pub_names[i]):
            Goal = MoveBaseActionGoal()
            Goal.header.stamp = rospy.Time.now()
            Goal.header.frame_id = ''
            Goal.goal.target_pose.header.stamp = rospy.Time.now()
            Goal.goal.target_pose.header.frame_id = 'map'
            Goal.goal.target_pose.pose.position.x = pos[0]
            Goal.goal.target_pose.pose.position.y = pos[1]
            Goal.goal.target_pose.pose.orientation.w = 1.0
            pub_obj = pub_objs[i]
            pub_obj.publish(Goal)
            print("goal set for robot " + str(robot_id))
            break
        else:
            pass

def set_goal2(robot_id, pos, pubs):
    pub_names = pubs.keys()
    pub_objs = pubs.values()
    for i in range(len(pub_names)):
        if robot_id == int(pub_names[i]):
            Goal = PoseStamped()
            Goal.header.stamp = rospy.Time.now()
            Goal.header.frame_id = 'map'
            Goal.pose.position.x = pos[0]
            Goal.pose.position.y = pos[1]
            # dodaj z orientation
            Goal.pose.orientation.w = 1.0
            pub_obj = pub_objs[i]
            pub_obj.publish(Goal)
            print("goal set for robot " + str(robot_id))
            break
        else:
            pass

def get_robot_poses(no_robots):
    poses = np.empty((0, 2))
    for i in range(no_robots):
        odom_topic = '/robot_' + str(i) + '/odom'
        msg = rospy.wait_for_message(odom_topic, Odometry)
        x_pos = msg.pose.pose.position.x
        y_pos = msg.pose.pose.position.y
        pos = np.array([x_pos, y_pos])
        poses = np.vstack((poses, pos))
    return poses

def get_robot_pose_by_id(robot_id):
    odom_topic = '/robot_' + str(robot_id) + '/odom'
    msg = rospy.wait_for_message(odom_topic, Odometry)
    x_pos = msg.pose.pose.position.x
    y_pos = msg.pose.pose.position.y
    return x_pos, y_pos
    

def get_distances_from_goal(goal, poses):
    # poses - column vector (no_robots, 2)
    distances = np.array([])
    nor = poses.shape[0]
    for i in range(nor):
        dist = np.linalg.norm([goal, poses[i, :]])
        distances = np.append(distances, dist)
    return distances

def get_distance_from_goal(goal, pose):
    return np.linalg.norm([goal, pose])

def get_robot_occupancy(no_robots):
    # 0 - available, 1 - not available
    occupancy = np.zeros(no_robots)
    for i in range(no_robots):
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

def generate_goal(current_robot_poses, prev_goals_buffer, x_space=1.4, y_space=1.1, margin=0.1):
    x_space = x_space - margin
    y_space = y_space - margin
    if prev_goals_buffer.shape == (0, 2):
        poses = current_robot_poses
    else:
        poses = np.concatenate((current_robot_poses, prev_goals_buffer))
    # print("poses:")
    # print(poses)
    x_poses = poses[:, 0]
    y_poses = poses[:, 1]
    while(1):
        # first we find x that fits current robot poses and previous n goals
        x_pos = np.random.uniform(-x_space, x_space)
        ok_cntr = 0
        for i in range(len(x_poses)):
            x_robot = x_poses[i]
            x_cond = x_pos <= (x_robot - margin) or x_pos >= (x_robot + margin)
            if x_cond:
                ok_cntr += 1
            else:
                break
        if ok_cntr == len(x_poses):
            break
        else:
            pass
    while(1):
        # then we find y that fits current robot poses and previous n goals
        y_pos = np.random.uniform(-y_space, y_space)
        ok_cntr = 0
        for i in range(len(x_poses)):
            y_robot = y_poses[i]
            y_cond = y_pos <= (y_robot - margin) or y_pos >= (y_robot + margin)
            if y_cond:
                ok_cntr += 1
            else:
                break
        if ok_cntr == len(x_poses):
            break
        else:
            pass
    return x_pos, y_pos

def wait_for_robots_to_reach_the_goal(no_robots):
    # TODO: nared alternativo -> ko pridejo roboti v okolico svojih ciljev
    # atm se tuki zatika ucenje
    goal_reached = np.zeros(no_robots)
    while(1):
        time.sleep(1.0)
        for i in range(no_robots):
            status_topic = '/robot_' + str(i) + '/move_base/status'
            msg = rospy.wait_for_message(status_topic, GoalStatusArray)
            msg_list = msg.status_list
            if msg_list == []:
                goal_reached[i] = 0
            else:
                if len(msg_list) > 1:
                    robot_status = msg_list[-1].status
                else:
                    robot_status = msg_list[0].status
                if robot_status == 3:
                    goal_reached[i] = 1
        if goal_reached.all():
            break
            # return True
        else:
            pass

def wait_for_robots_to_reach_the_goal2(no_last_active_robots, all_prev_goals_buffer, margin=0.1):
    # TODO: iz zadnjega occupancy-ja se vid keri roboti so bli nazadnje aktivni
    prev_n_goals = all_prev_goals_buffer[-no_last_active_robots:, :]
    last_robot_ids = prev_n_goals[:, 0]
    last_robot_goals = prev_n_goals[:, 1:3]
    print("prev n goals:", prev_n_goals)
    print("last goal ids:", last_robot_ids)
    print("last robot goals:", last_robot_goals)
    ok_cntr = 0
    while(1):
        time.sleep(0.5)
        for i in range(len(last_robot_ids)):
            for j in range(len(last_robot_ids)):
                if last_robot_ids[j] == i:
                    x_pos, y_pos = get_robot_pose_by_id(i)
                    x_goal, y_goal = last_robot_goals[j, 0], last_robot_goals[j, 1]
                    dist = np.sqrt((x_pos - x_goal)**2 + (y_pos - y_goal)**2)
                    print("id:", i)
                    print("pos:", x_pos, y_pos)
                    print("goal:", x_goal, y_goal)
                    print("dist:", dist)
                    if dist < margin:
                        ok_cntr += 1
                    else:
                        pass
                else:
                    pass
        if ok_cntr == no_last_active_robots:
            break
        else:
            pass

def wait_for_robots_to_reach_the_goal3(no_robots, all_prev_goals_buffer, goal_pubs, margin=0.1):

    occ = get_robot_occupancy(no_robots)
    no_last_active_robots = get_num_of_last_active_robots(occ)
    prev_n_goals = all_prev_goals_buffer[-no_last_active_robots:, :]
    last_robot_ids = prev_n_goals[:, 0]
    last_robot_goals = prev_n_goals[:, 1:3]
    print("prev n goals:", prev_n_goals)
    print("last goal ids:", last_robot_ids)
    print("last robot goals:", last_robot_goals)
    
    while(1):
        ok_cntr = 0
        time.sleep(0.5)
        for i in range(len(last_robot_ids)):
            rid = int(last_robot_ids[i])
            x_pos, y_pos = get_robot_pose_by_id(rid)
            x_goal, y_goal = last_robot_goals[i, 0], last_robot_goals[i, 1]
            set_goal(rid, (x_goal, y_goal), goal_pubs) # BUG: dodano
            dist = np.sqrt((x_pos - x_goal)**2 + (y_pos - y_goal)**2)
            print("id:", i)
            print("rid:", rid)
            print("pos:", x_pos, y_pos)
            print("goal:", x_goal, y_goal)
            print("dist:", dist)
            if dist < margin:
                ok_cntr += 1
            else:
                pass
            print("ok:", ok_cntr)
            if ok_cntr == no_last_active_robots:
                return 0
            else:
                pass

def get_num_of_last_active_robots(occupancy):
    last_active_robots = 0
    no_robots = len(occupancy)
    for i in range(no_robots):
        if occupancy[i] == 1:
            last_active_robots += 1
        else:
            pass
    return last_active_robots

def relative_occupancy(occupancy):
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
