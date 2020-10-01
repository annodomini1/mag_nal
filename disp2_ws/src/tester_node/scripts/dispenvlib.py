#!/usr/bin/env python

from __future__ import division
import numpy as np
import rospy
import time

from actionlib_msgs.msg import GoalStatusArray
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal
from tf.transformations import quaternion_from_euler


def distance_reward(action, distances):
    """ A function for evaluating action, which is based on distances from
    robots (robot_id == action) to goal. If closest robot is chosen, a
    reward is returned. In opposite case a punishment is returned.
    
    Arguments:
        action {[int]} -- Robot's id.
        distances {[array]} -- A* distances from robots to goal.
    
    Returns:
        [float] -- Reward or punishment, based on action.
    """    

    def index_by_size(distances):
        # bigger element gets a bigger number
        idxs = np.zeros_like(distances)
        arr = np.copy(distances)
        c = distances.size
        for i in range(distances.size):
            min_curr_idx = np.argmin(arr)
            arr[min_curr_idx] = 1000
            idxs[min_curr_idx] = c
            c -= 1
        return idxs

    idxs_array = index_by_size(distances)
    rews = np.array([-10, -2, 0, 2, 10]) # BUG: pozor pri temu arraju nared f(no_robots), interpolacija
    idxs_array = idxs_array - 1 # 0-4
    chosen_idx = int(idxs_array[action])
    return rews[chosen_idx]


def occupancy_reward(action, buffer_occupancy):
    """ A function for evaluating action based on the number of tasks a robot
    has. If robot with relatively large amount of tasks is chosen,
    punishment is returned.
    
    Arguments:
        action {[int]} -- Robot's id.
        buffer_occupancy {[array]} -- Array which indicates how many tasks
        each robot has.
    
    Returns:
        [float] -- Reward or punishment, based on action.
    """    
    occ = np.copy(buffer_occupancy)
    rel_occ = occ[action] / (1 + np.min(occ))
    if rel_occ > 1:
        # kazen
        return -(rel_occ**2)
    else:
        # nagrada
        return (1 / rel_occ)*2 ** 2


class TaskBuffers(object):
    # each robot has its own buffer
    """ This object contains buffers for each robot.

    Arguments:
        no_robots {[int]} -- Number of robots.

    """

    def __init__(self, no_robots):
        # data = [task_id, x_pos, y_pos, z_ori, deadline]
        self.no_robots = no_robots
        self.buffer_array = np.empty((0, 5))
        self.all_buffers = np.zeros(no_robots)
        self.all_buffers = self.all_buffers.tolist()
        self.prev_tasks = np.zeros(no_robots)
        self.prev_tasks = self.prev_tasks.tolist()
        self.buffer_array_hist = np.empty((0, 5))

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

    def add_task_history(self, robot_id, data):
        """ Add task to robot according to it's id.
        Stores all tasks.
        
        Arguments:
            robot_id {[int]} -- Robot's id.
            data {[array]} -- [task_id, x_goal, y_goal, z_orientation, deadline]
        """        
        if isinstance(self.prev_tasks[robot_id], np.float):
            individual_buffer = np.vstack((self.buffer_array_hist, data))
            self.prev_tasks[robot_id] = individual_buffer
        else:
            individual_buffer = self.prev_tasks[robot_id]
            individual_buffer = np.vstack((individual_buffer, data))
            self.prev_tasks[robot_id] = individual_buffer

    def get_first_task(self, robot_id):
        """ Returns first task in the individual robot buffer.
        Task is deleted.
        
           Arguments:
            robot_id {[int]} -- Robot's id.
        
        Returns:
            [array] -- [task_id, x_goal, y_goal, z_orientation, deadline]
        """     
        individual_buffer = self.all_buffers[robot_id]
        task = individual_buffer[-1]
        individual_buffer = np.delete(individual_buffer, -1, 0)
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
        return individual_buffer[-1]

    def get_last_task(self, robot_id):
        """ Returns last task in the individual robot buffer.
        Task is deleted.
        
           Arguments:
            robot_id {[int]} -- Robot's id.
        
        Returns:
            [array] -- [task_id, x_goal, y_goal, z_orientation, deadline]
        """     
        individual_buffer = self.all_buffers[robot_id]
        data = individual_buffer[0]
        individual_buffer = np.delete(individual_buffer, 0, 0)
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
        return individual_buffer[0]

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

    # def get_task(self, robot_id):
    #     # get first task (fifo)
    #     if isinstance(self.all_buffers[robot_id], np.ndarray):
    #         individual_buffer = self.all_buffers[robot_id]
    #         data = individual_buffer[0]
    #         individual_buffer = np.delete(individual_buffer, 0, 0)
    #         self.all_buffers[robot_id] = individual_buffer
    #         return data
    #     else:
    #         print("Robot " + str(robot_id) + " has an empty buffer.")
    #         return None

    # def get_task_by_index(self, robot_id, index):
    #     individual_buffer = self.all_buffers[robot_id]
    #     data = individual_buffer[index]
    #     individual_buffer = np.delete(individual_buffer, index, 0)
    #     self.all_buffers[robot_id] = individual_buffer
    #     return data

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

    def print_all_task_history(self):
        """ Prints the history of all buffers.
        """ 
        for robot in range(self.no_robots):
            print("Task history for robot: " + str(robot) + ":")
            print("Task ids:      X, Y goal:              Z orientation:   Deadline:")
            individual_buffer = self.prev_tasks[robot]
            if isinstance(individual_buffer, np.float) or individual_buffer == []:
                print("Buffer is empty for robot " + str(robot) + "!")
            else:
                for buff_row in range(individual_buffer.shape[0]):
                    task = individual_buffer[buff_row]
                    print("%d              %f, %f      %f         %f" % (int(task[0]), task[1], task[2], task[3], task[4]))

    def print_task_history_for_robot(self, robot_id):
        """ Prints robot's buffer history.
        
        Arguments:
            robot_id {[int]} -- Robot's id.
        """   
        individual_buffer = self.prev_tasks[robot_id]
        print("Task history for robot: " + str(robot_id) + ".:")
        print("Task ids:      X, Y goal:              Z orientation:   Deadline:")
        if isinstance(individual_buffer, np.float) or individual_buffer == []:
            print("Buffer is empty for robot " + str(robot_id) + "!")
        else:
            for buff_row in range(individual_buffer.shape[0]):
                task = individual_buffer[buff_row]
                print("%d              %f, %f      %f         %f" % (int(task[0]), task[1], task[2], task[3], task[4]))

    # def print_deadline_sums(self):
    #     deadline_sums = np.zeros(self.no_robots)
    #     for i in range(self.no_robots):
    #         if isinstance(self.all_buffers[i], np.ndarray):
    #             individual_buffer = self.all_buffers[i]
    #             # print("individual buffer:", individual_buffer)
    #             dl_sum = np.sum(individual_buffer[:, -1])
    #             deadline_sums[i] = dl_sum
    #         else:
    #             pass
    #     print("deadline sums:")
    #     print(deadline_sums)

    # def print_deadline_arrays(self):
    #     for i in range(self.no_robots):
    #         if isinstance(self.all_buffers[i], np.ndarray):
    #             individual_buffer = self.all_buffers[i]
    #             print("deadline array for robot: " + str(i))
    #             print(individual_buffer[:, -1])
    #         else:
    #             pass

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

    def get_buffer_sizes(self):
        """ Returns an array which indicates how many tasks are in individual
        robot's buffer.
        
        Returns:
            [array] -- Number of tasks in individual robot's buffer.
        """        
        buff_sizes = np.zeros(self.no_robots)
        for i in range(self.no_robots):
            if isinstance(self.all_buffers[i], np.ndarray):
                buff_sizes[i] = self.all_buffers[i].shape[0]
            else:
                pass
        return buff_sizes

    def check_task_by_deadline(self, robot_id, deadline):
        """ Returns task from robot's individual buffer. Returned task has a
        deadline that ends before the argumented deadline. If there is no such
        task, current active task is returned. Task is not deleted from the buffer.
        
        Arguments:
            robot_id {[int]} -- Robot's id.
            deadline {[float]} -- Deadline time.
        
        Returns:
            [array] -- [task_id, x_goal, y_goal, z_orientation, deadline]
        """        
        individual_buffer = self.all_buffers[robot_id]
        if individual_buffer.shape[0] == 1:
            task = self.check_last_task(robot_id)
            return task
        elif individual_buffer.shape[0] == 0:
            print("ERROR: buffer for robot " + str(robot_id) + " is empty!")
        else:
            all_buff_dls = individual_buffer[:, -1]
            task_idxs = np.where(all_buff_dls <= deadline)
            if task_idxs[0].size == 0:
                # no task has a deadline that is earlier
                # goal of active task is returned
                # print("no task has earlier deadline")
                # print("return goal of active task")
                task = individual_buffer[-1]
                # print("task:", task)
                return task
            else:
                relevant_task_idx = task_idxs[0][0]
                task = individual_buffer[relevant_task_idx, :]
                # print("task:", task)
                return task

    # def get_sum_of_available_task_exec_times(self, current_time):
    #     available_times_sum = np.zeros(self.no_robots)
    #     for i in range(self.no_robots):
    #         if isinstance(self.all_buffers[i], np.ndarray):
    #             individual_buffer = self.all_buffers[i]
    #             # print("individual buffer:", individual_buffer)
    #             avail_time_sum = np.sum(individual_buffer[:, -1] - current_time)
    #             available_times_sum[i] = avail_time_sum
    #         else:
    #             pass
    #     return available_times_sum


class TaskBuffer(object):
    """ This oject contains buffer for incoming tasks.
    Works on FIFO principle.

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

    def get_task(self):
        """ Returns first task from buffer. Task is deleted.
        
        Returns:
            [array] -- [task_id, x_goal, y_goal, z_orientation, deadline]
        """        
        task = self.buffer[0]
        self.buffer = np.delete(self.buffer, 0, 0)
        return task

    def check_task(self):
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

    def get_buffer(self):
        """ Returns buffer contents.
        
        Returns:
            [array] -- Buffer.
        """        
        return self.buffer

    def print_buffer(self):
        """ Prints buffer.
        """        
        print(self.buffer)

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

    def set_goal(self, robot_id, task, pub_msg):
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
                print("Goal set for robot " + str(robot_id) + ". Task id: " + str(int(task[0])) + ".")
                msg_str = "Goal set for robot " + str(robot_id) + ". Task id: " + str(int(task[0])) + ". Time: %s" % rospy.Time.now().to_sec()
                pub_msg.publish(msg_str)
                break
            else:
                pass

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
