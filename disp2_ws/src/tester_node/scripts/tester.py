#!/usr/bin/env python

from __future__ import division
import numpy as np
import time
import rospy
import dispenvlib as dl
import os
from taskgen.msg import Task


class SampleBuffers(object):
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
        self.general_buffer = np.empty((0, 5))

    def get_all_buffers(self):
        return self.all_buffers

    def add_sample_general(self, data):
        self.general_buffer = np.vstack((self.general_buffer, data))
    
    def get_sample_general(self, task_id):
        task_ids = self.general_buffer[:, 0]
        idx = np.where(task_ids == task_id)
        idx = idx[0]
        sample = self.general_buffer[idx, :]
        self.general_buffer = np.delete(self.general_buffer, idx, 0)
        return sample

    def add_sample(self, robot_id, data):
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

    def check_sample_by_id(self, robot_id, sample_id):
        individual_buffer = self.all_buffers[robot_id]
        sample_ids = individual_buffer[:, 0]
        sample_idx = np.where(sample_ids == sample_id)
        if sample_idx[0].size == 0:
            print("sample buffer error")
        return individual_buffer[sample_idx[0]][0]
#         print(individual_buffer[sample_idx[0]][0])

    def modify_sample_by_id(self, robot_id, sample_id, elements=[], data=[]):
        individual_buffer = self.all_buffers[robot_id]
        sample_ids = individual_buffer[:, 0]
        sample_idx = np.where(sample_ids == sample_id)
        if sample_idx[0].size == 0:
            print("sample buffer error")

        sample = individual_buffer[sample_idx[0]][0]
        for i in range(len(elements)):
            ele = elements[i]
            dat = data[i]
            sample[ele] = dat

        individual_buffer[sample_idx[0]] = [sample]
        self.all_buffers[robot_id] = individual_buffer

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


class TesterNode():
    # for each robot in a time period:
    # counts the number of completed tasks
    # times task completion
    # times waiting
    # calculates traveled distances

    def __init__(self, path):
        rospy.init_node('tester_node')
        print("Tester node is up!")
        time.sleep(1.0)
        self.no_robots = 4
        self.testing_time_period = 2 * 60
        self.prev_msg = None
        self.new_msg = None
        # individual robots' sample -> [task_id, task level, buffer arrival time, task start time, task end time] # TODO: distance
        self.sample_buffers = SampleBuffers(self.no_robots)
        self.interval = 1.0
        self.path = path

        # Subs
        rospy.Subscriber('/test_info', Task, self.info_analyzer)
        # Timer
        rospy.Timer(rospy.Duration(self.testing_time_period), self.end_of_test)
        print("ZACETEK TESTA - times!")
        print(rospy.Time.now().to_sec())

    def end_of_test(self, event):
        print("KONC TESTA - times!")
        print(rospy.Time.now().to_sec())
        data = self.sample_buffers.get_all_buffers()
        data = np.asarray(data)
        np.save(self.path, data)
        rospy.signal_shutdown("konc testa")

    def info_analyzer(self, msg):
        # Task message is repurposed for gathering of the test info
        self.new_msg = msg
        if self.new_msg != self.prev_msg:
            self.sample_buffers.print_all_buffers()
            # mamo nov msg
            task_stage = msg.x_position
            robot_id = int(msg.y_position)
            task_id = msg.task_id
            if task_stage == 0.0:
                # task came in some robots' buffer
                # new sample entry
                # data = [task_id, task_level, buffer arrival time, 0, 0]
                data = [msg.task_id, msg.z_orientation, msg.deadline, 0, 0]
                # self.sample_buffers.add_sample(robot_id, data)
                self.sample_buffers.add_sample_general(data)
            elif task_stage == 1.0:
                # robot started a task
                data_s1 = self.sample_buffers.get_sample_general(task_id)
                self.sample_buffers.add_sample(robot_id, data_s1)
                self.sample_buffers.modify_sample_by_id(robot_id, task_id, elements=[3], data=[msg.deadline])
            elif task_stage == 2.0:
                # task is completed
                self.sample_buffers.modify_sample_by_id(robot_id, task_id, elements=[-1], data=[msg.deadline])
            else:
                print("some kind of error")
            # konc
            self.prev_msg = self.new_msg
        else:
            pass


def main():
    test_name = 'test_new'
    path = '/home/martin/disp2_ws/src/tester_node/scripts/results/' + test_name + '.npy'
    tester = TesterNode(path=path)
    rospy.spin()


if __name__ == "__main__":
    main()
