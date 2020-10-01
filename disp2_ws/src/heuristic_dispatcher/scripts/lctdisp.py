#!/usr/bin/env python

from __future__ import division
import numpy as np
import time
import rospy
import dispenvlib as dl
import random

from taskgen.msg import Task
from std_msgs.msg import String

# lct -> least cumulative time selection


class HeurDisp():

    def __init__(self):
        # node inti
        rospy.init_node('heur_disp_node')
        rospy.Subscriber('task', Task, self.taskgen_callback)
        self.pub_msg = rospy.Publisher('/disp_msg', String, queue_size=10)
        self.pub_test_info = rospy.Publisher('/test_info', Task, queue_size=10)
        # class init
        self.no_robots = 4
        self.new_msg = None
        self.prev_msg = None
        self.incoming_buffer = dl.TaskBuffer()
        self.robot_buffers = dl.TaskBuffers(self.no_robots)
        self.dltools = dl.DispatcherTools(self.no_robots)
        self.check_interval = 1.0
        self.prev_occ = np.zeros(self.no_robots)
        self.active_tasks = np.zeros(self.no_robots)
        self.cumulative_times = np.zeros(self.no_robots)
        self.time_recorded = np.zeros(self.no_robots) # 0 if not, 1 if is
        self.start_times = np.zeros(self.no_robots)
        self.end_times = np.zeros(self.no_robots)
        time.sleep(1.0)

    def publish_info(self, task_id, stage, robot, task_level):
        # for publishing test info
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

    def taskgen_callback(self, msg):
        self.new_msg = msg
        if self.new_msg != self.prev_msg:
            task = np.array([msg.task_id, msg.x_position, msg.y_position, msg.z_orientation, msg.deadline])
            self.incoming_buffer.add_task(task)
            self.prev_msg = self.new_msg
        else:
            pass

    def execute_tasks(self):
        curr_occ = self.dltools.get_robot_occupancy()
        for robot in range(self.no_robots):
            if self.robot_buffers.is_buffer_empty_for_robot(robot) == False:
                if curr_occ[robot] == 0 and self.prev_occ[robot] == 0:
                    task = self.robot_buffers.check_last_task(robot)
                    self.dltools.set_goal(robot, task, self.pub_msg)
                    self.publish_info(int(task[0]), 1, robot, task[-1]) # stage 1
                    self.active_tasks[robot] = task[0]
                    self.prev_occ[robot] = 1
                    curr_occ[robot] = 1
                elif curr_occ[robot] == 0 and self.prev_occ[robot] == 1:
                    task_id = self.active_tasks[robot]
                    self.publish_info(int(task_id), 2, robot, 0) # stage 2
                    self.robot_buffers.delete_task_by_id(robot, task_id)
                    self.prev_occ[robot] = 0
                    msg_str = "Goal achieved for robot " + str(robot) + ". Task id: " + str(int(task_id)) + ". Time: %s" % rospy.Time.now().to_sec()
                    self.pub_msg.publish(msg_str)
                else:
                    pass
            else:
                pass

    def add_waiting_times(self):
        curr_occ = self.dltools.get_robot_occupancy()
        min_time = np.min(self.cumulative_times)
        self.cumulative_times -= min_time # so that numbers don't add to infinity
        for robot in range(self.no_robots):
            if curr_occ[robot] == 0 and self.time_recorded[robot] == 0:
                # robot is waiting
                # start recording
                self.start_times[robot] = rospy.Time.now().to_sec()
                self.time_recorded[robot] = 1
            elif curr_occ[robot] == 1 and self.time_recorded[robot] == 1:
                # robot got the task and is no longer waiting
                self.time_recorded[robot] = 0
            elif curr_occ[robot] == 0 and self.time_recorded[robot] == 1:
                end_time = rospy.Time.now().to_sec()
                delta_time = end_time - self.start_times[robot]
                self.cumulative_times[robot] += delta_time
                self.start_times[robot] = end_time
            else:
                pass

    def lct_selection(self):
        start_time = rospy.Time.now().to_sec()
        while(1):
            self.add_waiting_times()
            curr_time = rospy.Time.now().to_sec()
            if curr_time - start_time >= self.check_interval:
                start_time = curr_time
                self.execute_tasks()
                self.robot_buffers.print_all_buffers()
                print("cumulative times:")
                print(self.cumulative_times)
                # jedro noda
                if self.incoming_buffer.is_buffer_empty() == False:
                    # imamo cakajoce naloge
                    # robot_id = np.random.randint(self.no_robots)
                    # task = self.incoming_buffer.get_task()
                    # self.robot_buffers.add_task(robot_id, task)
                    robot_ids = np.where(self.cumulative_times == np.min(self.cumulative_times))
                    # robot_id = robot_ids[0][0]
                    robot_id = random.choice(robot_ids[0])
                    print("robot ids, robot id", robot_ids, robot_id)
                    task = self.incoming_buffer.get_task()
                    self.robot_buffers.add_task(robot_id, task)
                    self.publish_info(int(task[0]), 0, robot_id, task[-1]) # stage 0

                    print("_____________________________________________________________")
                    print("Robot " + str(robot_id) + " dobi nalogo " + str(int(task[0])) + "!")
                    print("_____________________________________________________________")


def main():
    random_dispatcher = HeurDisp()
    random_dispatcher.lct_selection()


if __name__ == "__main__":
    main()
