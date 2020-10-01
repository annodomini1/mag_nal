#!/usr/bin/env python

from __future__ import division
import numpy as np
import time
import rospy
import dispenvlib as dl
import astarlib as al

from taskgen.msg import Task
from std_msgs.msg import String


class HeuristicDisp():

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
        
        scale_factor = 12.5
        self.astar = al.AstarPathGenerator(scale_factor=scale_factor, threshold=99, show_costmap_rsz=False)
        # print(self.astar.get_costmap_size())
        self.coord_transf = al.CoordSysTransf(scale_factor=scale_factor)
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

    def closest_selection(self):
        start_time = rospy.Time.now().to_sec()
        while(1):
            curr_time = rospy.Time.now().to_sec()
            if curr_time - start_time >= self.check_interval:
                start_time = curr_time
                self.execute_tasks()
                self.robot_buffers.print_all_buffers()
                # jedro noda
                if self.incoming_buffer.is_buffer_empty() == False:
                    # imamo cakajoce naloge

                    # ce so roboti prosti, pos -> pos robota
                    # ce niso, pos -> pos predhodne naloge
                    task = self.incoming_buffer.get_task()
                    new_task_dl = task[-1]
                    new_task_goal_c = task[1], task[2]
                    new_task_goal_m = self.coord_transf.point_to_matrix_coord(new_task_goal_c)

                    robot_poses_c = self.dltools.get_robot_poses()
                    robot_poses_m = self.coord_transf.array_to_matrix_coord(robot_poses_c)
                    robot_poses_x_m = robot_poses_m[0]
                    robot_poses_y_m = robot_poses_m[1]
                    for robot in range(self.no_robots):
                        if self.robot_buffers.is_buffer_empty_for_robot(robot) == False:
                            # relevant_task = self.robot_buffers.check_task_by_deadline(robot, new_task_dl)
                            relevant_task = self.robot_buffers.check_first_task(robot)
                            # print("Relevant task: " + str(relevant_task) + " for robot " + str(robot) + "!")
                            relevant_goal_c = relevant_task[1], relevant_task[2]
                            relevant_goal_m = self.coord_transf.point_to_matrix_coord(relevant_goal_c)
                            robot_poses_x_m[robot] = relevant_goal_m[0]
                            robot_poses_y_m[robot] = relevant_goal_m[1]
                        else:
                            pass
                    poses_m = robot_poses_x_m, robot_poses_y_m
                    distances = self.astar.get_distances(new_task_goal_m, poses_m)
                    # print("goal:", new_task_goal_m)
                    # print("poses:", poses_m)
                    # print("dists:", distances)
                    # izbira
                    robot_ids = np.where(distances == np.min(distances))
                    robot_id = robot_ids[0]
                    if robot_id.size > 1:
                        robot_id = np.random.choice(robot_id)
                    else:
                        robot_id = robot_id[0]

                    self.robot_buffers.add_task(robot_id, task)
                    self.publish_info(int(task[0]), 0, robot_id, task[-1]) # stage 0

                    print("_____________________________________________________________")
                    print("Robot " + str(robot_id) + " dobi nalogo " + str(int(task[0])) + "!")
                    print("_____________________________________________________________")


def main():
    heuristic_dispatcher = HeuristicDisp()
    heuristic_dispatcher.closest_selection()


if __name__ == "__main__":
    main()
