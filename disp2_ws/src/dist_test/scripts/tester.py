#!/usr/bin/env python

from __future__ import division
import numpy as np
import time
import rospy
import dispenvlib as dl
import os
from taskgen.msg import Task


class TesterNode():
    # for each robot in a time period:
    # counts the number of completed tasks
    # times task completion
    # times waiting
    # calculates traveled distances

    def __init__(self, path):
        rospy.init_node('dists_node')
        print("dists node is up")
        time.sleep(1.0)
        
        self.no_robots = 4
        self.disptools = dl.DispatcherTools(self.no_robots)
        self.testing_time_period = 2 * 60
        self.dist_sample_interval = 0.2
        self.path = path
        self.distances = [[] for i in range(self.no_robots)]
        self.x_poses = [[] for i in range(self.no_robots)]
        self.y_poses = [[] for i in range(self.no_robots)]
        self.sampling_flag = np.zeros(self.no_robots)
        self.curr_stage = np.zeros(self.no_robots)

        rospy.Subscriber('/test_info', Task, self.info_analyzer)
        self.prev_msg = None
        self.new_msg = None
        self.robot_id = None
        self.task_stage = None
        rospy.Timer(rospy.Duration(self.dist_sample_interval), self.dist_sample)
        rospy.Timer(rospy.Duration(self.testing_time_period), self.end_of_test)
        print("ZACETEK TESTA - dists!")
        print(rospy.Time.now().to_sec())

    def end_of_test(self, event):
        print("KONC TESTA - dists!")
        print(rospy.Time.now().to_sec())
        # self.save_data(path)
        data = self.distances
        data = np.asarray(data)
        np.save(self.path, data)
        rospy.signal_shutdown("konc testa")

    def dist_sample(self, event):
        # print(self.task_stage, self.robot_id)
        for robot in range(self.no_robots):
            if self.curr_stage[robot] == 1.0 and self.sampling_flag[robot] == 0.0:
                # start sampling
                poses_x, poses_y = self.disptools.get_robot_poses()
                self.x_poses[robot].append(poses_x[robot])
                self.y_poses[robot].append(poses_y[robot])
                self.sampling_flag[robot] = 1.0
            elif self.curr_stage[robot] == 2.0 and self.sampling_flag[robot] == 1.0:
                # end sampling, calculate distance
                poses_x, poses_y = self.disptools.get_robot_poses()
                self.x_poses[robot].append(poses_x[robot])
                self.y_poses[robot].append(poses_y[robot])
                # calc distance
                dist = self.calc_dist(self.x_poses[robot], self.y_poses[robot])
                robot_distance = self.distances[robot]
                robot_distance.append(dist)
                self.distances[robot] = robot_distance
                # clear poses
                self.x_poses[robot] = []
                self.y_poses[robot] = []
                # set flag
                self.sampling_flag[robot] = 0.0
            else:
                pass
        # print(self.distances)

    def info_analyzer(self, msg):
        # msg = [task_id, task_stage, robot, task_level, time]
        self.new_msg = msg
        if self.new_msg != self.prev_msg:
            self.task_stage = msg.x_position # 1 - start, 2 - end
            self.robot_id = msg.y_position
            self.curr_stage[int(self.robot_id)] = self.task_stage
            self.prev_msg = self.new_msg
        else:
            pass

    def calc_dist(self, xposes, yposes):
        dist = 0
        if len(xposes) or len(yposes) > 1:
            for i in range(len(xposes) - 1):
                ddist = np.sqrt((xposes[i + 1] - xposes[i])**2 + (yposes[i + 1] - yposes[i])**2)
                dist += ddist
            return dist
        else:
            return 0


def main():
    test_name = 'test_new'
    path = '/home/martin/disp2_ws/src/dist_test/scripts/results/' + test_name + '.npy'
    tester = TesterNode(path=path)
    # tester.execute_test()
    rospy.spin()


if __name__ == "__main__":
    main()
