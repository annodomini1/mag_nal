#!/usr/bin/env python

import rospy
import numpy as np
from taskgen.msg import Task
import time
import taskgenlib as tgl
import random


class TaskGenerator(object):

    def __init__(self, no_robots):
        rospy.init_node('taskgen_node')
        self.no_robots = no_robots
        self.pub_task = rospy.Publisher('/task', Task, queue_size=10)
        self.task = Task()

        scale_factor = 12.5
        self.goal_generator = tgl.GoalGenerator(scale_factor=scale_factor, threshold=99, show_costmap_rsz=False)
        path = '/home/martin/disp2_ws/src/taskgen/scripts/costmap.npy'
        self.goal_generator.save_costmap(path)
        time.sleep(1.0) # wait for node to init (rospy.Time could return 0):

    def publish_tasks(self, avg_time_btwn):
        task_id = 0
        while(1):
            time_btwn_sample = np.random.poisson(avg_time_btwn, 1)
            # order_time = time.time() + time_btwn_sample
            order_time = rospy.Time.now().to_sec() + time_btwn_sample
            while(1):
                # running_time = time.time()
                running_time = rospy.Time.now().to_sec()
                if running_time >= order_time:
                    # new order time
                    time_btwn_sample = np.random.poisson(avg_time_btwn, 1)
                    # order_time = time.time() + time_btwn_sample
                    current_time = rospy.Time.now().to_sec()
                    order_time = current_time + time_btwn_sample
                    # generate and publish goal
                    self.task.task_id = task_id
                    task_id += 1
                    # mean_deadline = current_time + const_time_part
                    # self.task.deadline = np.random.normal(mean_deadline, std_deviation)
                    goal = self.goal_generator.generate_random_goal()
                    self.task.x_position = goal[0]
                    self.task.y_position = goal[1]
                    self.task.z_orientation = random.uniform(0, 360)
                    self.task.deadline = goal[2]
                    self.pub_task.publish(self.task)
                    print("task " + str(task_id) + " sent!")
                    print("goal:", goal[0], goal[1])
                    print("time:", rospy.Time.now().to_sec())
                    break
                else:
                    pass


def main():
    tg = TaskGenerator(no_robots=4)
    tg.publish_tasks(avg_time_btwn=9)


if __name__ == "__main__":
    main()
