#!/usr/bin/env python

from __future__ import division
import numpy as np
import random


def weighted_occupancy_reward(action, occupancy, task_level):
    def rew(num, interval=1, shift=0.5, power=1):
        # shift
        num = num - shift
        num = num * (interval / shift)
        if num >= 0: # spremenjen pogoj
            num = - num - 1
        else:
            num = abs(num) + 1
        # power
        if num < 0:
            num_pwr = abs(num) ** power
            num = -num_pwr
        elif num > 0:
            num = num ** power
        else:
            print("rew error")
        reward = num
        return reward
    
    occ = np.copy(occupancy)
    occ = occ - np.min(occ)
    occ_max = np.max(occ)
    if np.max(occ) == 0 and np.min(occ) == 0:
        return 0
    else:
        rel_occ = occ[action] / float(occ_max)
        if int(task_level) == 0: # pomembna naloga
            # velika kazen za dodelitev zasedenemu
            # velika nagrada za dodelitev prostemu
            return rew(rel_occ, power=3)
        elif int(task_level) == 1: # srednje pomembna
            # srednja kazen za dodelitev zasedenemu
            # srednja nagrada za dodelitev prostemu
            return rew(rel_occ, power=2)
        elif int(task_level) == 2: # nepomembna naloga
            # majhna kazen za dodelitev zasedenemu
            # nagrada kazen za dodelitev prostemu
            return rew(rel_occ, power=1)
        else:
            print("task level error")


def general_reward_discrete(action, distances, offset=1):
    
    def index_by_size(array):
        arr_cpy = np.copy(array)
        idxed_arr = np.zeros_like(array)
        idx = 0
        i = 0
        while(i < len(arr_cpy)):
            max_ele = np.where(arr_cpy == np.min(arr_cpy))
            max_ele = max_ele[0]
            if max_ele.size > 1:
                for j in range(max_ele.size):
                    idxed_arr[max_ele[j]] = idx
                    arr_cpy[max_ele[j]] = 1000
                i += max_ele.size
            else:
                idxed_arr[max_ele[0]] = idx
                arr_cpy[max_ele[0]] = 1000
                i += 1
            idx += 1
        return idxed_arr
    
    idxes = index_by_size(distances)
    # rewards = np.array([i for i in range(len(distances) - 1, len(distances) - 1 - len(distances), -1)]) - offset
    # rewards = [2, 1, -1, -2]
    rewards = [8, 2, -2, -8]
    idx = [i for i in range(len(distances))]
    for i in idx:
        if idxes[action] == i:
            return rewards[i]
        else:
            pass

def general_reward_continuous(action, observations, interval=1, shift=0.5, power=1):
    # treba je tko narest da se izognes nagradi okoli 0, power predznak
    obs = np.copy(observations)
    obs = obs - np.min(obs)
    obs_max = np.max(obs)
    if np.max(obs) == 0 and np.min(obs) == 0: # div w 0
        # print("rare event has occured")
        # return (interval + 1) ** power
        return 0
    else:
        rel_obs = obs[action] / float(obs_max) # 0-> good, 1-> bad
        rel_obs = rel_obs - shift
        rel_obs = rel_obs * (interval / shift)

        if rel_obs >= 0: # spremenjen pogoj iz > v >=, zaokrozeno navzdol
            rel_obs = -rel_obs - 1
        else:
            rel_obs = abs(rel_obs) + 1
        # power
        if rel_obs < 0:
            rlo_power = abs(rel_obs) ** power
            rel_obs = -rlo_power
        elif rel_obs > 0:
            rel_obs = rel_obs ** power
        else:
            print "error"
        reward = rel_obs
        return reward

def occupancy_reward_relative(action, occupancy):
    occ = np.copy(occupancy)
    occ = occ - np.min(occ)
    occ_max = np.max(occ)
    # print(occ)
    if np.max(occ) == 0 and np.max(occ) == 0: # div w 0
        return 8
    else:
        rel_occ = occ[action] / float(occ_max)

        if rel_occ >= 0.0 and rel_occ < 0.25:
            return 8
        elif rel_occ >= 0.25 and rel_occ < 0.5:
            return 2
        elif rel_occ >= 0.5 and rel_occ < 0.75:
            return -2
        elif rel_occ >= 0.75 and rel_occ <= 1.0:
            return -8
        else:
            return "error"

# old


def distance_reward(action, dist, shift_factor=1, power=2, reward_factor=1):
    dist_min = np.min(dist)
    dist_max = np.max(dist)
    shift = dist_min + ((dist_max - dist_min) / 2) * shift_factor
    dist_shifted = -dist + shift
    reward = dist_shifted[action] / np.max(dist_shifted)
    if reward > 0:
        return ((1 + reward) * reward_factor) ** power
    else:
        return -(((1 + abs(reward)) * reward_factor) ** power)

# rew_dist = distance_reward(action, distances, power=2, reward_factor=2)


def occupancy_reward(action, occupancy):
    occ = np.copy(occupancy)
    rel_occ = occ[action] / (1 + np.min(occ))
    if rel_occ > 1:
        # kazen
        return -(rel_occ**2)
    else:
        # nagrada
        return (1 / rel_occ)*2 ** 2

# robot_occ = self.tasks_buffer.get_buffer_sizes()
# # rew_occ = occupancy_reward(action, robot_occ)
# if self.steps == 0:
#     rew_occ = 0
# else:
#     rew_occ = occupancy_reward(action, robot_occ)



def main():

    no_robots = 5
    for i in range(10):
        # dists = array_of_rand_ints(no_robots, 50)
        dists = np.random.randint(low=0, high=50, size=no_robots)
        robot_id = np.random.randint(low=0, high=no_robots)
        rew = simple_distances_rew(robot_id, dists)
        print("dists:", dists)
        print("robot id:", robot_id)
        print("rew:", rew)




if __name__ == "__main__":
    main()
else:
    pass
