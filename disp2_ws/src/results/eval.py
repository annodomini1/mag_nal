import numpy as np
import matplotlib.pyplot as plt


def get_no_tasks(times_data):
    no_tasks = []
    for robot in range(len(times_data)):
        no_tasks.append(len(times_data[robot]))
    return no_tasks


def sort_by_difficulty(task_times):
    easy_tasks = []
    medium_tasks = []
    hard_tasks = []
    for task in task_times:
        if task[1] == 0.0:
            hard_tasks.append(task)
        elif task[1] == 1.0:
            medium_tasks.append(task)
        else:
            easy_tasks.append(task)
    return easy_tasks, medium_tasks, hard_tasks


def get_avg_task_waiting_times(task_times):
    no_robots = len(task_times)
    avg_tct = []
    std_tct = []
    all_times = []
    for i in range(no_robots):
        ttr = []
        time_data = task_times[i]
        for data in time_data:
            diff = data[-2] - data[-3]
            ttr.append(diff)
            all_times.append(diff)
        std_tct.append(np.std(ttr))
        avg_tct.append(np.average(ttr))
    avg_all = np.average(all_times)
    return avg_tct, avg_all


def get_avg_task_completion_times(task_times):
    no_robots = len(task_times)
    avg_tct = []
    std_tct = []
    all_times = []
    for i in range(no_robots):
        ttr = []
        time_data = task_times[i]
        for data in time_data:
            diff = data[-1] - data[-2]
            ttr.append(diff)
            all_times.append(diff)
        std_tct.append(np.std(ttr))
        avg_tct.append(np.average(ttr))
    avg_all = np.average(all_times)
    return avg_tct, avg_all


def get_avg_full_task_times(task_times):
    no_robots = len(task_times)
    avg_tct = []
    std_tct = []
    all_times = []
    for i in range(no_robots):
        ttr = []
        time_data = task_times[i]
        for data in time_data:
            diff = data[-1] - data[-3]
            ttr.append(diff)
            all_times.append(diff)
        std_tct.append(np.std(ttr))
        avg_tct.append(np.average(ttr))
    avg_all = np.average(all_times)
    return avg_tct, avg_all

def seperate_tasks_finished(times_data):
    # seperate finished from not finished tasks
    no_robots = len(times_data)
    finished_times = [[] for i in range(no_robots)]
    not_finished_times = [[] for i in range(no_robots)]

    # seperate finished from not finished tasks
    for robot in range(no_robots):
        finished_for_robot = finished_times[robot]
        not_finished_for_robot = not_finished_times[robot]
        for sample in times_data[robot]:
            if sample[-1] == 0.0:
                not_finished_for_robot.append(sample)
            else:
                finished_for_robot.append(sample)
        finished_times[robot] = finished_for_robot
        not_finished_times[robot] = not_finished_for_robot

    return finished_times, not_finished_times


def combine_tasks_by_difficulty(finished_times):
    no_robots = len(finished_times)
    all_easy_times = np.empty((0, 5))
    all_medium_times = np.empty((0, 5))
    all_hard_times = np.empty((0, 5))
    for robot in range(no_robots):
        td = finished_times[robot]
        td_e, td_m, td_h = sort_by_difficulty(td)
        # print("robot " + str(robot))
        td_e = np.asarray(td_e)
        td_m = np.asarray(td_m)
        td_h = np.asarray(td_h)
        # print(td_e.shape)
        # print(td_m.shape)
        # print(td_h.shape)

        if td_e.shape[0] != 0:
            all_easy_times = np.append(all_easy_times, td_e, axis=0)
        if td_m.shape[0] != 0:
            all_medium_times = np.append(all_medium_times, td_m, axis=0)
        if td_h.shape[0] != 0:
            all_hard_times = np.append(all_hard_times, td_h, axis=0)
    return all_easy_times, all_medium_times, all_hard_times


def evaluate_times_by_stage(times, stage):
    # stage=0 -> 0-1 times
    # stage=1 -> 1-2 times
    # stage=2 -> 0-2 times
    task_times = []
    if stage == 0:
        s = 2
        e = 3
    elif stage == 1:
        s = 3
        e = 4
    else:
        s = 2
        e = 4

    for task in times:
        diff = task[e] - task[s]
        task_times.append(diff)
    return task_times


def get_task_times_by_robot(times_data):
    task_times = []
    for robot in range(len(times_data)):
        tt = times_data[robot]
        tt = evaluate_times_by_stage(tt, 1)
        tt_a = np.average(tt)
        task_times.append(tt_a)
    return task_times


def get_waiting_times(times_data): # dela
    no_robots = len(times_data)
    waiting_times = [[] for i in range(no_robots)]
    all_waiting_times = []

    for robot in range(no_robots):
        wtimes = []
        robot_times = times_data[robot]
        for i in range(len(robot_times) - 1):
            task_i = robot_times[i]
            task_ii = robot_times[i + 1]
            diff = task_ii[3] - task_i[4]
            wtimes.append(diff)
            all_waiting_times.append(diff)
        waiting_times[robot] = wtimes
    return waiting_times, all_waiting_times


def eval_dists(dists):
    no_robots = len(dists)
    all_dists = np.array([])
    for robot in range(no_robots):
        # print(len(dists[robot]))
        all_dists = np.concatenate((all_dists, dists[robot]))

    return all_dists


def eval_dist2(dists):
    dist = []

    no_robots = len(dists)
    for robot in range(no_robots):
        avg_dists = np.average(dists[robot])
        dist.append(avg_dists)

    return dist

def eval_agent_times(times_data):
    n_easy = []
    n_medium = []
    n_hard = []
    easy_0 = []
    easy_1 = []
    easy_all_0 = []
    easy_all_1 = []
    medium_0 = []
    medium_1 = []
    medium_all_0 = []
    medium_all_1 = []
    hard_0 = []
    hard_1 = []
    hard_all_0 = []
    hard_all_1 = []
    for j in range(len(times_data)):
        td = times_data[j]
        easy_times_0 = []
        easy_times_1 = []
        medium_times_0 = []
        medium_times_1 = []
        hard_times_0 = []
        hard_times_1 = []
        for task in td:
            if task[1] == 2.0:
                diff_0 = task[-1] - task[-3]
                diff_1 = task[-1] - task[-2]
                easy_times_0.append(diff_0)
                easy_times_1.append(diff_1)
                easy_all_0.append(diff_0)
                easy_all_1.append(diff_1)
            elif task[1] == 1.0:
                diff_0 = task[-1] - task[-3]
                diff_1 = task[-1] - task[-2]
                medium_times_0.append(diff_0)
                medium_times_1.append(diff_1)
                medium_all_0.append(diff_0)
                medium_all_1.append(diff_1)
            elif task[1] == 0.0:
                diff_0 = task[-1] - task[-3]
                diff_1 = task[-1] - task[-2]
                hard_times_0.append(diff_0)
                hard_times_1.append(diff_1)
                hard_all_0.append(diff_0)
                hard_all_1.append(diff_1)
            else:
                print("error")
        easy_0.append(np.average(easy_times_0))
        easy_1.append(np.average(easy_times_1))
        n_easy.append(len(easy_times_0))

        medium_0.append(np.average(medium_times_0))
        medium_1.append(np.average(medium_times_1))
        n_medium.append(len(medium_times_0))

        hard_0.append(np.average(hard_times_0))
        hard_1.append(np.average(hard_times_1))
        n_hard.append(len(hard_times_0))

    print("easy")
    print("0:", easy_0)
    print("1:", easy_1)
    print("num:", n_easy)
    print("avg 0:", np.average(easy_all_0))
    print("avg 1:", np.average(easy_all_1))

    print("medium")
    print("0:", medium_0)
    print("1:", medium_1)
    print("num:", n_medium)
    print("avg 0:", np.average(medium_all_0))
    print("avg 1:", np.average(medium_all_1))

    print("hard")
    print("0:", hard_0)
    print("1:", hard_1)
    print("num:", n_hard)
    print("avg 0:", np.average(hard_all_0))
    print("avg 1:", np.average(hard_all_1))


def get_12_times(times_data):
    times_12 = []
    for i in range(len(times_data)):
        td = times_data[i]
        dta = []
        for task in td:
            diff = task[-1] - task[-2]
            dta.append(diff)
        print(dta)
        times_12.append(np.average(dta))
    return times_12




def main():
    path = "/home/martin/Desktop/rezultati/results"
    disps = ["agent", "closest", "farthest", "locc", "mct"]
    disp = 4
    filepth_t = path + "/" + disps[disp] + "_t.npy"
    filepth_d = path + "/" + disps[disp] + "_d.npy"
    times_data = np.load(filepth_t, allow_pickle=True)
    dists_data = np.load(filepth_d, allow_pickle=True)
    # np.save("/home/martin/Desktop/rezultati/results/mct_t.npy", times_data)
    # np.save("/home/martin/Desktop/rezultati/results/mct_d.npy", dists_data)

    finished_times, not_finished_times = seperate_tasks_finished(times_data)

    all_easy_times, all_medium_times, all_hard_times = combine_tasks_by_difficulty(finished_times)

    easy_times_0 = evaluate_times_by_stage(all_easy_times, stage=0)
    easy_times_1 = evaluate_times_by_stage(all_easy_times, stage=1)
    easy_times_2 = evaluate_times_by_stage(all_easy_times, stage=2)

    medium_times_0 = evaluate_times_by_stage(all_medium_times, stage=0)
    medium_times_1 = evaluate_times_by_stage(all_medium_times, stage=1)
    medium_times_2 = evaluate_times_by_stage(all_medium_times, stage=2)

    hard_times_0 = evaluate_times_by_stage(all_hard_times, stage=0)
    hard_times_1 = evaluate_times_by_stage(all_hard_times, stage=1)
    hard_times_2 = evaluate_times_by_stage(all_hard_times, stage=2)
    print("no tasks completed by robot")
    print(get_no_tasks(finished_times))

    # print("easy tasks")
    # print(np.average(easy_times_0))
    # print(np.average(easy_times_1))
    # print(np.average(easy_times_2))
    # print("no easy:", len(easy_times_0))

    # print("medium tasks")
    # print(np.average(medium_times_0))
    # print(np.average(medium_times_1))
    # print(np.average(medium_times_2))
    # print("no medium:", len(medium_times_0))

    # print("hard tasks")
    # print(np.average(hard_times_0))
    # print(np.average(hard_times_1))
    # print(np.average(hard_times_2))
    # print("no hard:", len(hard_times_0))
    # plt.figure()
    # plt.plot(easy_times_1)
    # plt.show()

    print("waiting times")
    wtms, awtms = get_waiting_times(finished_times)
    print(np.average(wtms[0]))
    print(np.average(wtms[1]))
    print(np.average(wtms[2]))
    print(np.average(wtms[3]))
    print(np.average(awtms))

    print("distances")
    print(np.average(eval_dists(dists_data)))
    # print(np.std(eval_dists(dists_data)))
    print("distances by robot")
    print(eval_dist2(dists_data))

    # print("task times by robot")
    # print(get_avg_task_waiting_times(finished_times))
    # eval_agent_times(finished_times)

    print(get_avg_task_completion_times(finished_times))
    print(get_avg_full_task_times(finished_times))
    # print(get_12_times(finished_times))
    # print(finished_times[0][-1])

if __name__ == "__main__":
    main()
