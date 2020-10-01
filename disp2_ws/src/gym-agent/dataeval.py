# -*- coding: utf-8 -*-
from __future__ import unicode_literals
import numpy as np
import matplotlib.pyplot as plt
import json


def smoothTriangle(data, degree):
    triangle=np.concatenate((np.arange(degree + 1), np.arange(degree)[::-1])) # up then down
    smoothed=[]

    for i in range(degree, len(data) - degree * 2):
        point=data[i:i + len(triangle)] * triangle
        smoothed.append(np.sum(point)/np.sum(triangle))
    # Handle boundaries
    smoothed=[smoothed[0]]*int(degree + degree/2) + smoothed
    while len(smoothed) < len(data):
        smoothed.append(smoothed[-1])
    return smoothed


datadir = '/home/martin/disp2_ws/src/gym-agent/traindata_konc'


epi_rew = []
nb_steps = []
mse = []
loss = []
nb_episode_steps = []
duration = []
episode = []
mean_q = []

with open(datadir) as json_file:
    data = json.load(json_file)

    for d in data:
        print(d)

    for d in data['episode_reward']:
        epi_rew.append(d)

    for d in data['loss']:
        loss.append(d)

    for d in data['duration']:
        duration.append(d)

    for d in data['mean_q']:
        mean_q.append(d)

    for d in data['nb_episode_steps']:
        nb_episode_steps.append(d)

no_episodes = len(epi_rew)
print("no episodes:", )
print("epi rew")
print("epi_rew_min:", np.min(epi_rew))
print("epi_rew_max:", np.max(epi_rew))
print("epi_rew_avg:", np.average(epi_rew))
epi_rew_s = smoothTriangle(epi_rew, 60)
plt.figure(figsize=(15, 5))
plt.plot(epi_rew, label="Nagrada epizode")
plt.plot(epi_rew_s, label="Nagrada epizode - gladka")
plt.xlabel('Epizoda [/]', fontsize=15)
plt.ylabel('Nagrada epizode [/]', fontsize=15)
plt.xlim([0, no_episodes])
plt.xticks(fontsize=15)
plt.yticks(fontsize=15)
plt.grid()
# plt.legend(loc='upper left')
plt.show()

print("loss") #
loss_s = smoothTriangle(loss, 60)
plt.figure(figsize=(15, 5))
plt.plot(loss)
# plt.plot(loss_s)
plt.xlabel('Epizoda [/]', fontsize=15)
plt.ylabel('Vrednost kriterijske funkcije [/]', fontsize=15)
plt.xlim([0, no_episodes])
plt.xticks(fontsize=15)
plt.yticks(fontsize=15)
plt.grid()
plt.show()

print("duration")
print("duration_min:", np.min(duration))
print("duration_max:", np.max(duration))
print("duration_avg:", np.average(duration))
duration_s = smoothTriangle(duration, 60)
plt.figure(figsize=(15, 5))
plt.plot(duration)
plt.plot(duration_s)
plt.xlabel('Epizoda [/]', fontsize=15)
plt.ylabel('Čas [s]', fontsize=15)
plt.xlim([0, no_episodes])
plt.xticks(fontsize=15)
plt.yticks(fontsize=15)
plt.grid()
plt.show()

print("mean q")
print("avg mean q:", np.average(mean_q))
plt.figure(figsize=(15, 5))
plt.plot(mean_q)
plt.xlabel('Epizoda [/]', fontsize=15)
plt.ylabel('Srednja vrednost Q-vrednosti [/]', fontsize=15)
plt.xlim(no_episodes)
plt.xlim([0, no_episodes])
plt.xticks(fontsize=15)
plt.yticks(fontsize=15)
plt.grid()
plt.show()

print("stevilo opravljenih epizod:")
print(len(epi_rew))
