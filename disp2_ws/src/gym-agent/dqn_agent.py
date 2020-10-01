#!/usr/bin/env python

# UPORABNI VIRI:
# https://machinelearningmastery.com/how-to-configure-the-number-of-layers-and-nodes-in-a-neural-network/
# https://www.youtube.com/watch?v=Ya1gYt63o3M
# https://arxiv.org/pdf/1910.09281.pdf
# ZELO DOBRO:
# https://adgefficiency.com/dqn-tuning/
# https://machinelearningmastery.com/early-stopping-to-avoid-overtraining-neural-network-models/#:~:text=During%20training%2C%20the%20model%20is,the%20training%20process%20is%20stopped.

import numpy as np
import gym
from gym_agent.envs.AgentTestEnv import AgentTestEnv

from keras.models import Sequential
from keras.layers import Dense, Activation, Flatten
from keras.optimizers import Adam
from keras.utils.vis_utils import plot_model

from rl.agents.dqn import DQNAgent
from rl.policy import BoltzmannQPolicy
from rl.memory import SequentialMemory
from rl.callbacks import FileLogger

ENV_NAME = 'agent_test-v0'

env = gym.make(ENV_NAME)
np.random.seed(123)
env.seed(123)

nb_actions = env.action_space.n

model = Sequential()
model.add(Flatten(input_shape=(1,) + env.observation_space.shape))
model.add(Dense(35))
model.add(Activation('relu'))
model.add(Dense(35))
model.add(Activation('relu'))
model.add(Dense(nb_actions))
model.add(Activation('linear'))
plot_model(model, to_file='/home/martin/disp2_ws/src/gym-agent/model_plot.png', show_shapes=True, show_layer_names=True)

memory = SequentialMemory(limit=5000, window_length=1)
policy = BoltzmannQPolicy()
# policy = BoltzmannQPolicy(tau=0)
dqn = DQNAgent(model=model, nb_actions=nb_actions, memory=memory, nb_steps_warmup=10000,
               target_model_update=1000, policy=policy)
dqn.compile(Adam(lr=1e-3), metrics=['mse'])

# training
# logpath = '/home/martin/disp2_ws/src/gym-agent/traindata_konc'
# dqn.fit(env, nb_steps=1000000, visualize=True, verbose=1, callbacks=[FileLogger(filepath=logpath)]) # 500k za konvergenco
# dqn.save_weights('/home/martin/disp2_ws/src/gym-agent/dqn_{}_weights_logging.h5f'.format(ENV_NAME), overwrite=True)

dqn.load_weights('/home/martin/disp2_ws/src/gym-agent/dqn_agent_test-v0_weights.h5f')

dqn.test(env, nb_episodes=20, visualize=True)
