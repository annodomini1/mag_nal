#!/usr/bin/env python

import gym
import numpy as np
from gym_dispatcher.envs.DispatcherEnv import DispatcherEnv

from keras.models import Sequential
from keras.layers import Dense, Activation, Flatten
from keras.optimizers import Adam

from rl.agents.dqn import DQNAgent
from rl.policy import BoltzmannQPolicy
from rl.memory import SequentialMemory

ENV_NAME = 'dispatcher-v0'

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

memory = SequentialMemory(limit=5000, window_length=1)
# policy = BoltzmannQPolicy()
policy = BoltzmannQPolicy(tau=0)
dqn = DQNAgent(model=model, nb_actions=nb_actions, memory=memory, nb_steps_warmup=1000,
               target_model_update=1000, policy=policy)
dqn.compile(Adam(lr=1e-3), metrics=['mse'])

# training
# dqn.fit(env, nb_steps=300000, visualize=True, verbose=1)
# dqn.save_weights('/home/martin/Desktop/astar_gym/gym-agent/dqn_{}_weights_deep_rel_occ1.h5f'.format(ENV_NAME), overwrite=True)

# test
dqn.load_weights('/home/martin/disp2_ws/src/agent_dispatcher/scripts/dqn_agent_test-v0_weights_new_model.h5f')

# Finally, evaluate our algorithm for 5 episodes.
dqn.test(env, nb_episodes=1, visualize=True)