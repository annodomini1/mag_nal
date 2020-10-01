from gym.envs.registration import register
register(
    id='dispatcher-v0',
    entry_point='gym_dispatcher.envs:DispatcherEnv',
)
