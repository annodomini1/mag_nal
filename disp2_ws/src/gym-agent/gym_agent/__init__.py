from gym.envs.registration import register

register(
    id='agent_test-v0',
    entry_point='gym_agent.envs:AgentTestEnv',
)
