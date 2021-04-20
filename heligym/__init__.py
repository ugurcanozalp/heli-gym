from gym.envs.registration import register

register(
    id='Heli-v0',
    entry_point='heligym.envs:Helicopter',
    max_episode_steps=5000,
    reward_threshold = 0.95,
    nondeterministic = False
)