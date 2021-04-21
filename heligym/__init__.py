from gym.envs.registration import register
from heligym.envs import Helicopter

register(
    id='Heli-v0',
    entry_point='heligym.envs:Helicopter',
    max_episode_steps=5000,
    reward_threshold = 0.95,
    nondeterministic = False
)