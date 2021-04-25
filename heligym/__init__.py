from gym.envs.registration import register
from heligym.envs import Heli, HeliHover

register(
    id='Heli-v0',
    entry_point='heligym.envs:Heli',
    max_episode_steps = 5000,
    reward_threshold  = 0.95,
    nondeterministic  = False
)

register(
    id='HeliHover-v0',
    entry_point='heligym.envs:HeliHover',
    max_episode_steps = 5000,
    reward_threshold  = 0.95,
    nondeterministic  = False
)
