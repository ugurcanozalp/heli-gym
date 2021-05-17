import gym
import heligym
import numpy as np
import time

env = heligym.HeliHover()
env.reset()
action = env.heli_dyn.last_action + np.array([0.02, 0,0,0.04])


for i in range(1000000):

    _, _, done, info = env.step(action)
    env.render()

    if done:
        print(info)
        env.close()
        print('Done!')
        break

