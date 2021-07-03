import gym
import heligym
import numpy as np
import time

env = heligym.HeliHover()
#env.set_max_time(1000)
env.reset()
action = env.heli_dyn.last_action + np.array([0.09, 0,0,0.04])

for i in range(1000000):

    _, r, done, info = env.step(action)
    #print(r)
    env.render()

    if done:
        print(info)
        env.close()
        print('Done!')
        break

