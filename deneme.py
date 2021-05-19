import gym
import heligym
import numpy as np
import time

env = heligym.HeliHover()
env.reset()
env.set_max_time(600)
action = env.heli_dyn.last_action  # + np.array([0.02, 0,0,0.04])


for i in range(1000000):
    t0 = time.time()
    _, _, done, info = env.step(action)
    print(f"Step time: {time.time()-t0}")
    t0 = time.time()
    env.render()
    print(f"Elapsed time: {time.time()-t0}")

    if done:
        print(info)
        env.close()
        print('Done!')
        break

