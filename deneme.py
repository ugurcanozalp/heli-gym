import gym
import heligym
import numpy as np

env = heligym.HeliHover()
env.reset()
action = env.heli_dyn.last_action
for i in range(1000000):
    print(i)

    _, _, done, info = env.step(action)
    #done = False

    env.render()

    if done:
        print(info)
        env.close()
        print('probleeem')
        break

print("hello world!")