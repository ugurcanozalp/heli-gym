import gym
import heligym
import numpy as np

env = heligym.HeliHover()
env.reset()

for i in range(1000000):
    print(i)

    _, _, done, _ = env.step(np.array([0.25, 0.5, 0.45, 0.35]))
    #done = False

    env.render()

    if done:
        #env.close()
        print('probleeem')
        break

print("hello world!")