import gym
import heligym
import numpy as np

env = heligym.HeliHover()
env.reset()
action = env.heli_dyn.last_action + np.array([0.02, 0,0,0.04])
#print(env.heli_dyn.state)
print(action)
env.set_maxtime(600)

for i in range(1000000):
    #print(i)

    _, _, done, info = env.step(action)
    #done = False

    env.render()

    if done:
        print(info)
        env.close()
        print('probleeem')
        break

#print("hello world!")