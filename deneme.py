import gym
import heligym
import numpy as np

env = heligym.HeliHover()
env.reset(yaw_rate=15, ned_vel=np.array([100.0, 0, 0]))
action = env.heli_dyn.last_action #+ np.array([0.0, 0.1,0.0,0.0])
#print(env.heli_dyn.state)
print(action)
env.set_max_time(600)

for i in range(1000000):
    #print(i)

    _, _, done, info = env.step(action)
    #done = False
    #print(env.heli_dyn.state)

    env.render()

    if done:
        print(info)
        env.close()
        print('probleeem')
        break

#print("hello world!")