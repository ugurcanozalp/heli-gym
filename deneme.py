import gym
import heligym
import numpy as np

env = gym.make("HeliHover-v0")
env.reset()

action = np.array([0.01, 0.5, 0.5, 0.01])

for i in range(100):
    obs, reward, info, done = env.step(action)
