"""
Helicopter control.
"""
import yaml
import pprint
from typing import List
import sys, math
import numpy as np
import os

import gym
from gym import spaces
from gym.utils import seeding, EzPickle

from .dynamics import HelicopterDynamics

FPS         = 100.0
DT          = 1/FPS
FTS2KNOT    = 0.5924838; # ft/s to knots conversion
EPS         = 1e-10; # small value for divison by zero
R2D         = 180/math.pi; # Rad to deg
D2R         = 1/R2D;

#
class Helicopter(gym.Env, EzPickle):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second' : FPS
    }
    _default_yaml = os.path.join(os.path.dirname(__file__), "dynamics", "a109_param.yaml")
    def __init__(self, yaml_path:str = None):
        EzPickle.__init__(self)
        yaml_path = self._default_yaml if yaml_path is None else yaml_path
        self.heli_dyn = HelicopterDynamics.init_yaml(yaml_path)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(18,), dtype=np.float32)
        self.action_space = spaces.Box(0, +1, (4,), dtype=np.float32)

    def render(self):
        return self.heli_dyn.render_text()

    def step(self, actions):
        self.heli_dyn.step(actions)
        #next_states = self.heli_dyn.states
        observations = self.heli_dyn.get_observation()
        reward = self._calculate_reward()
        done = self._is_done()
        info = {}
        return observations, reward, done, info

    def reset(self):
        self.heli_dyn.reset()
        return self.heli_dyn.get_observation()

    def _calculate_reward(self):
        #self.heli_dyn.states
        return 0

    def _is_done(self):
        #self.heli_dyn.states
        return False

if __name__=='__main__':
    heli_env = Helicopter()
    print(heli_env)
    action = np.array([0.7007, 0.5391, 0.5351, 0.6429])