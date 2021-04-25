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

class Heli(gym.Env, EzPickle):
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
        self.max_time = 30 # seconds
        self.success_duration = 5 # seconds
        self.successed_time = 0 # time counter for successing task through time.
        self.renderer = None

    def render(self):
        # return self.renderer()
        return self.heli_dyn.render_text()

    def step(self, actions):
        self.time_counter += DT
        self.heli_dyn.step(actions)
        observations = self.heli_dyn.get_observation()
        reward = self._calculate_reward()
        info = self._get_info()
        done = info['failed'] or info['successed'] or (self.time_counter > self.max_time)
        self.successed_time = self.successed_time + DT if info['successed_step'] else 0
        return observations, reward, done, info

    def reset(self):
        self.time_counter = 0
        self.heli_dyn.reset()
        return self.heli_dyn.get_observation()      

    def _get_info(self):
        return {'failed': self._is_failed(), 'successed': self._is_successed(), 'successed_step': self._is_successed_step()}

    def _is_successed(self):
        return self.successed_time >= self.success_duration  

    def _is_failed(self):
        cond1 = self.heli_dyn._does_hit_ground(-self.heli_dyn.state['xyz'][2])
        cond2 = self.heli_dyn.state_dots['xyz'][2] > self.heli_dyn.MR['V_TIP']*0.05
        cond3 = self.heli_dyn.state['euler'][0] > 60*D2R
        cond4 = self.heli_dyn.state['euler'][1] > 60*D2R
        cond5 = np.abs(self.heli_dyn.state['xyz'][0]) > 5000 or np.abs(self.heli_dyn.state['xyz'][1]) > 5000 or -self.heli_dyn.state['xyz'][2] > self.heli_dyn.ENV['GR_ALT'] + 10000
        cond = (cond1 and (cond2 or cond3 or cond4)) or cond5
        return cond

    def _is_successed_step(self):
        raise NotImplementedError

    def _calculate_reward(self):
        raise NotImplementedError

class HeliHover(Heli):
    def __init__(self, yaml_path:str = None):
        Heli.__init__(self, yaml_path=yaml_path)
        self.target_location = np.array([0,0,-self.heli_dyn.ENV['GR_ALT']-100])

    def _is_successed_step(self):
        return np.linalg.norm(self.heli_dyn.state['xyz'] - self.target_location) < (2*self.heli_dyn.MR['R'])

    def _calculate_reward(self):
        dloc = self.heli_dyn.state['xyz'] - self.target_location
        dloc_norm = np.linalg.norm(dloc)
        vel = self.heli_dyn.state_dots['xyz']
        direction_sim = np.inner(dloc/dloc_norm, vel/(0.75*self.heli_dyn.MR['V_TIP']))
        closeness = np.exp(-dloc_norm/(4*self.heli_dyn.MR['R'])) 
        reward = (1-closeness)*direction_sim + closeness
        return reward

if __name__=='__main__':
    env = HeliHover()
    #print(env)
    #obs = env.reset()
    #action = np.array([0.7007, 0.5391, 0.5351, 0.6429])
    #obs, reward, done, info = env.step(action)