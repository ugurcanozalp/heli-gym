"""
Helicopter control.
"""
import sys, math
import numpy as np
import os

import gym
from gym import spaces
from gym.utils import seeding, EzPickle

from .dynamics import HelicopterDynamics
from .renderer.api import Renderer

FPS         = 100.0
DT          = 1/FPS
FTS2KNOT    = 0.5924838 # ft/s to knots conversion
EPS         = 1e-10 # small value for divison by zero
R2D         = 180/math.pi # Rad to deg
D2R         = 1/R2D
FT2MTR      = 0.3048 # ft to meter

class Heli(gym.Env, EzPickle):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second' : FPS
    }
    def __init__(self, heli_name:str = "aw109"):
        EzPickle.__init__(self)
        yaml_path = os.path.join(os.path.dirname(__file__), "helis", heli_name + ".yaml")
        self.heli_dyn = HelicopterDynamics.init_yaml(yaml_path, DT, os.environ['HELIGYM_RESOURCE_DIR'] + '/models/terrain/hmap.npy')
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(18,), dtype=np.float32)
        self.action_space = spaces.Box(-1, +1, (4,), dtype=np.float32)
        self.max_time = 30 # seconds
        self.success_duration = 5 # seconds
        self.successed_time = 0 # time counter for successing task through time.
        self.trim_cond = {
            "yaw": 0.0,
            "yaw_rate": 0.0,
            "ned_vel": [0.0, 0.0, 0.0],
            "gr_alt": 100.0,
            "xy": [0.0, 0.0],
            "psi_mr": 0.0,
            "psi_tr": 0.0
        }
        
        self.renderer = Renderer(w=1024, h=768, title='heligym')
        self.renderer.set_fps(FPS)

        self.heli_obj = self.renderer.create_model('/resources/models/'+heli_name+'/'+heli_name+'.obj',
                                                          '/resources/shaders/'+heli_name+'_vertex.vs',
                                                          '/resources/shaders/'+heli_name+'_frag.fs')
        self.renderer.add_permanent_object_to_window(self.heli_obj)
     
        self.terrain = self.renderer.create_model('/resources/models/terrain/terrain.obj',
                                                 '/resources/shaders/terrain_vertex.vs',
                                                 '/resources/shaders/terrain_frag.fs')
        self.renderer.add_permanent_object_to_window(self.terrain)        

        self.sky = self.renderer.create_model('/resources/models/sky/sky.obj')
        self.renderer.add_permanent_object_to_window(self.sky)

        self._bGuiText = False
       
    def set_max_time(self, max_time):
        self.max_time = max_time

    def __create_guiINFO_text(self):
        self.guiINFO_text = []
        self.guiINFO_text.append( bytes( "POWER      : %5.2f hp" , 'utf-8'))
        self.guiINFO_text.append( bytes( "TAS        : %5.2f ft/s", 'utf-8'))
        self.guiINFO_text.append( bytes( "AOA        : %5.2f °", 'utf-8'))
        self.guiINFO_text.append( bytes( "SSLIP      : %5.2f °", 'utf-8'))
        self.guiINFO_text.append( bytes( "N_VEL      : %5.2f ft/s", 'utf-8'))
        self.guiINFO_text.append( bytes( "E_VEL      : %5.2f ft/s", 'utf-8'))
        self.guiINFO_text.append( bytes( "CLIMB_RATE : %5.2f ft/s", 'utf-8'))
        self.guiINFO_text.append( bytes( "ROLL       : %5.2f °", 'utf-8'))
        self.guiINFO_text.append( bytes( "PITCH      : %5.2f °", 'utf-8'))
        self.guiINFO_text.append( bytes( "YAW        : %5.2f °", 'utf-8'))
        self.guiINFO_text.append( bytes( "ROLL_RATE  : %5.2f °/sec", 'utf-8'))
        self.guiINFO_text.append( bytes( "PITCH_RATE : %5.2f °/sec", 'utf-8'))
        self.guiINFO_text.append( bytes( "YAW_RATE   : %5.2f °/sec", 'utf-8'))
        self.guiINFO_text.append( bytes( "LON_ACC    : %5.2f ft/sec^2", 'utf-8'))
        self.guiINFO_text.append( bytes( "LAT_ACC    : %5.2f ft/sec^2", 'utf-8'))
        self.guiINFO_text.append( bytes( "DOWN_ACC   : %5.2f ft/sec^2", 'utf-8'))
        self.guiINFO_text.append( bytes( "N_POS      : %5.2f ft", 'utf-8'))
        self.guiINFO_text.append( bytes( "E_POS      : %5.2f ft", 'utf-8'))
        self.guiINFO_text.append( bytes( "ALTITUDE   : %5.2f ft", 'utf-8'))

    def __add_to_guiText(self):
        self.renderer.add_guiOBS(self.guiINFO_text, self.heli_dyn._get_observation())        

    def render(self):
        val = self.heli_dyn._get_observation()
        val = np.append(val, np.array([0.3, 0.2]))

        self.renderer.set_guiOBS(self.guiINFO_text, val)
        
        self.renderer.rotate_MR(self.heli_obj, 
                                self.heli_dyn.state['betas'][1],
                                self.heli_dyn.state['betas'][0],
                                self.heli_dyn.state['psi_mr'][0])

        self.renderer.rotate_TR(self.heli_obj, 
                                0,
                                self.heli_dyn.state['psi_tr'][0],
                                0)


        self.renderer.translate_model(self.heli_obj, 
                                    self.heli_dyn.state['xyz'][0] * FT2MTR,
                                    self.heli_dyn.state['xyz'][1] * FT2MTR,
                                    self.heli_dyn.state['xyz'][2] * FT2MTR
                                    )

        self.renderer.rotate_model(self.heli_obj, 
                                    self.heli_dyn.state['euler'][0],
                                    self.heli_dyn.state['euler'][1],
                                    self.heli_dyn.state['euler'][2]
                                    )

        self.renderer.set_camera_pos(self.heli_dyn.state['xyz'][0] * FT2MTR,
                                     self.heli_dyn.state['xyz'][1] * FT2MTR + 30,
                                     self.heli_dyn.state['xyz'][2] * FT2MTR )


        if not self.renderer.is_visible():
            self.renderer.show_window()
            
        self.renderer.render()
        
    def close(self):
        self.renderer.close()
        self.renderer.terminate()
    
    def exit(self):
        sys.exit()

    def step(self, actions):
        self.time_counter += DT
        self.heli_dyn.step(actions)
        observation = self.heli_dyn._get_observation()
        reward = self._calculate_reward()
        info = self._get_info()
        done = info['failed'] or info['successed'] or info['time_up']
        self.successed_time = self.successed_time + DT if info['successed_step'] else 0
        return observation, reward, done, info

    def reset(self):
        self.time_counter = 0
        self.heli_dyn.reset(self.trim_cond)
        if not self._bGuiText:
            self.__create_guiINFO_text()
            self.__add_to_guiText()
            self._bGuiText = True
        return self.heli_dyn._get_observation()      

    def _get_info(self):
        return {
            'failed': self._is_failed(), 
            'successed': self._is_successed(), 
            'successed_step': self._is_successed_step(),
            'time_up': self._is_time_up()
            }

    def _is_failed(self):
        cond1 = self.heli_dyn._does_hit_ground(-self.heli_dyn.state['xyz'][2])
        cond2 = self.heli_dyn.state_dots['xyz'][2] > self.heli_dyn.MR['V_TIP']*0.05
        cond3 = self.heli_dyn.state['euler'][0] > 60*D2R
        cond4 = self.heli_dyn.state['euler'][1] > 60*D2R
        cond5 = np.abs(self.heli_dyn.state['xyz'][0]) > 5000 or np.abs(self.heli_dyn.state['xyz'][1]) > 5000 or -self.heli_dyn.state['xyz'][2] > self.heli_dyn.ENV['GR_ALT'] + 10000
        cond = (cond1 and (cond2 or cond3 or cond4)) or cond5
        return cond

    def _is_successed(self):
        return self.successed_time >= self.success_duration  

    def _is_time_up(self):
        return self.time_counter > self.max_time

    def _is_successed_step(self):
        raise NotImplementedError

    def _calculate_reward(self):
        raise NotImplementedError

class HeliHover(Heli):
    def __init__(self, heli_name:str = "aw109"):
        Heli.__init__(self, heli_name=heli_name)
        self.target_location = np.array([0,0,-self.heli_dyn.ENV['GR_ALT']-200])
        self.trim_cond = {
            "yaw": 0.0,
            "yaw_rate": 0.0,
            "ned_vel": [0.0, 0.0, 0.0],
            "gr_alt": 0.0,
            "xy": [0.0, 0.0],
            "psi_mr": 0.0,
            "psi_tr": 0.0
        }

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
