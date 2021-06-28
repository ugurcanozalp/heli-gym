"""
Helicopter control.
"""
import yaml
import sys, math
import numpy as np
import os

import gym
from gym import spaces
from gym.utils import seeding, EzPickle

from .dynamics import HelicopterDynamics
from .dynamics import WindDynamics
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
    # Default maximum time for an episode
    default_max_time = 40.0
    # Default task target
    default_task_target = np.zeros(HelicopterDynamics.n_obs)
    # Default trim condition, ground trim.
    default_trim_cond = {
        "yaw": 0.0,
        "yaw_rate": 0.0,
        "ned_vel": [0.0, 0.0, 0.0],
        "gr_alt": 10.0,
        "xy": [0.0, 0.0],
        "psi_mr": 0.0,
        "psi_tr": 0.0
    }
    default_reward_weight = np.zeros((HelicopterDynamics.n_obs,HelicopterDynamics.n_obs))
    def __init__(self, heli_name:str = "aw109"):
        EzPickle.__init__(self)
        yaml_path = os.path.join(os.path.dirname(__file__), "helis", heli_name + ".yaml")
        with open(yaml_path) as foo:
            params = yaml.safe_load(foo)

        self.heli_dyn = HelicopterDynamics(params, DT)
        self.wind_dyn = WindDynamics(params['ENV'], DT)
        self.heli_dyn.set_wind(self.wind_dyn.wind_mean_ned) # set mean wind as wind so that helicopter trims accordingly
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(self.heli_dyn.n_obs,), dtype=np.float32)
        self.action_space = spaces.Box(-1, +1, (self.heli_dyn.n_act,), dtype=np.float32)
        self.obs_mask = np.zeros(HelicopterDynamics.n_obs)
        self.successed_time = 0 # time counter for successing task through time.
        self.set_max_time()
        self.set_target()
        self.set_trim_cond()
        self.set_reward_weight()
        
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
       
    # Setter functions for RL tasks
    def set_max_time(self, max_time=None):
        self.max_time = self.default_max_time if max_time is None else max_time  # [sec] Given time for episode
        self.success_duration = self.max_time/4 # [sec] Required successfull time for maneuver
        self.task_duration = self.max_time/4 # [sec] Allowed time for successing task

    def set_target(self, target=None):
        self.task_target = self.default_task_target if target is None else target

    def set_trim_cond(self, trim_cond={}):
        self.trim_cond = self.default_trim_cond
        self.trim_cond.update(trim_cond)

    def set_reward_weight(self, reward_weight=None):
        self.reward_weight = self.default_reward_weight if reward_weight is None else reward_weight

    def __create_guiINFO_text(self):
        self.guiINFO_ID = self.renderer.create_guiText("Observations", 30.0, 30.0, 250.0, 0.0)
        self.guiINFO_text = []
        self.guiINFO_text.append("FPS        : %3.0f ")
        self.guiINFO_text.append("POWER      : %5.2f hp" )
        self.guiINFO_text.append("TAS        : %5.2f ft/s")
        self.guiINFO_text.append("AOA        : %5.2f °")
        self.guiINFO_text.append("SSLIP      : %5.2f °")
        self.guiINFO_text.append("N_VEL      : %5.2f ft/s")
        self.guiINFO_text.append("E_VEL      : %5.2f ft/s")
        self.guiINFO_text.append("DES_RATE   : %5.2f ft/s")
        self.guiINFO_text.append("ROLL       : %5.2f °")
        self.guiINFO_text.append("PITCH      : %5.2f °")
        self.guiINFO_text.append("YAW        : %5.2f °")
        self.guiINFO_text.append("ROLL_RATE  : %5.2f °/sec")
        self.guiINFO_text.append("PITCH_RATE : %5.2f °/sec")
        self.guiINFO_text.append("YAW_RATE   : %5.2f °/sec")
        self.guiINFO_text.append("LON_ACC    : %5.2f ft/sec^2")
        self.guiINFO_text.append("LAT_ACC    : %5.2f ft/sec^2")
        self.guiINFO_text.append("DOWN_ACC   : %5.2f ft/sec^2")
        self.guiINFO_text.append("N_POS      : %5.2f ft")
        self.guiINFO_text.append("E_POS      : %5.2f ft")
        self.guiINFO_text.append("ALTITUDE   : %5.2f ft")
        self.guiINFO_text.append("GR_ALT     : %5.2f ft")

    def __add_to_guiText(self):
        info_val = np.array(self.renderer.get_fps())
        info_val = np.append(info_val, self.heli_dyn.observation)
        self.renderer.add_guiText(self.guiINFO_ID, self.guiINFO_text, info_val) 

    def render(self):
        info_val = np.array(self.renderer.get_fps())
        info_val = np.append(info_val, self.heli_dyn.observation)
        
        self.renderer.set_guiText(self.guiINFO_ID, self.guiINFO_text, info_val)
        
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

        self.renderer.translate_model(self.sky, 
                                    self.heli_dyn.state['xyz'][0] * FT2MTR,
                                    self.heli_dyn.state['xyz'][1] * FT2MTR,
                                    self.heli_dyn.state['xyz'][2] * FT2MTR + 500
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
        # Turbulence calculations
        pre_observations = self.heli_dyn.observation
        wind_action = np.concatenate([pre_observations[4:7], pre_observations[19:]])
        self.wind_dyn.step(wind_action)
        wind_turb_vel = self.wind_dyn.observation
        # Helicopter dynamics calculations
        self.heli_dyn.set_wind(wind_turb_vel)        
        self.heli_dyn.step(actions)
        observation = self.heli_dyn.observation
        reward, successed_step = self._calculate_reward()
        info = self._get_info()
        done = info['failed'] or info['successed'] or info['time_up'] or reward == np.nan
        self.successed_time = self.successed_time + DT if successed_step else 0
        return observation, reward, done, info

    def reset(self):
        self.time_counter = 0
        self.successed_time = 0
        self.wind_dyn.reset()
        self.heli_dyn.reset(self.trim_cond)
        if not self._bGuiText:
            self.__create_guiINFO_text()
            self.__add_to_guiText()
            self._bGuiText = True
        return self.heli_dyn.observation      

    def _get_info(self):
        return {
            'failed': self._is_failed(), 
            'successed': self._is_successed(), 
            'time_up': self._is_time_up()
            }

    def _is_failed(self):
        cond1 = self.heli_dyn._does_hit_ground(-self.heli_dyn.state['xyz'][2])
        cond2 = self.heli_dyn.state_dots['xyz'][2] > self.heli_dyn.MR['V_TIP']*0.05
        cond3 = self.heli_dyn.state['euler'][0] > 60*D2R
        cond4 = self.heli_dyn.state['euler'][1] > 60*D2R
        cond5 = np.abs(self.heli_dyn.state['xyz'][0]) > self.heli_dyn.ENV["NS_MAX"] / 2 or np.abs(self.heli_dyn.state['xyz'][1]) > self.heli_dyn.ENV["EW_MAX"] / 2 \
             or -self.heli_dyn.state['xyz'][2] > self.heli_dyn.ground_touching_altitude() + 10000
        cond = (cond1 and (cond2 or cond3 or cond4)) or cond5
        return cond

    def _is_successed(self):
        return self.successed_time >= self.success_duration  

    def _is_time_up(self):
        return self.time_counter > self.max_time

    def _calculate_reward(self):
        obs_error = (self.heli_dyn.observation - self.task_target) * self.obs_mask
        obs_prev_error = (self.heli_dyn.previous_observation - self.task_target) * self.obs_mask
        cost_base = (obs_prev_error).transpose()@self.reward_weight@(obs_prev_error)
        cost_terminal = obs_error.transpose()@self.reward_weight@obs_error
        rule = (cost_terminal - cost_base) / 1000.0 - 60000.0 / cost_terminal
        reward = -np.tanh(rule)
        successed_step = abs(reward) > 0.999
        if successed_step:
            reward *= 4
        #print(f"Cost base: {cost_base}")
        #print(f"Cost terminal: {150000.0 / cost_terminal} {(cost_terminal - cost_base) / 1000.0 }")
        #print(f"Reward : {reward}")
        #print(f"Rule : {rule}")
        #print(f"Successed step :{successed_step}")
        return reward, successed_step

class HeliHover(Heli):
    def __init__(self, heli_name:str = "aw109"):
        Heli.__init__(self, heli_name=heli_name)
        max_time = 40.0
        task_target = Heli.default_task_target
        task_target[18] = 3000.0
        
        reward_weight = Heli.default_reward_weight
        reward_weight[7,7] = 0.9
        reward_weight[8,8] = 0.9
        reward_weight[9,9] = 0.9
        reward_weight[10,10] = 0.8
        reward_weight[11,11] = 0.8
        reward_weight[12,12] = 0.8
        reward_weight[16,16] = 1.0
        reward_weight[17,17] = 1.0
        reward_weight[18,18] = 1.0

        self.obs_mask[4:20] = 1.0

        self.set_max_time(max_time)
        self.set_target(task_target)
        self.set_reward_weight(reward_weight)
        self.new_start_hover_point()

    def new_start_hover_point(self, start_point=(0.0, 0.0, 0.0), alt_low = 0.0, alt_hight = 10.0):
        if start_point == None:
            gr_alt =  alt_low + (alt_hight - alt_low) * np.random.rand()
            xy = np.random.randn(2) * 1000
        else:
            gr_alt = start_point[2]
            xy = [start_point[0],start_point[1]]

        if gr_alt < 0:
            gr_alt = 0
        
        gr_alt = np.round(gr_alt, 2)

        trim_cond = {
            "yaw": 0.0,
            "yaw_rate": 0.0,
            "ned_vel": [0.0, 0.0, 0.0],
            "gr_alt": gr_alt,
            "xy": xy,
            "psi_mr": 0.0,
            "psi_tr": 0.0
        }
        self.set_trim_cond(trim_cond)

    def arange_observations(self, observations):
        obs = self.task_target - observations
        obs[0] /= 5000 # normalize HP
        obs[1] /= 500 # normalize TAS
        obs[2] /= 180 # normalize aoa
        obs[3] /= 180 # normalize sslip
        obs[4] /= 500 # normalize N_vel
        obs[5] /= 500 # normalize E_vel
        obs[6] /= 500 # normalize Des_rate
        obs[7] /= 180 # normalize roll
        obs[8] /= 180 # normalize pitch
        obs[9] /= 180 # normalize yaw
        obs[10] /= 360 # normalize roll_rate
        obs[11] /= 360 # normalize pitch_rate
        obs[12] /= 360 # normalize yaw_rate
        obs[13] /= 1000 # normalize long_acc
        obs[14] /= 1000 # normalize lat_acc
        obs[15] /= 1000 # normalize down_acc
        obs[16] /= 10000 # normalize n_pos
        obs[17] /= 10000 # normalize e_pos
        obs[18] /= 10000 # normalize alt
        obs[19] /= 10000 # normalize gr_alt
        return obs


class HeliForwardFlight(Heli):

    def __init__(self, heli_name:str = "aw109"):
        Heli.__init__(self, heli_name=heli_name)
        max_time = 40.0
        task_target = Heli.default_task_target
        task_target[4:6] = np.array([100, 50])
        task_target[18] = 2000.0
        trim_cond = {
            "yaw": 0.0,
            "yaw_rate": 0.0,
            "ned_vel": [0.0, 0.0, 0.0],
            "gr_alt": 10.0,
            "xy": [0.0, 0.0],
            "psi_mr": 0.0,
            "psi_tr": 0.0
        }
        reward_weight = Heli.default_reward_weight
        reward_weight[4,4] = 1/(self.heli_dyn.MR['R']*self.heli_dyn.ENV['GRAV'])
        reward_weight[5,5] = 1/(self.heli_dyn.MR['R']*self.heli_dyn.ENV['GRAV'])
        reward_weight[18,18] = 1/self.heli_dyn.MR['R']**2
        self.set_max_time(max_time)
        self.set_target(task_target)
        self.set_trim_cond(trim_cond)
        self.set_reward_weight(reward_weight)

class HeliObliqueFlight(Heli):

    def __init__(self, heli_name:str = "aw109"):
        Heli.__init__(self, heli_name=heli_name)
        max_time = 40.0
        task_target = Heli.default_task_target
        task_target[4:7] = np.array([40.0,0.0,15.0], dtype=np.float)
        trim_cond = {
            "yaw": 0.0,
            "yaw_rate": 0.0,
            "ned_vel": [0.0, 0.0, 0.0],
            "gr_alt": 10.0,
            "xy": [0.0, 0.0],
            "psi_mr": 0.0,
            "psi_tr": 0.0
        }
        reward_weight = Heli.default_reward_weight
        reward_weight[4,4] = 1/(self.heli_dyn.MR['R']*self.heli_dyn.ENV['GRAV'])
        reward_weight[5,5] = 1/(self.heli_dyn.MR['R']*self.heli_dyn.ENV['GRAV'])
        reward_weight[6,6] = 1/(self.heli_dyn.MR['R']*self.heli_dyn.ENV['GRAV'])
        self.set_max_time(max_time)
        self.set_target(task_target)
        self.set_trim_cond(trim_cond)
        self.set_reward_weight(reward_weight)

if __name__=='__main__':
    env = HeliHover()
    #print(env)
    #obs = env.reset()
    #action = np.array([0.7007, 0.5391, 0.5351, 0.6429])
    #obs, reward, done, info = env.step(action)
