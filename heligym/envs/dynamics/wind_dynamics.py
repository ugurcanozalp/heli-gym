import yaml
import sys, math
import numpy as np
import os
import copy

from .dynamics import DynamicSystem, State
from .lookup import LookUpTable

EPS         = 1e-4 # small value for divison by zero
FT2MTR      = 0.3048 # ft to meter
SQRT_3      = 1.7320508075688772 # sqrt(3)
TWO_D_PI    = 0.6366197723675814 # 2/pi
R2D         = 180/math.pi # Rad to deg
D2R         = 1/R2D

class WindDynamics(DynamicSystem):
    """Wind model for aircraft simulations with Dryden Turbulence model."""
    _observations = ["WIND_N", "WIND_E", "WIND_D"]

    def __init__(self, params, dt):
        super(WindDynamics, self).__init__(dt)
        self.eta_norm = 1.0/np.sqrt(dt)
        self.turbulence_level = params['TURB_LVL']
        self.wind_dir = params['WIND_DIR']*D2R
        self.wind_speed = params['WIND_SPD']
        self.wind_mean_ned = self.wind_speed*np.array([np.cos(self.wind_dir), np.sin(self.wind_dir), 0], dtype=np.float32)
        self.__register_states()
        self.TEP = LookUpTable(7,12) # Turbulence Exceedence Probability Lookup Table
        self.TEP       << 500.0 << 1750.0 << 3750.0 << 7500.0 << 15000.0 << 25000.0 << 35000.0 << 45000.0 << 55000.0 << 65000.0 << 75000.0 << 80000.0 \
                << 1   <<   3.2 <<    2.2 <<    1.5 <<    0.0 <<     0.0 <<     0.0 <<     0.0 <<     0.0 <<     0.0 <<     0.0 <<     0.0 <<     0.0 \
                << 2   <<   4.2 <<    3.6 <<    3.3 <<    1.6 <<     0.0 <<     0.0 <<     0.0 <<     0.0 <<     0.0 <<     0.0 <<     0.0 <<     0.0 \
                << 3   <<   6.6 <<    6.9 <<    7.4 <<    6.7 <<     4.6 <<     2.7 <<     0.4 <<     0.0 <<     0.0 <<     0.0 <<     0.0 <<     0.0 \
                << 4   <<   8.6 <<    9.6 <<   10.6 <<   10.1 <<     8.0 <<     6.6 <<     5.0 <<     4.2 <<     2.7 <<     0.0 <<     0.0 <<     0.0 \
                << 5   <<  11.8 <<   13.0 <<   16.0 <<   15.1 <<    11.6 <<     9.7 <<     8.1 <<     8.2 <<     7.9 <<     4.9 <<     3.2 <<     2.1 \
                << 6   <<  15.6 <<   17.6 <<   23.0 <<   23.6 <<    22.1 <<    20.0 <<    16.0 <<    15.1 <<    12.1 <<     7.9 <<     6.2 <<     5.1 \
                << 7   <<  18.7 <<   21.5 <<   28.4 <<   30.2 <<    30.7 <<    31.0 <<    25.2 <<    23.1 <<    17.5 <<    10.7 <<     8.4 <<     7.2

    def __register_states(self):
        self._register_state('us', np.zeros(1, dtype=np.float32)) # States related to u
        self._register_state('vs', np.zeros(2, dtype=np.float32)) # States related to v
        self._register_state('ws', np.zeros(2, dtype=np.float32)) # States related to w

    def reset(self):
        self.state['us'] = np.zeros(1, dtype=np.float32)
        self.state['vs'] = np.zeros(2, dtype=np.float32)
        self.state['ws'] = np.zeros(2, dtype=np.float32)

    def step_before(self):
        """Randomly generate white noise for turbulence input
        """
        self.eta = np.random.randn(3)*self.eta_norm

    def _calc_params(self, h_gr, vel_inf_ned):
        # MIL-HDBK-1797 and MIL-HDBK-1797B
        w20 = self.turbulence_level / 7 * 88.61 # mean wind speed at 20ft in [ft/s]
        if h_gr <= 1000.0: # Low-altitude turbulence 
            h_gr = max(h_gr, 10.0)
            Lu = h_gr/( (0.177 + 0.000823*h_gr)**1.2 )
            Lv = 0.5*Lu
            Lw = 0.5*h_gr
            sigma_w = 0.1*w20
            sigma_u = sigma_w/( (0.177 + 0.000823*h_gr)**0.4 )
            sigma_v = sigma_u
            turb_azimuth = self.wind_dir
        elif h_gr >= 2000.0: # High-altitude turbulence
            Lu = 1750.0
            Lv = 0.5*Lu
            Lw = 0.5*Lu
            sigma = self.TEP.get_value_2D(self.turbulence_level, h_gr)
            sigma_u, sigma_v, sigma_w = sigma, sigma, sigma
            turb_azimuth = np.arctan2(vel_inf_ned[1], vel_inf_ned[0])
        else: # Medium-altitude turbulence which is interpolation of 1000 ft (Low-altitude) and 2000 ft (high-altitude)
            Lu = 1000 + (h_gr - 1000.0) / 1000.0 * 750.0
            Lv = 0.5*Lu
            Lw = Lu
            sigma = 0.1 * w20 + (h_gr - 1000.0) / 1000.0 * (self.TEP.get_value_2D(self.turbulence_level, h_gr) - 0.1 * w20)
            sigma_u, sigma_v, sigma_w = sigma, sigma, sigma
            r = (h_gr - 1000.0) / 1000.0
            turb_azimuth = np.arctan2(vel_inf_ned[1]*r + self.wind_mean_ned[1]*(1-r), 
                vel_inf_ned[0]*r + self.wind_mean_ned[0]*(1-r))

        return Lu, Lv, Lw, sigma_u, sigma_v, sigma_w, turb_azimuth

    def dynamics(self, state, set_observation=False):
        state_dots = self.state_dots

        us = state["us"]
        vs = state["vs"]
        ws = state["ws"]

        vel_ac_ned = self.action[:3]
        vel_inf_ned = vel_ac_ned + self.wind_mean_ned
        vel_inf = np.linalg.norm(vel_inf_ned)
        h_gr = self.action[3]
        Lu, Lv, Lw, sigma_u, sigma_v, sigma_w, turb_azimuth = self._calc_params(float(h_gr), vel_inf_ned)
        t_u = Lu/(vel_inf + EPS)
        t_v = Lv/(vel_inf + EPS)
        t_w = Lw/(vel_inf + EPS)
        
        usdot = np.array([1/t_u*(self.eta[0] - us[0])], dtype=np.float32)
        vsdot = np.array([1/(4*t_v**2)*(self.eta[1] - vs[1]) - 1/t_v*vs[0], 
                          vs[0]], dtype=np.float32) 
        wsdot = np.array([1/(4*t_w**2)*(self.eta[2] - ws[1]) - 1/t_w*ws[0], 
                          ws[0]], dtype=np.float32) 

        state_dots["us"] = usdot
        state_dots["vs"] = vsdot
        state_dots["ws"] = wsdot

        if set_observation:
            K_u = sigma_u*np.sqrt(TWO_D_PI*t_u)
            K_v = sigma_v*np.sqrt(TWO_D_PI*t_v)
            K_w = sigma_w*np.sqrt(TWO_D_PI*t_w)
            u_turb = K_u*us[0]
            v_turb = K_v*(vs[1]+2*SQRT_3*vs[0])
            w_turb = K_w*(ws[1]+2*SQRT_3*ws[0])
            cturb, sturb = np.cos(turb_azimuth), np.sin(turb_azimuth)
            turb_vel = np.array([cturb*u_turb - sturb*v_turb, 
                sturb*u_turb + cturb*v_turb, 
                w_turb], dtype=np.float32)
            wind_vel = self.wind_mean_ned + turb_vel
            self.observation = wind_vel

        return state_dots

if __name__=='__main__':
    import matplotlib.pyplot as plt 
    params = {'TURB_LVL': 3, 'WIND_DIR': 45, 'WIND_SPD': 30}
    turb_dyn = WindDynamics(turbulence_level=1, dt=0.01)
    h_gr = np.array([500], dtype=np.float32)
    vel = np.array([100,0,0], dtype=np.float32)
    all_obs = []
    time = 0.01*np.arange(30000)
    for t in time:
        action = np.concatenate([vel, h_gr])
        turb_dyn.step(action)
        observation = turb_dyn.observation()
        all_obs.append(observation)

    all_obs = np.stack(all_obs)
    fig = plt.figure()
    axis = fig.subplots(3,1)
    axis[0].plot(time, all_obs[:, 0])
    axis[1].plot(time, all_obs[:, 1])
    axis[2].plot(time, all_obs[:, 2])
    fig.show()
