import sys, math
import numpy as np
import os
import copy
import imageio

from .kinematic import euler_to_rotmat, pqr_to_eulerdot_mat
from .dynamics import DynamicSystem, State
from .utils import pi_bound
from .lookup import LookUpTable

FTS2KNOT    = 0.5924838 # ft/s to knots conversion
EPS         = 1e-4 # small value for divison by zero
R2D         = 180/math.pi # Rad to deg
D2R         = 1/R2D
FT2MTR      = 0.3048 # ft to meter

class HelicopterDynamics(DynamicSystem):

    _observations = ["POWER", "TAS", "AOA", "SSLIP", "N_VEL", "E_VEL", "DES_RATE", 
        "ROLL", "PITCH", "YAW", "ROLL_RATE", "PITCH_RATE", "YAW_RATE", 
        "LON_ACC", "LAT_ACC", "DWN_ACC", "N_POS", "E_POS", "ALTITUDE", "GROUND_ALTITUDE"]
    
    def __init__(self, params, dt):
        super(HelicopterDynamics, self).__init__(dt)
        self.HELI = params['HELI']
        self.ENV = params['ENV']
        # Null state dots, since it is not updated.
        self.__precalculations()
        self.set_wind() # wind velocity in earth frame:
        self.__register_states()
        hmap_img = imageio.imread(os.environ['HELIGYM_RESOURCE_DIR'] + self.ENV["HMAP_PATH"])
        hmap_img = hmap_img/np.iinfo(hmap_img.dtype).max
        normal_img = imageio.imread(os.environ['HELIGYM_RESOURCE_DIR'] + self.ENV["NMAP_PATH"])
        normal_img = normal_img/np.iinfo(normal_img.dtype).max
        self.terrain_hmap = hmap_img*self.ENV["MAX_GR_ALT"]
        self.terrain_normal = normal_img/np.sqrt((normal_img**2+EPS).sum(axis=-1, keepdims=True))
        self.default_trim_cond = {
            "yaw": 0.0,
            "yaw_rate": 0.0,
            "ned_vel": [0.0, 0.0, 0.0],
            "gr_alt": 100.0,
            "xy": [0.0, 0.0],
            "psi_mr": 0.0,
            "psi_tr": 0.0
        }

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
        self._register_state('vi_mr', np.zeros(1, dtype=np.float))
        self._register_state('vi_tr', np.zeros(1, dtype=np.float))
        self._register_state('psi_mr', np.zeros(1, dtype=np.float))
        self._register_state('psi_tr', np.zeros(1, dtype=np.float))
        self._register_state('betas', np.zeros(2, dtype=np.float))
        self._register_state('uvw', np.zeros(3, dtype=np.float))
        self._register_state('pqr', np.zeros(3, dtype=np.float))
        self._register_state('euler', np.zeros(3, dtype=np.float))
        self._register_state('xyz', np.zeros(3, dtype=np.float))
              
    def reset(self, trim_cond={}):
        input_trim_cond = copy.deepcopy(self.default_trim_cond)
        input_trim_cond.update(trim_cond)
        self.trim(input_trim_cond)

    def step_end(self):
        self.state['psi_mr'] = pi_bound(self.state['psi_mr'])
        self.state['psi_tr'] = pi_bound(self.state['psi_tr'])
        self.state['betas'] = pi_bound(self.state['betas'])
        self.state['euler'] = pi_bound(self.state['euler'])

    @property
    def MR(self):
        return self.HELI['MR']

    @property
    def TR(self):
        return self.HELI['TR']

    @property
    def FUS(self):
        return self.HELI['FUS']

    @property
    def HT(self):
        return self.HELI['HT']

    @property
    def VT(self):
        return self.HELI['VT'] 

    @property
    def WN(self):
        return self.HELI['WN']    
    
    def __precalculations(self):
        # Component positions wrt CG locations
        # 1/12 comes from inch to feet conversion
        self.MR['H'] = (self.MR['WL']-self.HELI['WL_CG'])/12  # [ft]
        self.MR['D'] = (self.MR['FS']-self.HELI['FS_CG'])/12
        self.FUS['H'] = (self.FUS['WL']-self.HELI['WL_CG'])/12
        self.FUS['D'] = (self.FUS['FS']-self.HELI['FS_CG'])/12
        self.WN['H']  = (self.WN['WL']-self.HELI['WL_CG'])/12
        self.WN['D']  = (self.WN['FS']-self.HELI['FS_CG'])/12
        self.HT['H']  = (self.HT['WL']-self.HELI['WL_CG'])/12
        self.HT['D']  = (self.HT['FS']-self.HELI['FS_CG'])/12
        self.VT['H']  = (self.VT['WL']-self.HELI['WL_CG'])/12
        self.VT['D']  = (self.VT['FS']-self.HELI['FS_CG'])/12
        self.TR['H']  = (self.TR['WL']-self.HELI['WL_CG'])/12
        self.TR['D']  = (self.TR['FS']-self.HELI['FS_CG'])/12
        #
        self.HELI['M']=self.HELI['WT']/self.ENV['GRAV'] # [slug] vehicle mass
        # Main Rotor precalculations
        self.MR['OMEGA'] = self.MR['RPM']*2*math.pi/60 # [rad/s] MR rev speed
        self.MR['V_TIP'] = self.MR['R']*self.MR['OMEGA'] # [ft/s] MR tip speed
        self.MR['FR'] = self.MR['CD0']*self.MR['R']*self.MR['B']*self.MR['C'] # eff.frontal area MR
        self.MR['SOL'] = self.MR['B']*self.MR['C']/(self.MR['R']*math.pi) # MR solidity (SIGMA)
        self.MR['A_SIGMA'] = self.MR['A']*self.MR['SOL'] # product(lift-curve-slope & solidity)
        self.MR['GAM_OM16_DRO'] = self.MR['A']*self.MR['C']*self.MR['R']**4/self.MR['IB']* \
            self.MR['OMEGA']/16*(1+8/3*self.MR['E']/self.MR['R']) # one sixth the product(lock# and rotor ang.rate), divided by air density
        # primary(direct)flapping stiffness [rad/sec2]
        self.MR['DL_DB1'] = self.MR['B']/2* \
            (1.5*self.MR['IB']*self.MR['E']/self.MR['R']*self.MR['OMEGA']**2)
        # cross(off-axis)flapping stiffness [rad/sec2], divided by air density
        self.MR['DL_DA1_DRO'] = 0.5*self.MR['A']*self.MR['B']*self.MR['C']*self.MR['R']*self.MR['V_TIP']**2*self.MR['E']/6
        self.MR['COEF_TH'] = self.MR['V_TIP']*self.MR['R']*self.MR['A']*self.MR['B']*self.MR['C'] # coefficient for thrust calculation
        # Tail Rotor precalculations
        self.TR['OMEGA'] = self.TR['RPM']*2*math.pi/60 # [rad/s] TR rev speed
        self.TR['FR'] = self.TR['CD0']*self.TR['R']*self.TR['B']*self.TR['C'] # eff.frontal area TR
        self.TR['V_TIP'] = self.TR['R']*self.TR['OMEGA'] # [ft/s] TR tip speed
        self.TR['SOL'] = self.TR['B']*self.TR['C']/(self.TR['R']*math.pi) # TR solidity (SIGMA)
        self.TR['COEF_TH'] = self.TR['V_TIP']*self.TR['R']*self.TR['A']*self.TR['B']*self.TR['C'] # coefficient for thrust calculation
        # Inertia
        #print(self.HELI['IX'])
        self.HELI['I'] = np.array([ [self.HELI['IX']     ,   0.0                 ,   -self.HELI['IXZ']], 
                                    [0.0                 ,   self.HELI['IY']     ,   0.0             ],
                                    [-self.HELI['IXZ']   ,   0.0                 ,   self.HELI['IZ'] ] ], dtype=np.float32 )

    def set_wind(self, wind_ned: np.ndarray = np.zeros(3)):
        # wind velocity in earth frame:
        self.WIND_NED = wind_ned

    def _altitude_to_air_properties(self, altitude):
        """Calculate Air temperature and density from altitude.
        """
        temp = self.ENV['T0'] - self.ENV['LAPSE']*altitude # [R] Temperature at the current altitude
        rho = self.ENV['RO_SEA']*(temp/self.ENV['T0'])**((self.ENV['GRAV']/(self.ENV['LAPSE']*self.ENV['R']))-1.0) # [slug/ft**3]
        return temp, rho

    def __get_ground_height_from_hmap(self):
        x_ = self.ENV["NS_MAX"] / self.terrain_hmap.shape[0] # terrain x size per pixel
        y_ = self.ENV["EW_MAX"] / self.terrain_hmap.shape[1] # terrain y size per pixel

        x_loc = self.state['xyz'][0] / x_ + self.terrain_hmap.shape[0] // 2
        y_loc = self.state['xyz'][1] / y_ + self.terrain_hmap.shape[1] // 2

        x_ind =  math.floor(x_loc)
        y_ind =  math.floor(y_loc)

        middle = self.terrain_hmap[x_ind, y_ind]
        north = self.terrain_hmap[x_ind+1, y_ind]
        east = self.terrain_hmap[x_ind, y_ind+1]

        height = middle + (north - middle) * (x_loc - x_ind) + (east - middle) * (y_loc - y_ind)
        return height

    def _does_hit_ground(self, altitude):
        return altitude - self.ground_touching_altitude() < 0.0

    def ground_touching_altitude(self):
        return self.__get_ground_height_from_hmap() + self.HELI['WL_CG']/12 # divide by 12 to make inch to feet

    def _calc_mr_fm(self, rho, coll, lon, lat, betas, uvw_air, pqr, vi_mr, psi_mr):
        """Calculate Forces and Moments caused by Main Rotor
        ans Main Rotor Dynamics
        """
        ### Calculate required parameters first. 
        # one sixth the product(lock# and rotor ang.rate)
        GAM_OM16 = rho*self.MR['GAM_OM16_DRO']
        # flapping aero cpl(flapping coupling factor)    
        KC = (0.75*self.MR['OMEGA']*self.MR['E']/self.MR['R']/GAM_OM16)+self.MR['K1']
        # flapping x-cpl coef
        ITB2_OM = self.MR['OMEGA']/(1+(self.MR['OMEGA']/GAM_OM16)**2)
        # flapping primary resp(inverse TPP lag) [rad/s]
        ITB = ITB2_OM*self.MR['OMEGA']/GAM_OM16
        # primary(direct)flapping stiffness [rad/sec2]
        DL_DB1 = self.MR['DL_DB1']
        # cross(off-axis)flapping stiffness [rad/sec2]
        DL_DA1 = rho*self.MR['DL_DA1_DRO']

        ## MR Force Moments and inflow dynamics.
        v_adv_2 = uvw_air[0]**2+uvw_air[1]**2
        wr = uvw_air[2] + (betas[0]-self.MR['IS'])*uvw_air[0] - betas[1]*uvw_air[1] # z-axis vel re rotor plane
        wb = wr + 2/3*self.MR['V_TIP']*(coll+0.75*self.MR['TWST']) + \
            v_adv_2/self.MR['V_TIP']*(coll+0.5*self.MR['TWST']) # z-axis vel re blade (equivalent)
        
        thrust_mr = (wb - vi_mr[0]) * rho*self.MR['COEF_TH']/4
        vi_mr_dot = np.zeros(1)
        vi_mr_dot[0] = math.pi*3/4/self.MR['R']*(thrust_mr/(2*math.pi*rho*self.MR['R']**2) - vi_mr[0]*np.sqrt(v_adv_2+(wr-vi_mr[0])**2))

        # MR induced flow power consumption
        induced_power=thrust_mr*(vi_mr[0]-wr)
        # MR profile drag power consumption
        profile_power=0.5*rho*(self.MR['FR']/4)*self.MR['V_TIP']*(self.MR['V_TIP']**2+ \
                     3.0*v_adv_2)
        power_mr=induced_power+profile_power
        torque_mr=power_mr/self.MR['OMEGA']

        # thrust coeff.
        CT = thrust_mr/(rho*math.pi*self.MR['R']**2*self.MR['V_TIP']**2)
        CT = np.max([CT, 0])
        ## Dihedral effect on TPP
        # TPP dihedral effect(late.flap2side vel)
        DB1DV = 2/self.MR['V_TIP']*(8*CT/self.MR['A_SIGMA']+(math.sqrt(CT/2)))
        DA1DU = -DB1DV # TPP pitchup with speed

        ### MR TPP Dynamics
        wake_fn = 0.5 + 0.5*np.tanh(10*(np.abs(uvw_air[0])/self.HELI['VTRANS']-1.0))
        a_sum = betas[1]-lat+KC*betas[0]+DB1DV*uvw_air[1]*(1+wake_fn)
        b_sum = betas[0]+lon-KC*betas[1]+DA1DU*uvw_air[0]*(1+2*wake_fn)
        betas_dot = np.zeros(2)
        betas_dot[0] = -ITB*b_sum-ITB2_OM*a_sum-pqr[1]
        betas_dot[1] = -ITB*a_sum+ITB2_OM*b_sum-pqr[0]

        ### Transmission dynamics of MR
        psi_mr_dot = np.zeros(1)
        psi_mr_dot[0] = self.MR['OMEGA']

        ### Compute main rotor force and moment components
        X_MR=-thrust_mr*(betas[0]-self.MR['IS'])
        Y_MR=thrust_mr*betas[1]
        Z_MR=-thrust_mr
        L_MR=Y_MR*self.MR['H']+DL_DB1*betas[1]+DL_DA1*(betas[0]+lon-self.MR['K1']*betas[1])
        M_MR=Z_MR*self.MR['D']-X_MR*self.MR['H']+DL_DB1*betas[0]+DL_DA1*(-betas[1]+lat-self.MR['K1']*betas[0])
        N_MR=torque_mr

        force_mr = np.array([X_MR,Y_MR,Z_MR])
        moment_mr = np.array([L_MR, M_MR, N_MR])
        return force_mr, moment_mr, power_mr, betas_dot, vi_mr_dot, psi_mr_dot

    def _calc_tr_fm(self, rho, pedal, uvw_air, pqr, vi_tr, psi_tr):
        """Calculate Forces and Moments caused by Tail Rotor
        """

        ## TR Force Moments and inflow dynamics.
        v_adv_2 = (uvw_air[2]+pqr[1]*self.TR['D'])**2 + uvw_air[0]**2
        vr = -(uvw_air[1] - pqr[2]*self.TR['D'] + pqr[0]*self.TR['H']) # vel re rotor plane
        vb = vr + 2/3*self.TR['V_TIP']*(pedal+0.75*self.TR['TWST']) + \
            v_adv_2/self.TR['V_TIP']*(pedal+0.5*self.TR['TWST'])# vel re blade plane (equivalent)
        
        thrust_tr = (vb - vi_tr[0])*rho*self.TR['COEF_TH']/4
        vi_tr_dot = np.zeros(1)
        vi_tr_dot[0] = math.pi*3/4/self.TR['R']*(thrust_tr/(2*math.pi*rho*self.TR['R']**2) - vi_tr[0]*np.sqrt(v_adv_2+(vr-vi_tr[0])**2))
        vi_tr_dot[0] *= 0.5 # slow down inflow dynamics due to numerical unstability.

        ### Transmission dynamics of TR
        psi_tr_dot = np.zeros(1)
        psi_tr_dot[0] = self.TR['OMEGA']

        power_tr = thrust_tr*(vi_tr[0]-vr)
        # torque=power_tr/self.TR['OMEGA'];

        ### Compute tail rotor force and moment components
        Y_TR=thrust_tr
        L_TR=Y_TR*self.TR['H']
        N_TR=-Y_TR*self.TR['D']
        force_tr = np.array([0,Y_TR,0])
        moment_tr = np.array([L_TR, 0, N_TR])
        return force_tr, moment_tr, power_tr, vi_tr_dot, psi_tr_dot

    def _calc_fus_fm(self, rho, uvw_air, vi_mr):
        """Calculate Forces and Moments caused by Fuselage
        """
        wa_fus = uvw_air[2]-vi_mr[0] # Include rotor downwash on fuselage
        wa_fus += (wa_fus>0)*EPS # Make it nonzero!
        d_fw=(uvw_air[0]/(-wa_fus)*(self.MR['H']-self.FUS['H']))-(self.FUS['D']-self.MR['D']) # Pos of downwash on fuselage
        X_FUS = 0.5*rho*self.FUS['XUU']*np.abs(uvw_air[0])*uvw_air[0]
        Y_FUS = 0.5*rho*self.FUS['YVV']*np.abs(uvw_air[1])*uvw_air[1]
        Z_FUS = 0.5*rho*self.FUS['ZWW']*np.abs(wa_fus)*wa_fus
        L_FUS = Y_FUS*self.FUS['H']
        M_FUS = Z_FUS*d_fw-X_FUS*self.FUS['H']
        # Fuselage power consumption
        power_parasite=-X_FUS*uvw_air[0]-Y_FUS*uvw_air[1]-Z_FUS*wa_fus
        power_fus=power_parasite
        force_fus = np.array([X_FUS,Y_FUS,Z_FUS])
        moment_fus = np.array([L_FUS, M_FUS, 0])        
        return force_fus, moment_fus, power_fus

    def _calc_ht_fm(self, rho, uvw_air, pqr, vi_mr):
        """Calculate Forces and Moments caused by Horizontal Tail
        """
        # downwash impinges on tail?
        v_dw = np.max([vi_mr[0]-uvw_air[2], EPS])
        d_dw=(uvw_air[0]/v_dw*(self.MR['H']-self.HT['H'])) \
            - (self.HT['D']-self.MR['D']-self.MR['R'])
        
        if d_dw >0 and d_dw<self.MR['R']: #Triangular downwash
            eps_ht=2*(1-d_dw/self.MR['R'])
        else:
            eps_ht=0
        
        wa_ht = uvw_air[2]-eps_ht*vi_mr[0]+self.HT['D']*pqr[1] # local z-vel at h.t
        vta_ht = np.sqrt(uvw_air[0]**2+uvw_air[1]**2+wa_ht**2) # 
        if np.abs(wa_ht) > 0.3*np.abs(uvw_air[0]): # surface stalled ?
            Z_HT=0.5*rho*self.HT['ZMAX']*np.abs(vta_ht)*wa_ht # circulation
        else:
            Z_HT=0.5*rho*(self.HT['ZUU']*np.abs(uvw_air[0])*uvw_air[0]+self.HT['ZUW']*np.abs(uvw_air[0])*wa_ht) # circulation
        
        M_HT = Z_HT*self.HT['D'] # pitching moment
        force_ht = np.array([0,0,Z_HT])
        moment_ht = np.array([0, M_HT, 0])        
        return force_ht, moment_ht

    def _calc_vt_fm(self, rho, uvw_air, pqr, vi_tr):
        """Calculate Forces and Moments caused by Vertical Tail
        """
        va_vt=uvw_air[1]+vi_tr[0]-self.VT['D']*pqr[2]
        vta_vt=np.sqrt(uvw_air[0]**2+va_vt**2)

        if np.abs(va_vt) > 0.3*np.abs(uvw_air[0]):
            Y_VT=0.5*rho*self.VT['YMAX']*np.abs(vta_vt)*va_vt
        else:
            Y_VT=0.5*rho*(self.VT['YUU']*np.abs(uvw_air[0])*uvw_air[0]+self.VT['YUV']*np.abs(uvw_air[0])*va_vt)
        
        L_VT=Y_VT*self.VT['H']
        N_VT=-Y_VT*self.VT['D']
        force_vt = np.array([0,Y_VT,0])
        moment_vt = np.array([L_VT, 0, N_VT])        
        return force_vt, moment_vt

    def _calc_wn_fm(self, rho, uvw_air, vi_mr):
        """Calculate Forces and Moments caused by Wing
        """
        ## Wing
        wa_wn= uvw_air[2]-vi_mr[0] # local z-vel at wing
        vta_wn=np.sqrt(uvw_air[0]**2+wa_wn**2)

        if np.abs(wa_wn) > 0.3*np.abs(uvw_air[0]): # surface stalled ?
            Z_WN=0.5*rho*self.WN['ZMAX']*np.abs(vta_wn)*wa_wn
        else:
            Z_WN=0.5*rho*(self.WN['ZUU']*uvw_air[0]**2+self.WN['ZUW']*uvw_air[0]*wa_wn)
        
        X_WN=-0.5*rho/math.pi/vta_wn**2*(self.WN['ZUU']*uvw_air[0]**2+self.WN['ZUW']*uvw_air[0]*wa_wn)**2 # ? induced drag
        power_wn=np.abs(X_WN*uvw_air[0]) # wing power
        force_wn = np.array([X_WN,0,Z_WN])
        moment_wn = np.array([0, 0, 0])        
        return force_wn, moment_wn, power_wn

    def dynamics(self, state, action, set_observation=False):
        #
        state_dots = self.state_dots
        #
        vi_mr = state['vi_mr']
        vi_tr = state['vi_tr']
        psi_mr = state['psi_mr']
        psi_tr = state['psi_tr']
        betas = state['betas']
        uvw = state['uvw']
        pqr = state['pqr']
        euler = state['euler']
        xyz = state['xyz']

        ### Control input calculations 
        coll = D2R*( self.HELI['COL_OS'] + 
            0.5*action[0]*(self.HELI['COL_H'] - self.HELI['COL_L']) + 
            0.5*(self.HELI['COL_H'] + self.HELI['COL_L']) )
        lon = D2R*( 0.5*action[1]*(self.HELI['LON_H'] - self.HELI['LON_L']) + 
            0.5*(self.HELI['LON_H'] + self.HELI['LON_L']) )
        lat = D2R*( 0.5*action[2]*(self.HELI['LAT_H'] - self.HELI['LAT_L']) + 
            0.5*(self.HELI['LAT_H'] + self.HELI['LAT_L']) )
        pedal = D2R*( self.HELI['PED_OS'] + 0.5*action[3]*(self.HELI['PED_H'] - self.HELI['PED_L']) + 
            0.5*(self.HELI['PED_H'] + self.HELI['PED_L']) )

        ###  Kinematic calculations
        earth2body = euler_to_rotmat(euler) # Earth to Body DCM matrix
        body2earth = earth2body.transpose() #  Body to Earth DCM matrix

        pqr_to_eulerdot = pqr_to_eulerdot_mat(euler) # par to eulerdot function.
        euler_dot = pqr_to_eulerdot@pqr # calculated eulerdot..
        ned_vel = body2earth@uvw # ned velocity 

        ###  Airspeed calculations
        uvw_air = uvw - earth2body@self.WIND_NED
      
        #### Some Observations ####
        power_climb = self.HELI['WT']*(-ned_vel[2]) # Climbing power [hp]

        ### Atmosphere calculations
        temperature, rho = self._altitude_to_air_properties(-xyz[2])
        ### Main Rotor
        force_mr, moment_mr, power_mr, betas_dot, vi_mr_dot, psi_mr_dot = self._calc_mr_fm(rho, coll, lon, lat, betas, uvw_air, pqr, vi_mr, psi_mr)
        force_tr, moment_tr, power_tr, vi_tr_dot, psi_tr_dot = self._calc_tr_fm(rho, pedal, uvw_air, pqr, vi_tr, psi_tr)
        force_fus, moment_fus, power_fus = self._calc_fus_fm(rho, uvw_air, vi_mr)
        force_ht, moment_ht = self._calc_ht_fm(rho, uvw_air, pqr, vi_mr)
        force_vt, moment_vt = self._calc_vt_fm(rho, uvw_air, pqr, vi_tr)
        force_wn, moment_wn, power_wn = self._calc_wn_fm(rho, uvw_air, vi_mr)

        # Other power consumptions are counted for main rotor torque
        power_extra_mr = power_climb + power_fus
        extra_mr_torque = power_extra_mr / self.MR['OMEGA']
        moment_mr[2] += extra_mr_torque

        power_total = power_mr + power_tr + power_extra_mr + 550*self.HELI['HP_LOSS'] 
        
        force_gravity = earth2body@np.array([0,0,self.HELI['WT']])
        force_total = force_mr + force_tr + force_fus + force_ht + force_vt + force_wn + force_gravity
        moment_total = moment_mr + moment_tr + moment_fus + moment_ht + moment_vt + moment_wn

        if self._does_hit_ground(-xyz[2]):
            w = 5.0
            zeta = 2.0
            K = w**2 * self.HELI["M"]
            C =  2 * zeta * self.HELI["M"] * w
            cxdot = C * ned_vel[2]
            kx = K * (xyz[2] + self.ground_touching_altitude())
            force_ground = earth2body@ np.array([0.0, 0.0, -(cxdot + kx) + EPS])
            force_total += force_ground

        body_acc = force_total/self.HELI['M']
        uvw_dot = body_acc - np.cross(pqr, uvw)
        pqr_dot = np.linalg.inv(self.HELI['I'])@(moment_total - np.cross(pqr, self.HELI['I']@pqr))
        xyz_dot = ned_vel

        ### State derivatives
        state_dots['vi_mr'] = vi_mr_dot
        state_dots['vi_tr'] = vi_tr_dot
        state_dots['psi_mr'] = psi_mr_dot
        state_dots['psi_tr'] = psi_tr_dot
        state_dots['betas'] = betas_dot
        state_dots['uvw'] = uvw_dot
        state_dots['pqr'] = pqr_dot
        state_dots['euler'] = euler_dot
        state_dots['xyz'] = xyz_dot

        if set_observation:
            ### Observation calculations
            power_total_hp = power_total/550 # [hp] Power consumption in Horse Power
            tas = np.linalg.norm(uvw_air) # true air speed in ft/s
            sideslip_deg = R2D*np.arcsin(uvw_air[1]/(tas+EPS))# [deg] Sideslip angle
            aoa_deg = R2D*np.arctan2(uvw_air[2], (uvw_air[0]+EPS)) # [deg] % Angle of Attack
            alt_gr = -xyz[2] - self.__get_ground_height_from_hmap()
            # These two are not need for now. 
            #ground_speed = np.linalg.norm(ned_vel[:2]) # [ft/s] Ground speed
            #track_angle_deg = R2D*np.arctan2(ned_vel[1],ned_vel[0]) # [deg] Track angle
            
            self.observation = np.array([power_total_hp, \
                tas, aoa_deg, sideslip_deg, \
                ned_vel[0], ned_vel[1], ned_vel[2], \
                R2D*euler[0], R2D*euler[1], R2D*euler[2], \
                R2D*pqr[0], R2D*pqr[1], R2D*pqr[2],
                body_acc[0], body_acc[1], body_acc[2], \
                xyz[0], xyz[1], -xyz[2], alt_gr],
                )

        return state_dots

    def trim(self, params):
        """This function trims the helicopter given the parameters.
        If necessary, user can specify velocity of helicopter in earth frame along with
        yaw_rate which should be deg/sec. Other parameters like rotor azimuths, yaw angle and 
        north east locations and ground altitude can also be specified.
        """
        # First, fix some parameters which are not iterated through trim algorithm.
        # However, these parameters will affect the trim.
        self.state['euler'][-1] = params["yaw"]
        self.state['psi_mr'][0] = params["psi_mr"]
        self.state['psi_tr'][0] = params["psi_tr"]
        cg_from_bottom = -self.ground_touching_altitude()
        self.state['xyz'][0] = params["xy"][0]
        self.state['xyz'][1] = params["xy"][1]
        self.state['xyz'][2] = cg_from_bottom-params["gr_alt"]
        self.last_action = np.zeros(4)  

        n_vars = 16
        y_target = np.zeros(n_vars, dtype=np.float)
        y_target[-4] = params['yaw_rate']
        y_target[-3:] = np.array(params['ned_vel'], dtype=np.float)/self.MR['R']
        uvw0 = np.array(params['ned_vel'], dtype=np.float)/self.MR['V_TIP']
        x = np.array([0.05, 0.05, 0, 0, # vi_mr, vi_tr, betas
            uvw0[-3], uvw0[-2], uvw0[-1], # uvw
            0, 0, y_target[-4], # pqr
            -0.01, 0.01, # phi, theta
            0.0, 0.0, 0.0, 0.0, # actions
            ], dtype=np.float)

        y = self.__trim_fcn(x)
        tol = (y-y_target).transpose()@(y-y_target)
        while tol>EPS**2:
            dydx = []
            for i in range(n_vars):
                dxi = np.zeros(n_vars); dxi[i]+=EPS
                dydxi = (self.__trim_fcn(x+dxi)-self.__trim_fcn(x-dxi))/(2*EPS)
                dydx.append(dydxi)

            dydx = np.stack(dydx, axis=-1)
            x = x - 0.4*np.linalg.inv(dydx)@(y-y_target) # decrease step by multiplying 0.2 to guarantee convergence.
            y = self.__trim_fcn(x)
            tol = (y-y_target).transpose()@(y-y_target)

        # Finalize the trim algorithm by assigning solved states to the system.
        self.state['vi_mr'] = x[0:1]*self.MR['V_TIP']
        self.state['vi_tr'] = x[1:2]*self.TR['V_TIP']
        self.state['betas'] = x[2:4]
        self.state['uvw'] = x[4:7]*self.MR['V_TIP']
        self.state['pqr'] = x[7:10]*self.MR['OMEGA']
        self.state['euler'][:-1] = x[10:12]
        self.last_action = x[12:16]
        # set state dots
        self.state_dots = self.dynamics(self.state, self.last_action, set_observation=True)

    def __trim_fcn(self, x):
        state = copy.deepcopy(self.state)
        state['vi_mr'] = x[0:1]*self.MR['V_TIP']
        state['vi_tr'] = x[1:2]*self.TR['V_TIP']
        state['betas'] = x[2:4]
        state['uvw'] = x[4:7]*self.MR['V_TIP']
        state['pqr'] = x[7:10]*self.MR['OMEGA']
        state['euler'][:-1] = x[10:12]
        action = x[12:16]

        state_dots = self.dynamics(state, action)
        y = np.concatenate([state_dots['vi_mr']/self.MR['V_TIP'],
                            state_dots['vi_tr']/self.TR['V_TIP'],
                            state_dots['betas'],
                            state_dots['uvw']/self.MR['V_TIP'],
                            state_dots['pqr']/self.MR['OMEGA'],
                            state_dots['euler'],
                            state_dots['xyz']/self.MR['R']])

        return y
