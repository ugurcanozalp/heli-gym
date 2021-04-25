import yaml
from typing import List
import sys, math
import numpy as np
import os

from .kinematic import euler_to_rotmat, pqr_to_eulerdot_mat
from .dynamics import DynamicSystem, State

FPS         = 100.0
DT          = 1/FPS
FTS2KNOT    = 0.5924838; # ft/s to knots conversion
EPS         = 1e-6; # small value for divison by zero
R2D         = 180/math.pi; # Rad to deg
D2R         = 1/R2D;

class HelicopterDynamics(DynamicSystem):

    _observations = ["TAS", "AOA", "SSLIP", "GROUND_SPD", "TRACK", "CLIMB_RATE", 
        "ROLL", "PITCH", "YAW", "ROLL_RATE", "PITCH_RATE", "YAW_RATE", 
        "ACC_LON", "LAT_ACC", "DWN_ACC", "XPOS", "YPOS", "ALTITUDE", "POWER"]
    
    _default_yaml = os.path.join(os.path.dirname(__file__), "a109_param.yaml")
    @classmethod
    def init_yaml(cls, yaml_path: str = None):
        yaml_path = self._default_yaml if yaml_path is None else yaml_path
        with open(yaml_path) as foo:
            params = yaml.safe_load(foo)

        return cls(params)

    def __init__(self, params):
        super(HelicopterDynamics, self).__init__(DT)
        self.__dict__.update(params)
        # Null state dots, since it is not updated.
        self.reset()
        self.__precalculations()
        self.set_wind() # wind velocity in earth frame:

    def reset(self):
        self.register_state('betas', np.array([0.0, 0.0]))
        self.register_state('uvw', np.array([0.0, 0.0, 0.0]))
        self.register_state('pqr', np.array([0.0, 0.0, 0.0]))
        self.register_state('euler', np.array([0.0, 0.0, 0.0]))
        cg_from_bottom = -self._ground_touching_altitude()
        self.register_state('xyz', np.array([0.0, 0.0, cg_from_bottom]))     
        self.last_action = np.zeros(4)       

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
        self.MR['H'] = (self.MR['WL']-self.HELI['WL_CG'])/12;  # [ft] 
        self.MR['D'] = (self.MR['FS']-self.HELI['FS_CG'])/12;
        self.FUS['H'] = (self.FUS['WL']-self.HELI['WL_CG'])/12;
        self.FUS['D'] = (self.FUS['FS']-self.HELI['FS_CG'])/12;
        self.WN['H']  = (self.WN['WL']-self.HELI['WL_CG'])/12;
        self.WN['D']  = (self.WN['FS']-self.HELI['FS_CG'])/12;
        self.HT['H']  = (self.HT['WL']-self.HELI['WL_CG'])/12;
        self.HT['D']  = (self.HT['FS']-self.HELI['FS_CG'])/12;
        self.VT['H']  = (self.VT['WL']-self.HELI['WL_CG'])/12;
        self.VT['D']  = (self.VT['FS']-self.HELI['FS_CG'])/12;
        self.TR['H']  = (self.TR['WL']-self.HELI['WL_CG'])/12;
        self.TR['D']  = (self.TR['FS']-self.HELI['FS_CG'])/12;
        #
        self.HELI['M']=self.HELI['WT']/self.ENV['GRAV']; # [slug] vehicle mass
        # Main Rotor precalculations
        self.MR['OMEGA'] = self.MR['RPM']*2*math.pi/60; # [rad/s] MR rev speed
        self.MR['V_TIP'] = self.MR['R']*self.MR['OMEGA']; # [ft/s] MR tip speed
        self.MR['FR'] = self.MR['CD0']*self.MR['R']*self.MR['B']*self.MR['C']; # eff.frontal area MR
        self.MR['SOL'] = self.MR['B']*self.MR['C']/(self.MR['R']*math.pi); # MR solidity (SIGMA)
        self.MR['A_SIGMA'] = self.MR['A']*self.MR['SOL']; # product(lift-curve-slope & solidity)
        # Tail Rotor precalculations
        self.TR['OMEGA'] = self.TR['RPM']*2*math.pi/60; # [rad/s] TR rev speed
        self.TR['FR'] = self.TR['CD0']*self.TR['R']*self.TR['B']*self.TR['C']; # eff.frontal area TR
        self.TR['SOL'] = self.TR['B']*self.TR['C']/(self.TR['R']*math.pi); # TR solidity (SIGMA)
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
        temp = self.ENV['T0'] - self.ENV['LAPSE']*altitude; # [R] Temperature at the current altitude
        rho = self.ENV['RO_SEA']*(temp/self.ENV['T0'])**((self.ENV['GRAV']/(self.ENV['LAPSE']*self.ENV['R']))-1.0); # [slug/ft**3]
        return temp, rho

    def _does_hit_ground(self, altitude):
        return altitude -self.HELI['WL_CG']/12 - self.ENV['GR_ALT'] > 0.0

    def _ground_touching_altitude(self):
        return self.HELI['WL_CG']/12 - EPS # divide by 12 to make inch to feet

    def _calc_mr_fm(self, rho, coll, lon, lat, betas, uvw_air, pqr):
        """Calculate Forces and Moments caused by Main Rotor
        ans Main Rotor Dynamics
        """
        ### Calculate required parameters first. 
        # one sixth the product(lock# and rotor ang.rate)
        GAM_OM16 = rho*self.MR['A']*self.MR['C']*self.MR['R']**4/self.MR['IB']* \
            self.MR['OMEGA']/16*(1+8/3*self.MR['E']/self.MR['R'])
        # flapping aero cpl(flapping coupling factor)    
        KC = (0.75*self.MR['OMEGA']*self.MR['E']/self.MR['R']/GAM_OM16)+self.MR['K1']; 
        # flapping x-cpl coef
        ITB2_OM = self.MR['OMEGA']/(1+(self.MR['OMEGA']/GAM_OM16)**2);   
        # flapping primary resp(inverse TPP lag) [rad/s]
        ITB = ITB2_OM*self.MR['OMEGA']/GAM_OM16; 
        # primary(direct)flapping stiffness [rad/sec2]
        DL_DB1 = self.MR['B']/2* \
            (1.5*self.MR['IB']*self.MR['E']/self.MR['R']*(self.MR['OMEGA'])**2);  
        # cross(off-axis)flapping stiffness [rad/sec2]
        DL_DA1 = 0.5*rho*self.MR['A']*self.MR['B']*self.MR['C']*self.MR['R']*self.MR['V_TIP']**2*self.MR['E']/6;    
        # thrust coeff.
        CT = self.HELI['WT']/(rho*math.pi*self.MR['R']**2*self.MR['V_TIP']**2); 
        ## Dihedral effect on TPP
        # TPP dihedral effect(late.flap2side vel)
        DB1DV = 2/self.MR['OMEGA']/self.MR['R']*(8*CT/self.MR['A_SIGMA']+(math.sqrt(CT/2))); 
        DA1DU = -DB1DV; # TPP pitchup with speed   

        wake_fn = 1/(1+np.exp(10*(np.abs(uvw_air[0])-self.HELI['VTRANS'])/self.HELI['VTRANS']))
        ### MR TPP Dynamics
        a_sum = betas[1]-lon+KC*betas[0]+DB1DV*uvw_air[1]*(1+wake_fn)
        b_sum = betas[0]+lat-KC*betas[1]+DA1DU*uvw_air[0]*(1+2*wake_fn)
        betas_dot = np.zeros(2)
        betas_dot[0] = -ITB*b_sum-ITB2_OM*a_sum-pqr[1]
        betas_dot[1] = -ITB*a_sum+ITB2_OM*b_sum-pqr[0]

        ## MR Force Moments
        wr = uvw_air[2] + (betas[0]-self.MR['IS'])*uvw_air[0] - betas[1]*uvw_air[1]; # z-axis vel re rotor plane
        wb = wr + 2/3*self.MR['OMEGA']*self.MR['R']*(coll+0.75*self.MR['TWST']); # z-axis vel re blade (equivalent)
        
        COEF = self.MR['OMEGA']*self.MR['A']*self.MR['B']*self.MR['C']
        b,c = COEF/(8*math.pi), -wb*COEF/(8*math.pi)
        vi_mr = 0.5*(-b + np.sqrt(np.max([0, b**2 + 4*c]))) # Initialization
        
        for i in range(20):
            thrust_mr = (wb - vi_mr) * rho*(self.MR['R']**2*COEF/4);
            v_hat_2 = uvw_air[0]**2 + uvw_air[1]**2 + wr*(wr-2*vi_mr)
            vi_2 = np.sqrt( (v_hat_2/2)**2 + (thrust_mr/(2*math.pi*rho*self.MR['R']**2))**2 ) - v_hat_2
            vi_mr = np.sqrt(np.abs(vi_2))

        # MR induced flow power consumption
        induced_power=thrust_mr*vi_mr;
        # MR profile drag power consumption
        profile_power=0.5*rho*(self.MR['FR']/4)*self.MR['OMEGA']*self.MR['R']*(self.MR['OMEGA']**2*self.MR['R']**2+ \
                     4.6*(uvw_air[0]**2+uvw_air[1]**2))
        power_mr=induced_power+profile_power
        torque_mr=power_mr/self.MR['OMEGA']

        ## Compute main rotor force and moment components
        X_MR=-thrust_mr*(betas[0]-self.MR['IS'])
        Y_MR=thrust_mr*betas[1]
        Z_MR=-thrust_mr
        L_MR=Y_MR*self.MR['H']+DL_DB1*betas[1]+DL_DA1*(betas[0]+lat-self.MR['K1']*betas[1])
        M_MR=Z_MR*self.MR['D']-X_MR*self.MR['H']+DL_DB1*betas[0]+DL_DA1*(-betas[1]+lon-self.MR['K1']*betas[0])
        N_MR=torque_mr

        force_mr = np.array([X_MR,Y_MR,Z_MR])
        moment_mr = np.array([L_MR, M_MR, N_MR])
        return force_mr, moment_mr, power_mr, vi_mr, betas_dot

    def _calc_tr_fm(self, rho, pedal, uvw_air, pqr):
        """Calculate Forces and Moments caused by Tail Rotor
        """
        vr = -(uvw_air[1] - pqr[2]*self.TR['D'] + pqr[0]*self.TR['H']); # vel re rotor plane
        vb = vr + 2/3*self.TR['OMEGA']*self.TR['R']*(pedal+0.75*self.TR['TWST']); # vel re blade plane (equivalent)
        
        COEF = self.TR['OMEGA']*self.TR['A']*self.TR['B']*self.TR['C']
        b,c = COEF/(8*math.pi), -vb*COEF/(8*math.pi)
        vi_tr = 0.5*(-b + np.sqrt(np.max([0, b**2 + 4*c]))) # Initialization
        for i in range(20):
            thrust_tr = (vb - vi_tr)*rho*(self.TR['R']**2*COEF/4);
            v_hat_2 = (uvw_air[2]+pqr[1]*self.TR['D'])**2 + uvw_air[0]**2 + vr*(vr-2*vi_tr)
            vi_2 = np.sqrt( (v_hat_2/2)**2 + (thrust_tr/(2*math.pi*rho*self.TR['R']**2))**2 ) - v_hat_2
            vi_tr = np.sqrt(np.abs(vi_2))

        power_tr = thrust_tr*vi_tr
        # torque=power_tr/self.TR['OMEGA'];
        ## Compute tail rotor force and moment components
        Y_TR=thrust_tr
        L_TR=Y_TR*self.TR['H']
        N_TR=-Y_TR*self.TR['D']
        force_tr = np.array([0,Y_TR,0])
        moment_tr = np.array([L_TR, 0, N_TR])
        return force_tr, moment_tr, power_tr, vi_tr

    def _calc_fus_fm(self, rho, uvw_air, vi_mr):
        """Calculate Forces and Moments caused by Fuselage
        """
        wa_fus = uvw_air[2]-vi_mr; # Include rotor downwash on fuselage
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
        d_dw=(uvw_air[0]/(vi_mr-uvw_air[2])*(self.MR['H']-self.HT['H'])) \
            - (self.HT['D']-self.MR['D']-self.MR['R'])
        
        if d_dw >0 and d_dw<self.MR['R']: #Triangular downwash
            eps_ht=2*(1-d_dw/self.MR['R'])
        else:
            eps_ht=0
        
        wa_ht = uvw_air[2]-eps_ht*vi_mr+self.HT['D']*pqr[1] # local z-vel at h.t
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
        va_vt=uvw_air[1]+vi_tr-self.VT['D']*pqr[2];
        vta_vt=np.sqrt(uvw_air[0]**2+va_vt**2);

        if np.abs(va_vt) > 0.3*np.abs(uvw_air[0]):
            Y_VT=0.5*rho*self.VT['YMAX']*np.abs(vta_vt)*va_vt;
        else:
            Y_VT=0.5*rho*(self.VT['YUU']*np.abs(uvw_air[0])*uvw_air[0]+self.VT['YUV']*np.abs(uvw_air[0])*va_vt);   
        
        L_VT=Y_VT*self.VT['H']; 
        N_VT=-Y_VT*self.VT['D'];
        force_vt = np.array([0,Y_VT,0])
        moment_vt = np.array([L_VT, 0, N_VT])        
        return force_vt, moment_vt

    def _calc_wn_fm(self, rho, uvw_air, vi_mr):
        """Calculate Forces and Moments caused by Wing
        """
        ## Wing
        wa_wn= uvw_air[2]-vi_mr; # local z-vel at wing
        vta_wn=np.sqrt(uvw_air[0]**2+wa_wn**2);

        if np.abs(wa_wn) > 0.3*np.abs(uvw_air[0]): # surface stalled ?
            Z_WN=0.5*rho*self.WN['ZMAX']*np.abs(vta_wn)*wa_wn;
        else:
            Z_WN=0.5*rho*(self.WN['ZUU']*uvw_air[0]**2+self.WN['ZUW']*uvw_air[0]*wa_wn);
        
        X_WN=-0.5*rho/math.pi/vta_wn**2*(self.WN['ZUU']*uvw_air[0]**2+self.WN['ZUW']*uvw_air[0]*wa_wn)**2; # ? induced drag 
        power_wn=np.abs(X_WN*uvw_air[0]); # wing power
        force_wn = np.array([X_WN,0,Z_WN])
        moment_wn = np.array([0, 0, 0])        
        return force_wn, moment_wn, power_wn

    def dynamics(self, state, action):
        #
        state_dots = self.state_dots
        #
        betas = state['betas']
        uvw = state['uvw']
        pqr = state['pqr']
        euler = state['euler']
        xyz = state['xyz']

        ### Control input calculations 
        coll = D2R*( self.HELI['COL_OS'] + action[0]*(self.HELI['COL_H'] - self.HELI['COL_L']) + self.HELI['COL_L'] )
        lon = D2R*( action[1]*(self.HELI['LON_H'] - self.HELI['LON_L']) + self.HELI['LON_L'] )
        lat = D2R*( action[2]*(self.HELI['LAT_H'] - self.HELI['LAT_L']) + self.HELI['LAT_L'] )
        pedal = D2R*( self.HELI['PED_OS'] + action[3]*(self.HELI['PED_H'] - self.HELI['PED_L']) + self.HELI['PED_L'] )

        ###  Kinematic calculations
        earth2body = euler_to_rotmat(state['euler']); # Earth to Body DCM matrix
        body2earth = earth2body.transpose() #  Body to Earth DCM matrix

        pqr_to_eulerdot = pqr_to_eulerdot_mat(euler) # par to eulerdot function.
        euler_dot = pqr_to_eulerdot@pqr # calculated eulerdot..
        ned_vel = body2earth@uvw # ned velocity 

        ###  Airspeed calculations
        uvw_air = uvw - earth2body@self.WIND_NED

        #### Some Observations ####
        phi_deg, theta_deg, psi_deg = R2D*euler[0], R2D*euler[1], R2D*euler[2]
        p_dps, q_dps, r_dps = R2D*pqr[0], R2D*pqr[1], R2D*pqr[2]
        tas = np.linalg.norm(uvw_air) # true air speed in ft/s
        ktas = tas*FTS2KNOT # ktas in knots
        sideslip_deg = R2D*np.arcsin(uvw_air[1]/(tas+EPS));# [deg] Sideslip angle
        aoa_deg = R2D*np.arctan2(uvw_air[2], (uvw_air[0]+EPS)) # [def] % Angle of Attack
        ground_speed = np.linalg.norm(ned_vel[:2]) # [ft/s] Ground speed
        track_angle_deg = R2D*np.arctan2(ned_vel[1],ned_vel[0]) # [deg] Track angle
        climb_rate = -ned_vel[2]; # [ft/s] ascending rate (descending if negative)
        power_climb = self.HELI['WT']*climb_rate # Climbing power [hp]
        altitude = -xyz[2] # [ft] altitude

        ### Atmosphere calculations
        temperature, rho = self._altitude_to_air_properties(altitude)
        ### Main Rotor
        force_mr, moment_mr, power_mr, vi_mr, betas_dot = self._calc_mr_fm(rho, coll, lon, lat, betas, uvw_air, pqr)
        force_tr, moment_tr, power_tr, vi_tr = self._calc_tr_fm(rho, pedal, uvw_air, pqr)
        force_fus, moment_fus, power_fus = self._calc_fus_fm(rho, uvw_air, vi_mr)
        force_ht, moment_ht = self._calc_ht_fm(rho, uvw_air, pqr, vi_mr)
        force_vt, moment_vt = self._calc_vt_fm(rho, uvw_air, pqr, vi_tr)
        force_wn, moment_wn, power_wn = self._calc_wn_fm(rho, uvw_air, vi_mr)

        power_extra_mr = power_climb + power_fus
        extra_mr_torque = power_extra_mr / self.MR['OMEGA']
        moment_mr[2] += extra_mr_torque

        power_total = power_mr + power_tr + power_extra_mr + 550*self.HELI['HP_LOSS'] 
        power_total_hp = power_total/550

        force_gravity = earth2body@np.array([0,0,self.HELI['WT']])
        force_total = force_mr + force_tr + force_fus + force_ht + force_vt + force_wn + force_gravity
        moment_total = moment_mr + moment_tr + moment_fus + moment_ht + moment_vt + moment_wn

        if self._does_hit_ground(altitude):
            force_total_on_earth = body2earth@force_total
            force_ground = earth2body@np.array([0,0,-force_total_on_earth[2] + EPS])
            force_total += force_ground

        body_acc = force_total/self.HELI['M']
        uvw_dot = body_acc - np.cross(pqr, uvw)
        pqr_dot = np.linalg.inv(self.HELI['I'])@(moment_total - np.cross(pqr, self.HELI['I']@pqr))
        xyz_dot = ned_vel

        ### State derivatives
        state_dots['betas'] = betas_dot
        state_dots['uvw'] = uvw_dot
        state_dots['pqr'] = pqr_dot
        state_dots['euler'] = euler_dot
        state_dots['xyz'] = xyz_dot

        ### Observations
        observartion = np.array([power_total_hp, \
            tas, aoa_deg, sideslip_deg, \
            ground_speed, track_angle_deg, climb_rate, \
            phi_deg, theta_deg, psi_deg, \
            p_dps, q_dps, r_dps,
            body_acc[0], body_acc[1], body_acc[2], \
            xyz[0], xyz[1], altitude],
            )

        return state_dots, observartion
       
    def render_text(self):
        obs = self.get_observation()
        text = f""" \t-----SENSOR READINGS-----
            POWER \t\t\t: {obs[0]:5.2f} hp
            TAS \t\t\t: {obs[1]:5.2f} ft/s
            AOA \t\t\t: {obs[2]:5.2f} °
            SSLIP \t\t\t: {obs[3]:5.2f} °
            GRS \t\t\t: {obs[4]:5.2f} ft/s
            TRACK \t\t\t: {obs[5]:5.2f} °
            CLIMB_RATE \t\t\t: {obs[6]:5.2f} ft/s
            ROLL \t\t\t: {obs[7]:5.2f} °
            PITCH \t\t\t: {obs[8]:5.2f} °
            YAW \t\t\t: {obs[9]:5.2f} °
            ROLL_RATE \t\t\t: {obs[10]:5.2f} °/sec
            PITCH_RATE \t\t\t: {obs[11]:5.2f} °/sec
            YAW_RATE \t\t\t: {obs[12]:5.2f} °/sec
            LON_ACC \t\t\t: {obs[13]:5.2f} ft/sec^2
            LAT_ACC \t\t\t: {obs[14]:5.2f} ft/sec^2
            DWN_ACC \t\t\t: {obs[15]:5.2f} ft/sec^2
            X_LOC \t\t\t: {obs[16]:5.2f} ft
            Y_LOC \t\t\t: {obs[17]:5.2f} ft
            Z_LOC \t\t\t: {obs[18]:5.2f} ft

        """
        return print(text, sep=' ', end='', flush=True)
