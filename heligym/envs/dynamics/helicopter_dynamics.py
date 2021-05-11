import yaml
import sys, math
import numpy as np
import os
import copy

from .kinematic import euler_to_rotmat, pqr_to_eulerdot_mat
from .dynamics import DynamicSystem, State

FTS2KNOT    = 0.5924838 # ft/s to knots conversion
EPS         = 1e-4 # small value for divison by zero
R2D         = 180/math.pi # Rad to deg
D2R         = 1/R2D

class HelicopterDynamics(DynamicSystem):

    _observations = ["POWER", "TAS", "AOA", "SSLIP", "GROUND_SPD", "TRACK", "CLIMB_RATE", 
        "ROLL", "PITCH", "YAW", "ROLL_RATE", "PITCH_RATE", "YAW_RATE", 
        "ACC_LON", "LAT_ACC", "DWN_ACC", "XPOS", "YPOS", "ALTITUDE"]
    
    _default_yaml = os.path.join(os.path.dirname(__file__), "..", "helis", "a109.yaml")
    @classmethod
    def init_yaml(cls, yaml_path: str = None, dt=0.01):
        yaml_path = cls._default_yaml if yaml_path is None else yaml_path
        with open(yaml_path) as foo:
            params = yaml.safe_load(foo)

        return cls(params, dt)

    def __init__(self, params, dt):
        super(HelicopterDynamics, self).__init__(dt)
        self.HELI = params['HELI']
        self.ENV = params['ENV']
        # Null state dots, since it is not updated.
        self.__precalculations()
        self.set_wind() # wind velocity in earth frame:
        self.register_states()

    def register_states(self):
        self.register_state('vi_mr', np.array([30.0]))
        self.register_state('vi_tr', np.array([40.0]))
        self.register_state('betas', np.array([0.0, 0.0]))
        self.register_state('uvw', np.array([0.0, 0.0, 0.0]))
        self.register_state('pqr', np.array([0.0, 0.0, 0.0]))
        self.register_state('euler', np.array([0.0, 0.0, 0.0]))
        cg_from_bottom = -self._ground_touching_altitude()
        self.register_state('xyz', np.array([0.0, 0.0, cg_from_bottom-100]))
        self.last_action = np.zeros(4)

    def reset(self):
        self.trim()
        print("Helicopter is trimmed!")

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

    def _does_hit_ground(self, altitude):
        return altitude -self.HELI['WL_CG']/12 - self.ENV['GR_ALT'] < 0.0

    def _ground_touching_altitude(self):
        return self.HELI['WL_CG']/12 - EPS # divide by 12 to make inch to feet

    def _calc_mr_fm(self, rho, coll, lon, lat, betas, uvw_air, pqr, vi_mr):
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

        ## MR Force Moments
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
        a_sum = betas[1]-lon+KC*betas[0]+DB1DV*uvw_air[1]*(1+wake_fn)
        b_sum = betas[0]+lat-KC*betas[1]+DA1DU*uvw_air[0]*(1+2*wake_fn)
        betas_dot = np.zeros(2)
        betas_dot[0] = -ITB*b_sum-ITB2_OM*a_sum-pqr[1]
        betas_dot[1] = -ITB*a_sum+ITB2_OM*b_sum-pqr[0]

        ## Compute main rotor force and moment components
        X_MR=-thrust_mr*(betas[0]-self.MR['IS'])
        Y_MR=thrust_mr*betas[1]
        Z_MR=-thrust_mr
        L_MR=Y_MR*self.MR['H']+DL_DB1*betas[1]+DL_DA1*(betas[0]+lat-self.MR['K1']*betas[1])
        M_MR=Z_MR*self.MR['D']-X_MR*self.MR['H']+DL_DB1*betas[0]+DL_DA1*(-betas[1]+lon-self.MR['K1']*betas[0])
        N_MR=torque_mr

        force_mr = np.array([X_MR,Y_MR,Z_MR])
        moment_mr = np.array([L_MR, M_MR, N_MR])
        return force_mr, moment_mr, power_mr, vi_mr_dot, betas_dot

    def _calc_tr_fm(self, rho, pedal, uvw_air, pqr, vi_tr):
        """Calculate Forces and Moments caused by Tail Rotor
        """
        v_adv_2 = (uvw_air[2]+pqr[1]*self.TR['D'])**2 + uvw_air[0]**2
        vr = -(uvw_air[1] - pqr[2]*self.TR['D'] + pqr[0]*self.TR['H']) # vel re rotor plane
        vb = vr + 2/3*self.TR['V_TIP']*(pedal+0.75*self.TR['TWST']) + \
            v_adv_2/self.TR['V_TIP']*(pedal+0.5*self.TR['TWST'])# vel re blade plane (equivalent)
        
        thrust_tr = (vb - vi_tr[0])*rho*self.TR['COEF_TH']/4
        vi_tr_dot = np.zeros(1)
        vi_tr_dot[0] = math.pi*3/4/self.TR['R']*(thrust_tr/(2*math.pi*rho*self.TR['R']**2) - vi_tr[0]*np.sqrt(v_adv_2+(vr-vi_tr[0])**2))
        vi_tr_dot[0] *= 0.5 # slow down inflow dynamics due to numerical unstability.

        power_tr = thrust_tr*(vi_tr[0]-vr)
        # torque=power_tr/self.TR['OMEGA'];
        ## Compute tail rotor force and moment components
        Y_TR=thrust_tr
        L_TR=Y_TR*self.TR['H']
        N_TR=-Y_TR*self.TR['D']
        force_tr = np.array([0,Y_TR,0])
        moment_tr = np.array([L_TR, 0, N_TR])
        return force_tr, moment_tr, power_tr, vi_tr_dot

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
        climb_rate = -ned_vel[2] # [ft/s] ascending rate (descending if negative)
        power_climb = self.HELI['WT']*climb_rate # Climbing power [hp]
        altitude = -xyz[2] # [ft] altitude

        ### Atmosphere calculations
        temperature, rho = self._altitude_to_air_properties(altitude)
        ### Main Rotor
        force_mr, moment_mr, power_mr, vi_mr_dot, betas_dot = self._calc_mr_fm(rho, coll, lon, lat, betas, uvw_air, pqr, vi_mr)
        force_tr, moment_tr, power_tr, vi_tr_dot = self._calc_tr_fm(rho, pedal, uvw_air, pqr, vi_tr)
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

        if self._does_hit_ground(altitude):
            force_total_on_earth = body2earth@force_total
            force_ground = earth2body@np.array([0,0,-force_total_on_earth[2] + EPS])
            force_total += force_ground

        body_acc = force_total/self.HELI['M']
        uvw_dot = body_acc - np.cross(pqr, uvw)
        pqr_dot = np.linalg.inv(self.HELI['I'])@(moment_total - np.cross(pqr, self.HELI['I']@pqr))
        xyz_dot = ned_vel

        ### State derivatives
        state_dots['vi_mr'] = vi_mr_dot
        state_dots['vi_tr'] = vi_tr_dot
        state_dots['betas'] = betas_dot
        state_dots['uvw'] = uvw_dot
        state_dots['pqr'] = pqr_dot
        state_dots['euler'] = euler_dot
        state_dots['xyz'] = xyz_dot

        if set_observation:
            ### Observation calculations
            power_total_hp = power_total/550 # [hp] Power consumption in Horse Power
            tas = np.linalg.norm(uvw_air) # true air speed in ft/s
            sideslip_deg = R2D*np.arcsin(uvw_air[1]/(tas+EPS))# [rad] Sideslip angle
            aoa_deg = R2D*np.arctan2(uvw_air[2], (uvw_air[0]+EPS)) # [rad] % Angle of Attack
            ground_speed = np.linalg.norm(ned_vel[:2]) # [ft/s] Ground speed
            track_angle_deg = R2D*np.arctan2(ned_vel[1],ned_vel[0]) # [rad] Track angle
            #
            self.observation = np.array([power_total_hp, \
                tas, aoa_deg, sideslip_deg, \
                ground_speed, track_angle_deg, climb_rate, \
                R2D*euler[0], R2D*euler[1], R2D*euler[2], \
                R2D*pqr[0], R2D*pqr[1], R2D*pqr[2],
                body_acc[0], body_acc[1], body_acc[2], \
                xyz[0], xyz[1], altitude],
                )

        return state_dots

    def trim(self):
        n_states = 10
        x = np.array([0.05, 0.05, 0, 0, 0, 0, 0.5, 0.5, 0.5, 0.5])
        y = self.trim_fcn(x)
        tol = y.transpose()@y
        while tol>EPS**2:
            dydx = []
            for i in range(n_states):
                dxi = np.zeros(n_states); dxi[i]+=EPS
                dydxi = (self.trim_fcn(x+dxi)-self.trim_fcn(x-dxi))/(2*EPS)
                dydx.append(dydxi)

            dydx = np.stack(dydx, axis=-1)
            x = x - 0.2*np.linalg.inv(dydx)@y
            y = self.trim_fcn(x)
            tol = y.transpose()@y

        self.state['vi_mr'] = x[0:1]*self.MR['V_TIP']
        self.state['vi_tr'] = x[1:2]*self.TR['V_TIP']
        self.state['betas'] = x[2:4]
        self.state['euler'][:-1] = x[4:6]
        self.last_action = x[6:10]
        # set state dots
        self.state_dots = self.dynamics(self.state, self.last_action, set_observation=True)

    def trim_fcn(self, x):
        state = copy.deepcopy(self.state)
        state['vi_mr'] = x[0:1]*self.MR['V_TIP']
        state['vi_tr'] = x[1:2]*self.TR['V_TIP']
        state['betas'] = x[2:4]
        state['euler'][:-1] = x[4:6]
        action = x[6:10]

        state_dots = self.dynamics(state, action)
        y = np.concatenate([state_dots['vi_mr']/self.MR['V_TIP'],
                            state_dots['vi_tr']/self.TR['V_TIP'],
                            state_dots['betas'],
                            state_dots['uvw'],
                            state_dots['pqr']/self.MR['OMEGA']])

        return y


