import numpy as np
from .helicopter import Heli

class HeliHover(Heli):
    def __init__(self, heli_name:str = "aw109"):
        Heli.__init__(self, heli_name=heli_name)
        max_time = Heli.default_max_time
        task_target = {
            "sea_alt": 4000,
            "north_loc": 0,
            "east_loc": 0
        }
        trim_cond = {
            "yaw": 0.0,
            "yaw_rate": 0.0,
            "ned_vel": [0.0, 0.0, 0.0],
            "gr_alt": 10.0,
            "xy": [0.0, 0.0],
            "psi_mr": 0.0,
            "psi_tr": 0.0
        }
        self.set_max_time(max_time)
        self.set_target(task_target)
        self.set_trim_cond(trim_cond)

    def _calculate_reward(self):
        xyz_norm = self.heli_dyn.state["xyz"] / self.normalizers["x"]
        xyzdot_norm = self.heli_dyn.state_dots["xyz"] / self.normalizers["v"]
        pqr_norm = self.heli_dyn.state["pqr"] * self.normalizers["t"] 
        pqrdot_norm = self.heli_dyn.state_dots["pqr"] * self.normalizers["t"]**2

        xyz_target_norm = np.array([self.task_target["north_loc"], self.task_target["east_loc"], -self.task_target["sea_alt"]], dtype=np.float) / self.normalizers["x"]

        pqr_final_reward = - (pqr_norm * pqr_norm).sum()
        pqr_terminal_reward = - (np.sign(pqr_norm) * pqrdot_norm).sum()
        pqr_reward = max(
            pqr_final_reward,
            pqr_terminal_reward)

        xyz_final_reward = - ((xyz_norm - xyz_target_norm) * (xyz_norm - xyz_target_norm)).sum()
        xyz_terminal_reward = - (np.sign(xyz_norm - xyz_target_norm) * xyzdot_norm).sum()

        xyz_reward = max(
            xyz_final_reward,
            xyz_terminal_reward)

        reward = (pqr_reward + xyz_reward) / 2.0

        success_step = pqr_final_reward > -1.0 and xyz_final_reward > -1.0  

        return reward, success_step

class HeliForwardFlight(Heli):

    def __init__(self, heli_name:str = "aw109"):
        Heli.__init__(self, heli_name=heli_name)
        max_time = Heli.default_max_time
        task_target = {
            "sea_alt": 4000,
            "heading": 0,
            "vel": 100
        }
        trim_cond = {
            "yaw": 0.0,
            "yaw_rate": 0.0,
            "ned_vel": [0.0, 0.0, 0.0],
            "gr_alt": 10.0,
            "xy": [0.0, 0.0],
            "psi_mr": 0.0,
            "psi_tr": 0.0
        }

        self.set_max_time(max_time)
        self.set_target(task_target)
        self.set_trim_cond(trim_cond)

    def _calculate_reward(self):
        vel = np.sqrt((self.heli_dyn.state["uvw"] * self.heli_dyn.state["uvw"]).sum())
        vel_norm =  vel / self.normalizers["v"]
        veldot_norm = (self.heli_dyn.state["uvw"] * self.heli_dyn.state_dots["uvw"]).sum() / vel / self.normalizers["a"]
        dwn_norm = self.heli_dyn.state["xyz"][2] / self.normalizers["x"]
        dwndot_norm = self.heli_dyn.state_dots["xyz"][2] / self.normalizers["v"]
        pqr_norm = self.heli_dyn.state["pqr"] * self.normalizers["t"] 
        pqrdot_norm = self.heli_dyn.state_dots["pqr"] * self.normalizers["t"]**2

        vel_target_norm = np.array(self.task_target["vel"], dtype=np.float) / self.normalizers["v"]
        dwn_target_norm = np.array(-self.task_target["sea_alt"], dtype=np.float) / self.normalizers["x"]

        pqr_final_reward = - (pqr_norm * pqr_norm).sum()
        pqr_terminal_reward = - (np.sign(pqr_norm) * pqrdot_norm).sum()
        pqr_reward = max(
            pqr_final_reward,
            pqr_terminal_reward)

        vel_final_reward = - ((vel_norm - vel_target_norm) * (vel_norm - vel_target_norm)).sum()
        vel_terminal_reward = - (np.sign(vel_norm - vel_target_norm) * veldot_norm).sum()

        vel_reward = max(
            vel_final_reward,
            vel_terminal_reward)

        dwn_final_reward = - ((dwn_norm - dwn_target_norm) * (dwn_norm - dwn_target_norm)).sum()
        dwn_terminal_reward = - (np.sign(dwn_norm - dwn_target_norm) * dwndot_norm).sum()

        dwn_reward = max(
            dwn_final_reward,
            dwn_terminal_reward)


        reward = ( pqr_reward + vel_reward + dwn_reward ) / 3.0

        success_step = pqr_final_reward > -1.0 and vel_final_reward > -1.0 and dwn_final_reward > -1.0 

        return reward, success_step
