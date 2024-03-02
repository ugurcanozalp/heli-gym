import numpy as np
from collections import OrderedDict
from copy import copy, deepcopy

class State(OrderedDict):
    def __init__(self, *args, **kwargs):
        super(State, self).__init__(*args, **kwargs)

    def __add__(self, rhs):
        selfcopy = deepcopy(self)
        if isinstance(rhs, State) and rhs.keys()==selfcopy.keys():
            for value, rhs_value in zip(selfcopy.values(), rhs.values()):
                value += rhs_value
            return selfcopy
        else:
            for value in selfcopy.values():
                value += rhs
            return selfcopy

    def __mul__(self, rhs):
        selfcopy = deepcopy(self)
        if isinstance(rhs, State) and rhs.keys()==selfcopy.keys():
            for value, rhs_value in zip(selfcopy.values(), rhs.values()):
                value *= rhs_value
            return selfcopy
        else:
            for value in selfcopy.values():
                value *= rhs
            return selfcopy

    def __sub__(self, rhs):
        selfcopy = deepcopy(self)
        if isinstance(rhs, State) and rhs.keys()==selfcopy.keys():
            for value, rhs_value in zip(selfcopy.values(), rhs.values()):
                value -= rhs_value
            return selfcopy
        else:
            for value in selfcopy.values():
                value -= rhs
            return selfcopy

    def __div__(self, rhs):
        selfcopy = deepcopy(self)
        if isinstance(rhs, State) and rhs.keys()==selfcopy.keys():
            for value, rhs_value in zip(selfcopy.values(), rhs.values()):
                value /= rhs_value
            return selfcopy
        else:
            for value in selfcopy.values():
                value /= rhs
            return selfcopy

    def __pow__(self, rhs):
        selfcopy = deepcopy(self)
        if isinstance(rhs, State) and rhs.keys()==selfcopy.keys():
            for value, rhs_value in zip(selfcopy.values(), rhs.values()):
                value = value ** rhs_value
            return selfcopy
        else:
            for value in selfcopy.values():
                value = value ** rhs
            return selfcopy

    def __mod__(self, rhs):
        selfcopy = deepcopy(self)
        if isinstance(rhs, State) and rhs.keys()==selfcopy.keys():
            for value, rhs_value in zip(selfcopy.values(), rhs.values()):
                value = value % rhs_value
            return selfcopy
        else:
            for value in selfcopy.values():
                value = value % rhs
            return selfcopy

class StateNumpy():
    def __init__(self):
        self.info = {}
        self.size = 0
        self.val = np.array([], dtype=np.float32)

    def __setitem__(self, name: str, value: np.ndarray):
        if name in self.info.keys():
            start, end = self.info[name][1]
            self.val[start:end] = value
        else:
            self.info[name] = (value, (self.size, self.size + len(value)))
            self.size += len(value)
            self.val = np.append(self.val, value)

    def __getitem__(self, name: str):
        start, end = self.info[name][1]
        return self.val[start:end]

    def __add__(self, rhs):
        arg = rhs.val if isinstance(rhs, StateNumpy) else rhs
        val = self.val + arg
        output = StateNumpy()
        output.info = self.info
        output.size = self.size
        output.val = val
        return output 

    def __mul__(self, rhs):
        arg = rhs.val if isinstance(rhs, StateNumpy) else rhs
        val = self.val * arg
        output = StateNumpy()
        output.info = self.info
        output.size = self.size
        output.val = val
        return output 

    def __sub__(self, rhs):
        arg = rhs.val if isinstance(rhs, StateNumpy) else rhs
        val = self.val - arg
        output = StateNumpy()
        output.info = self.info
        output.size = self.size
        output.val = val
        return output 

    def __div__(self, rhs):
        arg = rhs.val if isinstance(rhs, StateNumpy) else rhs
        val = self.val / arg
        output = StateNumpy()
        output.info = self.info
        output.size = self.size
        output.val = val
        return output 

class DynamicSystem(object):
    def __init__(self, dt: float = 0.01):
        self.state = StateNumpy()
        self.state_dots = StateNumpy()
        self.dt = dt
        self.action = None # should be filled after a step call
        self.observation = None # should be filled after a step call, or trim
        self.previous_observation = None # should be filled after a step call

    def dynamics(self, state, set_observations=False):
        raise NotImplementedError

    def _register_state(self, name:str, value: np.ndarray):
        self.state[name] = value
        self.state_dots[name] = np.zeros(value.shape)

    def __getitem__(self, name):
        if name in self.state.keys():
            return self.state[name] 
        else:
            assert False, "given state name not found!"

    def __setitem__(self, name, value):
        if name in self.state.keys():
            self.state[name] = np.array(value)
        else:
            assert False, "given state name not found!"
        
    def step(self, action):
        """This function lets the system to go one time step ahead using RK4.
        """
        self.action = action
        self.step_before()
        self.previous_observation = self.observation
        k1 = self.dynamics(self.state)
        k2 = self.dynamics(self.state + k1 * (0.5*self.dt))
        k3 = self.dynamics(self.state + k2 * (0.5*self.dt))
        k4 = self.dynamics(self.state + k3 * self.dt, set_observation=True)
        self.state += (k1 + k2*2 + k3*2 + k4)*(0.16666666666666666 * self.dt)
        self.state_dots = k4
        self.step_after()
        return self.observation

    def step_before(self):
        """This method can be overwritten by inherited classes.
        """
        pass

    def step_after(self):
        """This method can be overwritten by inherited classes.
        """
        pass

if __name__=='__main__':
    mystate = State([("uvw", np.zeros(3)), ("pqr", np.zeros(3)), ("xyz", np.zeros(3))])
    mydict = OrderedDict([("uvw", np.zeros(3)), ("pqr", np.zeros(3)), ("xyz", np.zeros(3))])
    print(mystate*0.01+1.3)