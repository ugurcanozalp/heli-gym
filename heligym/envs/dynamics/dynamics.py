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
                value -= rhs_value
            return selfcopy
        else:
            for value in selfcopy.values():
                value -= rhs
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

class DynamicSystem(object):
    def __init__(self, dt: float = 0.01):
        self.state = State()
        self.state_dots = State()
        self.dt = dt
        self.last_action = None # should be filled after a step call.
        self.observation = None # should be filled after a step call

    def dynamics(self, state, action, set_observations=False):
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
        # Solve system by RK4 a single step.
        k1 = self.dynamics(self.state, action)
        k2 = self.dynamics(self.state + k1 * (0.5*self.dt), action)
        k3 = self.dynamics(self.state + k2 * (0.5*self.dt), action)
        k4 = self.dynamics(self.state + k3 * self.dt, action, set_observation=True)
        self.state += (k1 + k2*2 + k3*2 + k4)*(1/6 * self.dt)
        self.state_dots = k4
        self.last_action = action
        # Solve system by Implicit Trapezoidal Rule a singel step.
        #a = self.dynamics(self.state, action)
        #next_state = a*self.dt
        #for i in range(20):
        #    b = self.dynamics(next_state, action)
        #    next_state = self.state + (a + b)*(0.5*self.dt)
        #
        #self.state = next_state
        #self.state_dots = b
        #self.last_action = action

    def _get_observation(self):
        return self.observation

    def trim(self):
        raise NotImplementedError

if __name__=='__main__':
    mystate = State([("uvw", np.zeros(3)), ("pqr", np.zeros(3)), ("xyz", np.zeros(3))])
    mydict = OrderedDict([("uvw", np.zeros(3)), ("pqr", np.zeros(3)), ("xyz", np.zeros(3))])
    print(mystate*0.01+1.3)