import numpy as np
from collections import OrderedDict
from typing import Any

class State(OrderedDict):
    def __init__(self, *args, **kwargs):
        super(State, self).__init__(*args, **kwargs)

    def __add__(self, rhs):
        if isinstance(rhs, State) and rhs.keys()==self.keys():
            for value, rhs_value in zip(self.values(), rhs.values()):
                value += rhs_value
            return self
        else:
            for value in self.values():
                value += rhs
            return self

    def __mul__(self, rhs):
        if isinstance(rhs, State) and rhs.keys()==self.keys():
            for value, rhs_value in zip(self.values(), rhs.values()):
                value *= rhs_value
            return self
        else:
            for value in self.values():
                value *= rhs
            return self

    def __sub__(self, rhs):
        if isinstance(rhs, State) and rhs.keys()==self.keys():
            for value, rhs_value in zip(self.values(), rhs.values()):
                value -= rhs_value
            return self
        else:
            for value in self.values():
                value -= rhs
            return self

    def __div__(self, rhs):
        if isinstance(rhs, State) and rhs.keys()==self.keys():
            for value, rhs_value in zip(self.values(), rhs.values()):
                value -= rhs_value
            return self
        else:
            for value in self.values():
                value -= rhs
            return self

    def __pow__(self, rhs):
        if isinstance(rhs, State) and rhs.keys()==self.keys():
            for value, rhs_value in zip(self.values(), rhs.values()):
                value = value ** rhs_value
            return self
        else:
            for value in self.values():
                value = value ** rhs
            return self

    def __mod__(self, rhs):
        if isinstance(rhs, State) and rhs.keys()==self.keys():
            for value, rhs_value in zip(self.values(), rhs.values()):
                value = value % rhs_value
            return self
        else:
            for value in self.values():
                value = value % rhs
            return self


    def __radd__(self, lhs):
        for value in self.values():
            value = lhs + value
        return self

    def __rmul__(self, lhs):
        for value in self.values():
            value = lhs * value
        return self

    def __rsub__(self, lhs):
        for value in self.values():
            value = lhs - value
        return self

    def __rdiv__(self, lhs):
        for value in self.values():
            value = lhs / value
        return self

class DynamicSystem(object):
    def __init__(self, dt: float = 0.01):
        self.state = State()
        self.state_dots = State()
        self.dt = dt
        self.last_action = None

    def dynamics(self, state, action):
        raise NotImplementedError

    def register_state(self, name:str, value: np.ndarray):
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
        k1, _ = self.dynamics(self.state, action)
        k2, _ = self.dynamics(self.state + 1/2 * k1 * self.dt, action)
        k3, _ = self.dynamics(self.state + 1/2 * k2 * self.dt, action)
        k4, _ = self.dynamics(self.state + k3 * self.dt, action)
        self.state = self.state + 1/6 * (k1 + 2*k2 + 2*k3 + k4)*self.dt
        self.state_dots, _ = self.dynamics(self.state, action)
        self.last_action = action

    def get_observation(self):
        state_dots, observartion = self.dynamics(self.state, self.last_action)
        return observartion

    def trim(self):
        raise NotImplementedError

if __name__=='__main__':
    mystate = State([("uvw", np.zeros(3)), ("pqr", np.zeros(3)), ("xyz", np.zeros(3))])
    mydict = OrderedDict([("uvw", np.zeros(3)), ("pqr", np.zeros(3)), ("xyz", np.zeros(3))])
    print(mystates*0.01+1.3)