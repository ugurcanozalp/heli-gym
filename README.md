# heli-gym 🚁
OpenAI GYM environment for 6-DOF Helicopter simulation.

This is GYM environment package for reinforcement learning for helicopter flight tasks using minimum complexity helicopter model.

## Environment Details

* Pure Gym environment
* Realistic Dynamic Model using Minimum Complexity Helicopter Model ([Heffley and Mnich](https://robertheffley.com/docs/Sim_modeling/Heffley-Mnich--Minimum-Complexity%20Helicopter%20Simulation%20Math%20Model--NASA%20CR%20177476.pdf))
* Rendering with OpenGL will be added soon.

### Action Space
| Num | Act                | Unit  | Min   | Max    |
|-----|--------------------|-------|-------|--------|
| 0   | Collective         | %     | 0.0   | 1.0    |
| 1   | Lon. Cyclic        | %     | 0.0   | 1.0    |
| 2   | Lat. Cyclic        | %     | 0.0   | 1.0    |
| 3   | Pedal              | %     | 0.0   | 1.0    |

### Observation Space
| Num | Obs                | Unit  | Min   | Max    |
|-----|--------------------|-------|-------|--------|
| 0   | power              | hp    | -∞    | ∞      |
| 1   | air speed          | ft/s  | 0.0   | ∞      |  
| 2   | angle of attack    | deg   | -π    | π      |
| 3   | sideslip angle     | deg   | -π    | π      |
| 4   | ground speed       | ft/s  | 0.0   | ∞      |  
| 5   | track angle        | deg   | -π    | π      |
| 6   | climb rate         | ft/s  | -∞    | ∞      |  
| 7   | roll angle         | deg   | -π    | π      |
| 8   | pitch angle        | deg   | -π    | π      |
| 9   | yaw angle          | deg   | -π    | π      |
| 10  | roll rate          | deg/s | -∞    | ∞      |  
| 11  | pitch rate         | deg/s | -∞    | ∞      |  
| 12  | yaw rate           | deg/s | -∞    | ∞      |  
| 13  | x acceleration     | ft/s^2| -∞    | ∞      |  
| 14  | y acceleration     | ft/s^2| -∞    | ∞      |  
| 15  | z acceleration     | ft/s^2| -∞    | ∞      |  
| 16  | x location(earth)  | ft    | -∞    | ∞      |  
| 17  | y location(earth)  | ft    | -∞    | ∞      |  
| 18  | z location(earth)  | ft    | -∞    | ∞      |

## Setup
Run following command.
```bash
pip install -e .
```

## Usage
Create environment by either,
```python
from heligym import Helicopter
env = Helicopter()
```
or
```python
import gym
import heligym
env = gym.make("Heli-v0")
```

