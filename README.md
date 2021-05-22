# heli-gym 🚁
OpenAI GYM environment for 6-DOF Helicopter simulation.

This is GYM environment package for reinforcement learning for helicopter flight tasks using minimum complexity helicopter model.

-------------------------------------
## __Environment Details__

* Pure Gym environment
* Realistic Dynamic Model based on Minimum Complexity Helicopter Model ([Heffley and Mnich](https://robertheffley.com/docs/Sim_modeling/Heffley-Mnich--Minimum-Complexity%20Helicopter%20Simulation%20Math%20Model--NASA%20CR%20177476.pdf))
In addition, inflow dynamics are added and model is adjusted so that it covers multiple flight conditions. 
* Rendering is done by OpenGL.

![Caption](resources/ex.gif)

-------------------------------------
### __Action Space__
| Num | Act                | Unit  | Min   | Max    |
|-----|--------------------|-------|-------|--------|
| 0   | Collective         |       | -1    | 1      |
| 1   | Lon. Cyclic        |       | -1    | 1      |
| 2   | Lat. Cyclic        |       | -1    | 1      |
| 3   | Pedal              |       | -1    | 1      |

-------------------------------------
### __Observation Space__
| Num | Obs                | Unit  | Min   | Max    |
|-----|--------------------|-------|-------|--------|
| 0   | power              | hp    | 0     | ∞      |
| 1   | air speed          | ft/s  | 0     | ∞      |  
| 2   | angle of attack    | deg   | -180  | 180    |
| 3   | sideslip angle     | deg   | -180  | 180    |
| 4   | ground speed       | ft/s  | 0     | ∞      |  
| 5   | track angle        | deg   | -180  | 180    |
| 6   | climb rate         | ft/s  | -∞    | ∞      |  
| 7   | roll angle         | deg   | -180  | 180    |
| 8   | pitch angle        | deg   | -180  | 180    |
| 9   | yaw angle          | deg   | -180  | 180    |
| 10  | roll rate          | deg/s | -∞    | ∞      |  
| 11  | pitch rate         | deg/s | -∞    | ∞      |  
| 12  | yaw rate           | deg/s | -∞    | ∞      |  
| 13  | x acc (body)       | ft/s^2| -∞    | ∞      |  
| 14  | y acc (body)       | ft/s^2| -∞    | ∞      |  
| 15  | z acc (body)       | ft/s^2| -∞    | ∞      |  
| 16  | x loc (earth)      | ft    | -∞    | ∞      |  
| 17  | y loc (earth)      | ft    | -∞    | ∞      |  
| 18  | z loc (earth)      | ft    | -∞    | ∞      |

-------------------------------------
## __Tasks__
For now only one task is available. 

| Environment        | Details              |
|--------------------|----------------------|
| HeliHover-v0       | Hovering Task        |

-------------------------------------
## __Tested OS__
Environment tested on these OSs. If you have any problem, probably shared libraries for rendering 
make it, please look at [renderer page.](heligym/envs/renderer/README.md)

Environment should be run at least 100 FPS for sync with the dynamics of helicopter.
Receiving max FPS with NVIDIA 1070-TI with Intel i7-8700K given in the table. Limiting factor of FPS is CPU because of calculation of dynamics of helicopter in Python. If system cannot reach 100 FPS, please make sure that GPU driver be installed and make sure that CPU can handle with the Python.

| Tested OS      |  Max FPS |
|----------------|----------|
| Windows 10     |    350   |
| Ubuntu 18.04   |    470   |
| Ubuntu 16.04   |    500   |

For the user who has lower than the version of Ubuntu 16.04 (like Ubuntu 14.04 etc.) or other Linux distros, 
please re-compile dependent libraries. Re-compile instruction can be found at [renderer page.](heligym/envs/renderer/README.md)

-------------------------------------
## __Setup__
Run following command.
```bash
pip install -e .
```

-------------------------------------
## __Usage__
Create environment by either,
```python
from heligym import HeliHover
env = HeliHover()
```
or
```python
import gym
import heligym
env = gym.make("HeliHover-v0")
```

The rest is usual as of any GYM environment !
