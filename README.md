# NICOL-Coppeliasim

Bindings for the CoppeliaSim robot simulator similar to the ROS NIAS-API.
The api uses the [ZeroMQ interface](https://github.com/CoppeliaRobotics/zmqRemoteApi/tree/master/clients/python) of Coppeliasim in the back to map the NIAS-API calls to CoppeliaSim.

# Publications

[NICOL: A Neuro-inspired Collaborative
Semi-humanoid Robot that Bridges
Social Interaction and Reliable
Manipulation](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=10304130) (Matthias Kerzel, Philipp Allgeuer, Erik Strahl, Nicolas Frick,
Jan-Gerrit Habekost, Manfred Eppe, and Stefan Wermter)

[CycleIK: Neuro-inspired Inverse Kinematics](https://link.springer.com/chapter/10.1007/978-3-031-44207-0_38) (Jan-Gerrit Habekost, Erik Strahl, Philipp Allgeuer, Matthias Kerzel, Stefan Wermter)

# Requirements
`GPU:` >= GTX 1050 TI

`Python:` 3.8, 3.10

`Ubuntu:` 20.04, 22.04  

# Installation
Download version 4.5.1 of CoppeliaSim here: https://www.coppeliarobotics.com/previousVersions

Current version: https://www.coppeliarobotics.com/downloads (will probably crash with a different version)

Add the following to your ~/.bashrc file: (NOTE: the 'EDIT ME' in the first line).

```bash
export COPPELIASIM_ROOT=EDIT/ME/PATH/TO/COPPELIASIM/INSTALL/DIR
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$COPPELIASIM_ROOT
export QT_QPA_PLATFORM_PLUGIN_PATH=$COPPELIASIM_ROOT
```

## Via script
The easiest way to install is via the setup.sh script.

```bash
git clone https://git.informatik.uni-hamburg.de/wtm-robots-and-equipment/nicol-coppeliasim.git
cd nicol_coppeliasim
source setup.sh
```

# Run the demo
You should be good to go now. A good first start might be to check out the examples in the project.
The demo.py file demonstrates the use of all functions that are implemented for the simulator.

```bash
python3 hri_demo.py
```

# Documentation:
The implementation uses pydoc for documentation. You can build the documentation e.g. as html with:

``` bash
pydoc -w nicol_api
```

Afterwards you can view the nicol_api.html

# Physics
We use the mujoco physics engine for this project, since it is very fast and reasonable accurate.
Getting a stable physics simulation right can be tricky anyhow.
It helps a lot to increase the number of simulation steps to make the simulation more stable. In the example scene we use 2 steps per frame (see simulation settings). In addition, we switched to elliptic collision calculations since this seems to help as well.
We are able to lift a cuboid of mass 0.001 with the default physics parameters like this.
If you have trouble with objects flying away I suggest you reduce the first two parameters of the solimp variable in the engine properties of the object. If you have problems with objects slipping out of the hand you probably have to increase this attribute, but in that case you probably have to increase the simulation steps as well to keep the simulation stable.
https://mujoco.readthedocs.io/en/stable/overview.html#softness-and-slip

# Zero Mq Api
If you need more functionallyties from the simulator you can find the documentation of the API here:
https://www.coppeliarobotics.com/helpFiles/en/apiFunctions.htm#objectParameters

# Contact
    - Lennart Clasmeier (lennart.clasmeier@uni-hamburg.de) 
    - Jan-Gerrit Habekost (jan-gerrit.habekost@uni-hamburg.de) 


<div align="center">
![Knowledge Technology Logo](https://www.inf.uni-hamburg.de/26013038/wtm-logo-white-b5929635e66483567df041e578f21cf47d574c35.jpg){width=15%}

Knowledge Technology, MIN-Faculty, University of Hamburg 

https://www.inf.uni-hamburg.de/en/inst/ab/wtm.html
</div>