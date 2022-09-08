# flatland_railway_extension
This repo extends [Flatland Railway Simulator](https://gitlab.aicrowd.com/flatland/flatland) with helpful features.  

## Extended RailEnv  
https://github.com/aiAdrian/flatland_railway_extension/blob/master/flatland_extensions/environment_extensions/XRailEnv.py

## Features
- Minimal train following time can be globally controlled 
- Multi-resource allocation can be implemented. This allows to implement mutual exclusive use of railroad switch cluster, connecting "edge cluster" and/or any other multi-ressource allocation problem - Railway-specific features can be implemented, e.g  ["Fahrstrassenausschluss"](https://de.wikipedia.org/wiki/Fahrstra%C3%9Fe), .. , ["Flankenschutz"](https://de.wikipedia.org/wiki/Fahrstra%C3%9Fe#Flankenschutz) 
 
- Dynamics - train power based acceleration (physics) and train specific "comfort" braking (with fixed negative acceleration).

### Details
Completely new experiments can be carried out using these extensions. In the current version of Flatland exactly one resource can be assigned to an agent at time. This allows solving a simplified resource allocation problem for railway operation - which is certainly not far away from problems in real railway world. But no important features such as "Flankenschutz" or "Durchrutschweg" can be modeled. With the [multiple resource allocation technics](https://github.com/aiAdrian/flatland_railway_extension/blob/master/flatland_extensions/environment_extensions/FlatlandResourceAllocator.py) - which allows to assigne more than one resource to an agent at time - this leads to a more realistic model and allows to model important aspects of railway safety. In consequence the modeling gets much more complex with respect of details/data requirement, but this leads to much more realistic simulation results. Another important aspect in the augmentation of reality is that the agents behave dynamically in terms of movement. Agents are able to dynamically interact with each others. The movement dynamics of each agent can be easy enabled by just using the [Flatland Dynamics](https://github.com/aiAdrian/flatland_railway_extension/blob/master/flatland_extensions/environment_extensions/FlatlandDynamics.py) extension. By combing all of those extensions the simulation will very realistic.

The goal of those extensions are that researchers can show where the limits and strengths of their proposed methods lie. Some optimization methods will not be able to deal with vehicle dynamics; some algorithms are not suitable for solving problems with multiple resource allocation and others are very slow to solve such complex dynamic problems. It will be very exciting to see where the effective limits lie and whether someone find a real-time solver of all kind of those problems.

### Examples
<p align="center" width="100%">
    <img width="33%" src="https://raw.githubusercontent.com/aiAdrian/flatland_railway_extension/master/images/flatland_scenario.png"> 
</p>


- [RailroadSwitchAnalyser](https://github.com/aiAdrian/flatland_railway_extension/blob/master/flatland_extensions/RailroadSwitchAnalyser.py)
  ![RailroadSwitchAnalyser](https://raw.githubusercontent.com/aiAdrian/flatland_railway_extension/master/images/RailroadSwitchAnalyser.png "RailroadSwitchAnalyser")

- [RailroadSwitchCluster](https://github.com/aiAdrian/flatland_railway_extension/blob/master/flatland_extensions/RailroadSwitchCluster.py)
  ![RailroadSwitchCluster](https://raw.githubusercontent.com/aiAdrian/flatland_railway_extension/master/images/RailroadSwitchCluster.png "RailroadSwitchCluster")

- [FlatlandDynamics](https://github.com/aiAdrian/flatland_railway_extension/blob/master/flatland_extensions/environment_extensions/FlatlandDynamics.py)
  ![FlatlandDynamics](https://github.com/aiAdrian/flatland_railway_extension/blob/master/images/FlatlandDynamics.png "FlatlandDynamics")

- [FlatlandResourceAllocator](https://github.com/aiAdrian/flatland_railway_extension/blob/master/flatland_extensions/environment_extensions/FlatlandResourceAllocator.py)
   
- [FlatlandGraphBuilder](https://github.com/aiAdrian/flatland_railway_extension/blob/master/flatland_extensions/FlatlandGraphBuilder.py)
 
  
## Working code 
- [Google coLab notebook - Recife export](https://github.com/aiAdrian/flatland_railway_extension/blob/master/Flatland_recife.ipynb)
- [Google coLab notebook - Simulation with multi-resource reservation](https://github.com/aiAdrian/flatland_railway_extension/blob/master/Flatland_Resource_Allocation.ipynb)
- [Google coLab notebook - Flatland dynamics](https://github.com/aiAdrian/flatland_railway_extension/blob/master/Flatland_Dynamics.ipynb)


## Links 
[Flatland Challenge](https://www.aicrowd.com/search?utf8=%E2%9C%93&q=flatland)
[Flatland introduction](https://flatland.aicrowd.com/getting-started/env.html)


##### Information  
The initial implementation is authored by Adrian Egli's (aiAdrian) [neurips2020 flatland challenge solution (submission)](https://gitlab.aicrowd.com/adrian_egli/neurips2020-flatland-starter-kit)

##### Permission to use  
If you use this or any idea out of this code for/in any academic publication - you must credit the authors. No commerical use allowed.
