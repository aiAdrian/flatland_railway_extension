# flatland_railway_extension
This repo extends [Flatland Railway Simulator](https://gitlab.aicrowd.com/flatland/flatland) with helpful features.  

## Extended RailEnv  
https://github.com/aiAdrian/flatland_railway_extension/blob/master/flatland_extensions/environment_extensions/XRailEnv.py

### Features
- Minimal train following time can be globally controlled 
- Multi-resource allocation can be implemented. This allows to implement mutual exclusive use of railroad switch cluster, connecting "edge cluster" and/or any other multi-ressource allocation problem, such as  ["Fahrstrassenausschluss"](https://de.wikipedia.org/wiki/Fahrstra%C3%9Fe), .. , ["Flankenschutz"](https://de.wikipedia.org/wiki/Fahrstra%C3%9Fe#Flankenschutz) 
 
- Dynamics - train power based acceleration (physics) and train specific "comfort" braking (with fixed negative acceleration).

### Features
Completely new experiments can be carried out using these extensions. The current version of Flatland allows to model  navigation (path finding) and limited to one resouce per time resource allocation problems. But in current version of Flatland exactly one resource can be assigned to an agent. This allows solving a simple resource allocation problem, which is certainly not far from the real railway specific problem domain. But no railway-technical important concepts such as "Flankenschutz" or "Durchrutschweg" can be modeled. With new introducted multiple resource allocation technics which allows to aligne more than one resource to an agent, these aspects can be modeled. Of course the modeling gets much more complex with respect of details/data, but the result is also much more realistic. Another important aspect in the augmentation of reality is that the agents behave dynamically in terms of movement. Agents have to  dynamically interact with each others. The braking distance and acceleration will be an important concept. To do this, the application can use the Flatland Dynamics extension. Of course, these can be combined with the multiple resource allocation. This finally leads to a very realistic modeling. 

The goal of this extension is that researchers can show where the limitaitons and streights of their methods lie. Some optimization methods will not be able to deal with the driving dynamics or only very limited, some algorithms are not suitable for dealing with multiple resource allocation problems and others are very slow to solve the problem. Others fites perfectly to one of the problem. It will be very exciting to see where the respective limits lie and whether someone find a real-time solver of any kind of those problems. 

## Information 
<p align="center" width="100%">
    <img width="33%" src="https://raw.githubusercontent.com/aiAdrian/flatland_railway_extension/master/images/flatland_scenario.png"> 
</p>

The initial implementation is authored by Adrian Egli's (aiAdrian) [neurips2020 flatland challenge solution (submission)](https://gitlab.aicrowd.com/adrian_egli/neurips2020-flatland-starter-kit)

- [RailroadSwitchAnalyser](https://github.com/aiAdrian/flatland_railway_extension/blob/master/flatland_extensions/RailroadSwitchAnalyser.py)
  ![RailroadSwitchAnalyser](https://raw.githubusercontent.com/aiAdrian/flatland_railway_extension/master/images/RailroadSwitchAnalyser.png "RailroadSwitchAnalyser")

- [RailroadSwitchCluster](https://github.com/aiAdrian/flatland_railway_extension/blob/master/flatland_extensions/RailroadSwitchCluster.py)
  ![RailroadSwitchCluster](https://raw.githubusercontent.com/aiAdrian/flatland_railway_extension/master/images/RailroadSwitchCluster.png "RailroadSwitchCluster")

- [FlatlandResourceAllocator](https://github.com/aiAdrian/flatland_railway_extension/blob/master/flatland_extensions/environment_extensions/FlatlandResourceAllocator.py)
   
- [FlatlandGraphBuilder](https://github.com/aiAdrian/flatland_railway_extension/blob/master/flatland_extensions/FlatlandGraphBuilder.py)
 

 
  
## Working code 
- [Google coLab notebook - Recife export](https://github.com/aiAdrian/flatland_railway_extension/blob/master/Flatland_recife.ipynb)
- [Google coLab notebook - Simulation with multi-resource reservation](https://github.com/aiAdrian/flatland_railway_extension/blob/master/Flatland_Resource_Allocation.ipynb)
- [Google coLab notebook - Flatland dynamics](https://github.com/aiAdrian/flatland_railway_extension/blob/master/Flatland_Dynamics.ipynb)




## Links 
[Flatland Challenge](https://www.aicrowd.com/search?utf8=%E2%9C%93&q=flatland)
[Flatland introduction](https://flatland.aicrowd.com/getting-started/env.html)


##### Permission to use  
If you use this or any idea out of this code for/in any academic publication - you must credit the authors. No commerical use allowed.
