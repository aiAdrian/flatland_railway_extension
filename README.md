# flatland_railway_extension
This repo extends [Flatland Railway Simulator](https://gitlab.aicrowd.com/flatland/flatland) with helpful features.  

## Extended RailEnv  
https://github.com/aiAdrian/flatland_railway_extension/blob/master/flatland_extensions/environment_extensions/XRailEnv.py

### Features
- Minimal train following time can be globally controlled 
- Multi-resource allocation can be implemented. This allows to implement mutual exclusive use of railroad switch cluster, connecting "edge cluster" and/or any other multi-ressource allocation problem, such as  ["Fahrstrassenausschluss"](https://de.wikipedia.org/wiki/Fahrstra%C3%9Fe), .. , ["Flankenschutz"](https://de.wikipedia.org/wiki/Fahrstra%C3%9Fe#Flankenschutz) 
 
- Dynamics - train power based acceleration (physics) and train specific "comfort" braking (with fixed negative acceleration).

## Information 
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
