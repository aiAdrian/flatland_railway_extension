# flatland_railway_extension
This repo extends [Flatland Railway Simulator](https://gitlab.aicrowd.com/flatland/flatland) with helpful features.  

## Extended RailEnv  
- class [XRailEnv](https://github.com/aiAdrian/flatland_railway_extension/blob/master/flatland_extensions/environment_extensions/XRailEnv.py) extends [RailEnv](https://gitlab.aicrowd.com/flatland/flatland/-/blob/master/flatland/envs/rail_env.py#L36)
  
  XRailEnv is an extended version of the Flatland environment and supports multiple resources allocation to an agent at time.

- class [FlatlandDynamics](https://github.com/aiAdrian/flatland_railway_extension/blob/master/flatland_extensions/environment_extensions/FlatlandDynamics.py) extends [XRailEnv](https://github.com/aiAdrian/flatland_railway_extension/blob/master/flatland_extensions/environment_extensions/XRailEnv.py)

  FlatlandDynamics extends XRailEnv and implements vehicle movement dynamics. The simulation takes into account rolling stock properties such as maximal allowed velocity and traction power, topology (gradient) and physical train length.

## Features
- Minimal train following time can be globally controlled 
- Multi-resource allocation can be implemented. This allows to implement mutual exclusive use of railroad switch cluster, connecting "edge cluster" and/or any other multi-ressource allocation problems - Railway-specific features can be implemented, e.g  ["Fahrstrassenausschluss"](https://de.wikipedia.org/wiki/Fahrstra%C3%9Fe), .. , ["Flankenschutz"](https://de.wikipedia.org/wiki/Fahrstra%C3%9Fe#Flankenschutz) 
 
- Dynamics - train power based acceleration (physics) and train specific "comfort" braking (with fixed negative acceleration).

### Details
Completely new experiments can be carried out using these extensions. In the current version of Flatland exactly one resource can be assigned to an agent at time. This allows solving a simplified resource allocation problem for railway operation - which is certainly not far away from problems in real railway world. But no important features such as "Flankenschutz" or "Durchrutschweg" can be modeled. With the [multiple resource allocation technics](https://github.com/aiAdrian/flatland_railway_extension/blob/master/flatland_extensions/environment_extensions/FlatlandResourceAllocator.py) - which allows to assigne more than one resource to an agent at time - this leads to a more realistic model and allows to model important aspects of railway safety. In consequence the modeling gets much more complex with respect of details/data requirement, but this leads to much more realistic simulation results. Another important aspect in the augmentation of reality is that the agents behave dynamically in terms of movement. Agents are able to dynamically interact with each others. The vehicle movement dynamics of each agent can be enabled by using the [FlatlandDynamics](https://github.com/aiAdrian/flatland_railway_extension/blob/master/flatland_extensions/environment_extensions/FlatlandDynamics.py) environment. By combing all of those extensions the simulation will become very realistic.

The goal of this extension package is that researchers can show where the limits and strengths of their proposed methods lie. Some optimization methods will not be able to deal with vehicle dynamics; some algorithms are not suitable for solving problems with multiple resource allocation at time. Others are might very slow to solve such complex dynamic problems and there is might a solution which can solve it in raisonable time, but the solution quality must be further improved. It's very exciting to see where the effective limits of each method lie and whether someone find a real-time solver of all kind of those problems.

### Examples
<p align="center" width="100%">
    <img width="33%" src="https://raw.githubusercontent.com/aiAdrian/flatland_railway_extension/master/images/flatland_scenario.png"> 
</p>


- [RailroadSwitchAnalyser](https://github.com/aiAdrian/flatland_railway_extension/blob/master/flatland_extensions/RailroadSwitchAnalyser.py)
  
  The illustration on the left shows all crossings (switches) and the illustration on the right shows all switch neighbours. A switch neighbours is an infrastructure element that has an intersection (crossing) as its neighbour.
  
  ![RailroadSwitchAnalyser](https://raw.githubusercontent.com/aiAdrian/flatland_railway_extension/master/images/RailroadSwitchAnalyser.png "RailroadSwitchAnalyser")


- [RailroadSwitchCluster](https://github.com/aiAdrian/flatland_railway_extension/blob/master/flatland_extensions/utils/FlatlandDynamicsRenderer.py)
  
  The left illustration shows all connecting edges (cluster). The number shows the cluster id. Cells with the same cluster id belong to the same cluster. The right illustration shows all switch clusters. A switch cluster contains one or more crossing (switch) cells where each switch within the cluster are all neighbors. The switches in the same cluster have the same switch cluster id. 
  
  ![RailroadSwitchCluster](https://raw.githubusercontent.com/aiAdrian/flatland_railway_extension/master/images/RailroadSwitchCluster.png "RailroadSwitchCluster")

- [FlatlandResourceAllocator](https://github.com/aiAdrian/flatland_railway_extension/blob/master/flatland_extensions/environment_extensions/FlatlandResourceAllocator.py)

   The Flatland Resource Allocator extension allows the implementation of a simultaneous allocation of multiple resources to an agent, and also allows the concept of minimal headway (train-following), which roughly simulates the real infrastructure behavior. A two-minute train sequence (n-flatland time steps) is often used in many real railway systems - the entire system is therefore designed for a minimum headway of n seconds. With the help of the multi-resource allocator it is possible to implement "flank protection", ... and dynamic movement.
   
   
- [FlatlandGraphBuilder](https://github.com/aiAdrian/flatland_railway_extension/blob/master/flatland_extensions/FlatlandGraphBuilder.py)
 

- [FlatlandDynamics](https://github.com/aiAdrian/flatland_railway_extension/blob/master/flatland_extensions/environment_extensions/FlatlandDynamics.py)
  
  The diagram on the left shows the speed diagram for each train. The traveled distance [m] is plotted on the x-axis. The speed in kilometers per hour [km/h] is shown on the y-axis. The orange curve shows the maximum allowed speed. The blue curve is the simulated speed. The length of the train is easy to see because the last axle of the train must have allowed a higher speed, otherwise the train will not accelerate. The diagram on the right shows the acculeration [m/s] on the y-axis.  On the x-axis traveled distance [m] is plotted again.

 ![FlatlandDynamics](https://github.com/aiAdrian/flatland_railway_extension/blob/master/images/FlatlandDynamics.png "FlatlandDynamics")


- [Rolling Stock](https://github.com/aiAdrian/flatland_railway_extension/blob/master/flatland_extensions/environment_extensions/RollingStock.py)
  
  The rolling stock data stores the technical characteristics of each locomotive, including tractive effort and speed limits. The [DynamicAgent](https://github.com/aiAdrian/flatland_railway_extension/blob/master/flatland_extensions/environment_extensions/DynamicAgent.py) stores the physical properties such as mass and length and requires the rolling stock data for the traction.
  
  The diagram shows the traction characteristics. The speed is plotted on the x-axis. The maximum tractive effort is plotted on the y-axis. Traction power is limited by the maximum force that traction can exert on the wheel and is further limited by the maximum power of the motor.
  
<p align="center" width="100%">
    <img width="20%" src="https://github.com/aiAdrian/flatland_railway_extension/blob/master/images/FlatlandDynamics_RollingStock_tractive_effort.png "> 
</p>
  
    P = F * v = m * a * v

        F : Force [N]
        P : Power [W]
        v : Velocity [m/s]
        m : Mass [kg] 
        a : Acceleration [m/s2]
 

  
    Pmax = v_max_traction * max_traction
    
        Pmax : Maximum power of the traction 
        v_max_traction : Is the speed up to which the vehicle can deliver the maximum force.
        max_traction : Is the overall maximum force the traction can deliver.
  
 



- [FlatlandDynamicsRendering](https://github.com/aiAdrian/flatland_railway_extension/blob/master/flatland_extensions/utils/FlatlandDynamicsRenderer.py)
  
  Resources colored orange, red or black are occupied by the train. Orange indicates a resource reserved for the train but not required by either the braking distance or the physical train. Red or black resources are security related. Black is the physical train and red resources are needed for braking. The physical train can occupy more than one cell since the train length can be greater than the length of the underlaying cell. In the visualization, however, a train that would fit into one cell can also take up more than one cell if part of the train is in the next cell and part is still in the current one. Green resources are still occupied. They are intended to approximately simulate the time required to handle all security elements - they represents the minimal train following time. 
  
  ![FlatlandDynamicsRendering](https://github.com/aiAdrian/flatland_railway_extension/blob/master/images/FlatlandDynamicsRendering.png "FlatlandDynamicsRendering")
The Example is showing a moving block based simulation. The rendering is done with FlatlandDynamicsRenderer.


  
## Working code 
- [Google coLab notebook - Recife export](https://github.com/aiAdrian/flatland_railway_extension/blob/master/Flatland_recife.ipynb)
- [Google coLab notebook - Simulation with multi-resource reservation](https://github.com/aiAdrian/flatland_railway_extension/blob/master/Flatland_Resource_Allocation.ipynb)
- [Google coLab notebook - Flatland dynamics](https://github.com/aiAdrian/flatland_railway_extension/blob/master/Flatland_Dynamics.ipynb)


## Links 
[Flatland Challenge](https://www.aicrowd.com/search?utf8=%E2%9C%93&q=flatland)

[Flatland introduction](https://flatland.aicrowd.com/getting-started/env.html)

[Rolling stock](https://en.wikipedia.org/wiki/Rolling_stock) 

[Nagel-Schrekenberg-Model](https://en.wikipedia.org/wiki/Nagel%E2%80%93Schreckenberg_model)


##### Information  
The initial implementation is authored by Adrian Egli's (aiAdrian) [neurips2020 flatland challenge solution (submission)](https://gitlab.aicrowd.com/adrian_egli/neurips2020-flatland-starter-kit)

##### Permission to use  
If you use this or any idea out of this code for/in any academic publication - you must credit the authors. No commerical use allowed.
