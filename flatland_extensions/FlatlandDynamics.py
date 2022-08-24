'''
The Flatland simulator does not have any train run dynamics implemented. Flatland Dynamics extends flatland to simulate
railway dynamics with a dynamics/physical layer. This code meant as a proof-of-concept to show how dynamics can be
integrated. The goal must be that all trains(agents) follow physical policy to accelerate. The acceleration must be
based on real train parameters such as train weight, train length and traction power. The breaking will be implemented
with a fixed physical delay. The physical delay will be a parameter which can be set per train (agent).

To simulate dynamics the simulator must be ensure that each train(agent) can break just before it will collide by any
others. This requires a new safety layer. Up to now flatland just ensures that there will be no more than maximum one
agent per cell. And a train (agent) can only be at one cell at once. This must be extended as well. Due of the
breaking distance which can vary from zero (train doesn’t move) up to many meters (train is moving). The breaking
distance strongly depends on the train speed. Thus, flatland has to ensure that one train (agent) can allocate many
cells and free them later. The allocation and freeze will no longer be restricted to one cell. Thus a cell can have
more than occupied or free. It can also be in state reserved. Reserved will mean that the train is not yet in the
cell by the cell is still occupied due of it’s breaking distance. The state reserved must not be implemented.
It can be implemented with occupied. But it’s important to understand that a train (agent) has more than two “cell
reservation state per cell”. Once a cell is locked (reserved, occupied) it will be for all other agents occupied
and they cannot enter.

Implementation idea: The simulator simulates with current agent position and direction the reservation point – this
is in flatland the current behavior. The agent can navigate as it will  with default flatland. Instead of the
real train run, we simulate virtual train run. Breaking distance and train (agent). But we must change the
occupied handling. A new feature is, that we must simulate the free point. The free point simulates the end of the
train. When the end of the train leaves a cell - it will be freezing the occupied cell. The dynamics will calculate
the speed for allocating (reservation point) – breaking distance and train run speed. Each cell needs the "distance"
or "length" information. There will be a minimal cell length, which depends on the simulation time step.
The reservation point cannot move faster than one cell per simulation step. If the reservation point cannot be
moved due of a locked (occupied) cell, the train has to break.
'''

# import all flatland dependance
from flatland.envs.rail_env import RailEnv


class FlatlandDynamics:
    def __init__(self, env: RailEnv):
        self.env = env
