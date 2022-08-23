'''
The Flatland currently has no trainrun dynamics implemented. Flatland Dynamics extends flatland to simulate railway
dynamics with a phyical component. This code is a proof-of-concept to simulate all agents acceleration based on train
parameters such as train weight,  traction power limitation. The breaking will be implemented with a fixed physical
delay.

To simulate dynamics the simulator must be ensure that each agents can break just before it will collide. This
requires a new safty layer. Up to now flatland just ensures that there will be no more than maximum one agent per cell.
And a agent can only be at one cell at once. This has to be extended as well. Due of the breaking distance which can
vary from zero upto many meters. It depends on the current train speed. Thus flatland has to ensure that one agent can
allocate a cell and free it when the agent leaves the a cell. Thus a cell can have more than occupied or free.
It can also be in state reserved. Once a cell is resevered it will be for all other agents occupied.

Thus we will simulate with current agent position and direction the reservation point. The agent can navigate as it will
 with default flatland. But we have to change the occupied handling. To do that we have to simulate the free point.
 The free point simulates the end of the train. When the end of the train leaves a cell - it will be tag as free.
 We have to dynamic calucate the speed for allocating (reservation point). This can be done by calculating the breaking
distance. Which strongly correlates to current train speed.
Each cell needs the "distance" or "lenght" information. There will be a minimal length, which depends on the
simulation time step. The reservation point can not move faster than one cell per simulaton step. If the reservation
point can not be moved due of a locked (occupied) cell, the train has to break.
'''