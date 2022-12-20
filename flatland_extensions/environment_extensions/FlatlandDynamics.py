# Idea: Adrian Egli / Erik Nygren
#
# Permission to use:
# ----------------------------------------------------------------------------------------------------------
# If you use this or any idea out of this code for/in any academic publication - you must credit the authors.
# No commercial use allowed - neither the underlaying idea nor the implementation.
# ----------------------------------------------------------------------------------------------------------

'''
The flatland simulator has no agent movement dynamics implemented. Flatland Dynamics expands Flatland with
agent movement dynamics (physics). This python code is intended as a proof of concept to show how dynamics can be
integrated into Flatland. The goal of this expansion is to show how all agents perform realistic
under acceleration/braking movements and interact with each other. The acceleration is based on realworld parameters
such as train weight, train length and traction power (rolling stock). Braking is implemented with fixed physical
delay. The physical delay is a parameter that can be set for each agent separately. The reason behind this fixed delay
(negative acceleration) is that this can be quite easy be implemented in a fixed time-step simulation where the next
state only depends on the current state.

In order to simulate the dynamics, the simulator must ensure that each agent can brake before colliding with
others. The braking distance can vary from zero (train is not moving) to many meters (train is moving). The braking
distance is highly dependent on train speed. The braking distance plus the train length can be longer then one cell's
length. Therefore the agent must be capable to reserve one or more resources (cell). Reserved means that the
train has not yet arrived at the cell but the cell is still occupied. The reserved state must not be explicitly
implemented. Once a cell is locked (reserved, occupied), it is occupied for all other agents and it can't be
allocated by other agents. Therefore Flatland must be able to lock more then one cell per agent. This requires a
multiple resource allocation function which is implemented as a further extension in the Python class:
FlatlandResourceAllocator.

Implementation idea: The Flatland Dynamics simulates a reservation point for each agent with the usual agent
position. The reservation point behaves as the agent's position does in Flatland. The agent can navigate freely
with the standard Flatland action calls. To simulate the agent with dynamic movement a second point will be used. The
second point of interest is the end-of-agent (train). When the end-of-agent leaves a cell, it is no longer occupied.
The underlaying cell will be deallocated. End-of-agent position plus the train length plus the braking distance in
the direction of travel is equal to agent's position. The entire space in between agent's position and end-of-agent is
occupied for mutual exclusive use for the agent. Consequently, no other train can be between the braking distance and
the train. If the agent moves forward, he must change position. A position change requires the allocation of a new
resource. If the resource allocation fails, the agent can not move. Due of the resource allocation conflict,
the agent has to brake instantly. Only if the agent brakes immediately is it guaranteed that the agent will
stop before the colliding. If the agent can move, it can accelerate, keep the current speed constant or brake. However,
the agent is forced to comply with the underlying speed restrictions. If the maximum allowed speed is less than the
current speed, the agent accelerates. If the maximum speed is equal to the current speed, the agent keeps the current
speed constant. And finally, if the current speed is greater than the maximum allowed speed, the agent brakes.
The maximum allowed speed is the minimum of all local maximum allowed velocity of all occupied cells between the agent's
end and the reservation point (agent's position) and the maximum allowed speed of the agent.

To ensure this works for a discrete world, is the minimal cell length limited to the maximum allowed reservation
point forward step. Thus the cell length can not be smaller than the reservation point can move on at one simulation
time step. The reason behind this limitation is that Flatland can only change one cell at one time step. To control
this limitation the simulation time step can be adjusted or the cell length.



Agent occupation space view
___ ___ ___ __E LLL LL_ ___ ___ ___ ___ _P_ ___ ___ ___ ___ ___ ___ ___ ___ ___
              |______|::::::::::::::::::::
                train   braking distance

 P  : Agent position (Flatland)
 E  : End-of-agent (Train end)
 L  : Agent length (Train length)



Resource occupation view
___ ___ ___ _X_ _X_ _X_ _X_ _X_ _X_ _X_ _X_ ___ ___ ___ ___ ___ ___ ___ ___ ___

_X_ : Locked ( allocated cell/resource )
___ : free cell / resource



See also
FlatlandDynamics.py
DynamicAgent.py
RollingStock.py
InfrastructureData.py
'''

# import all flatland dependance

from typing import Dict

from flatland.core.env_observation_builder import ObservationBuilder
from flatland.envs.observations import GlobalObsForRailEnv
from flatland.envs.rail_env_action import RailEnvActions

from flatland_extensions.environment_extensions.DynamicAgent import DynamicAgent
from flatland_extensions.environment_extensions.FlatlandResourceAllocator import FlatlandResourceAllocator
from flatland_extensions.environment_extensions.XRailEnv import XRailEnv


class FlatlandDynamics(XRailEnv):
    def __init__(self,
                 width,
                 height,
                 rail_generator=None,
                 line_generator=None,  # : line_gen.LineGenerator = line_gen.random_line_generator(),
                 number_of_agents=2,
                 obs_builder_object: ObservationBuilder = GlobalObsForRailEnv(),
                 malfunction_generator_and_process_data=None,  # mal_gen.no_malfunction_generator(),
                 malfunction_generator=None,
                 remove_agents_at_target=True,
                 random_seed=None,
                 record_steps=False,
                 ):
        super(FlatlandDynamics, self).__init__(width=width,
                                               height=height,
                                               rail_generator=rail_generator,
                                               line_generator=line_generator,
                                               number_of_agents=number_of_agents,
                                               obs_builder_object=obs_builder_object,
                                               malfunction_generator_and_process_data=malfunction_generator_and_process_data,
                                               malfunction_generator=malfunction_generator,
                                               remove_agents_at_target=remove_agents_at_target,
                                               random_seed=random_seed,
                                               record_steps=record_steps)

    def reset_agents(self):
        super(XRailEnv, self).reset_agents()
        x_dynamic_agents = []
        for agent in self.agents:
            x_dynamic_agents.append(DynamicAgent(agent))
        self.agents = x_dynamic_agents
        self._enforce_using_flatland_resource_allocator()
        self.set_max_episode_steps(50000)

    def _enforce_using_flatland_resource_allocator(self):
        if not self.is_flatland_resource_allocator_activated():
            self.activate_flatland_resource_allocator(FlatlandResourceAllocator(env=self))

    def step(self, action_dict_: Dict[int, RailEnvActions]):
        observations, all_rewards, done, info = super(FlatlandDynamics, self).step(action_dict_=action_dict_)
        return observations, all_rewards, done, info

    def _handle_end_reward(self, agent):
        return 0

    def post_preprocess_action(self, action, agent):
        preprocessed_action = super(XRailEnv, self).preprocess_action(action, agent)

        if preprocessed_action == RailEnvActions.STOP_MOVING:
            agent.set_hard_brake(True)

        if not agent.update_movement_dynamics():
            self.motionCheck.addAgent(agent.handle, agent.position, agent.position)
            preprocessed_action = RailEnvActions.STOP_MOVING

        if preprocessed_action == RailEnvActions.DO_NOTHING:
            preprocessed_action = RailEnvActions.MOVE_FORWARD

        return preprocessed_action
