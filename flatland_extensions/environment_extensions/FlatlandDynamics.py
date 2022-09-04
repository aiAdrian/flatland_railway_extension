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
length. Therefore the agent must be capable to reserve resources (cell). Reserved means that the train has not yet
arrived at the cell but the cell is still occupied. The reserved state must not be explicitly implemented.
Once a cell is locked (reserved, occupied), it is occupied for all other agents and it can't be allocated by other
agents. Flatland must be able to lock more then one cell per agent. This requires an other extension and it's
implemented in python class: FlatlandResourceAllocator.

Implementation idea: The simulator simulates the reservation point with the common Flatland's agent position and
direction - the reseravtion point (agent) behaves as any agent does in Flatland. The agent can navigate
freely with the standard Flatland actions. Instead of real train we simulate virtually the train at end of
reservation area - braking distance plus train lenght are locked for all other agents. But we have to change
that manned handling. What is new is that we have to simulate the free point. The free point simulates the end of the
train. If the end of the turn leaves a cell, the occupied cell is frozen. The dynamic is calculated the speed for
the assignment (reservation point) – braking distance and train speed. Every cell needs the "distance" or "length"
information. There is a minimum cell length that depends on the simulation time step. The reservation point cannot
move faster than one cell per simulation step. If the reservation point can not be moved because of a locked (occupied)
cell, the train must stop.
'''

# import all flatland dependance

from typing import Dict, Union

from flatland.core.env_observation_builder import ObservationBuilder
from flatland.envs.observations import GlobalObsForRailEnv
from flatland.envs.rail_env_action import RailEnvActions

from flatland_extensions.RailroadSwitchCluster import RailroadSwitchCluster
from flatland_extensions.environment_extensions.FlatlandResourceAllocator import FlatlandResourceAllocator
from flatland_extensions.environment_extensions.XDynamicAgent import XDynamicAgent
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

        # Flatland Resource Allocator
        self._flatland_resource_allocator: Union[FlatlandResourceAllocator, None] = None

        # Railroad Switch Cluster
        self._railroad_switch_cluster: Union[RailroadSwitchCluster, None] = None
        self._railroad_switch_cluster_switch_group_locking = False
        self._railroad_switch_cluster_connecting_edge_locking = False

    def activate_flatland_resource_allocator(self, flatland_resource_allocator: FlatlandResourceAllocator):
        self._flatland_resource_allocator = flatland_resource_allocator

    def deactivate_flatland_resource_allocator(self):
        self._flatland_resource_allocator = None

    def activate_railroad_switch_cluster_locking(self,
                                                 railroad_switch_cluster: RailroadSwitchCluster,
                                                 railroad_switch_cluster_switch_group_locking=True,
                                                 railroad_switch_cluster_connecting_edge_locking=True):
        self._railroad_switch_cluster = railroad_switch_cluster
        self._railroad_switch_cluster_switch_group_locking = railroad_switch_cluster_switch_group_locking
        self._railroad_switch_cluster_connecting_edge_locking = railroad_switch_cluster_connecting_edge_locking

    def deactivate_railroad_switch_cluster_locking(self):
        self._railroad_switch_cluster = None
        self._railroad_switch_cluster_switch_group_locking = False
        self._railroad_switch_cluster_connecting_edge_locking = False

    def reset_agents(self):
        super(XRailEnv, self).reset_agents()
        x_dynamic_agents = []
        for agent in self.agents:
            x_dynamic_agents.append(XDynamicAgent(agent))
        self.agents = x_dynamic_agents

    def step(self, action_dict_: Dict[int, RailEnvActions]):
        if self._flatland_resource_allocator is not None:
            self._flatland_resource_allocator.reset_locks()
            for agent_handle, agent in enumerate(self.agents):
                agent.all_resource_ok(self.allocate_current_resources(agent))
        else:
            for agent_handle, agent in enumerate(self.agents):
                agent.all_resource_ok(True)

        observations, all_rewards, done, info = super(XRailEnv, self).step(action_dict_=action_dict_)

        self.dones["__all__"] = False
        for agent in self.agents:
            agent.update_agent()

        return observations, all_rewards, done, info

    def _handle_end_reward(self, agent):
        return 0

    def post_preprocess_action(self, action, agent):
        preprocessed_action = action
        if not agent.update_movement_dynamics():
            self.motionCheck.addAgent(agent.handle, agent.position, agent.position)
            preprocessed_action = RailEnvActions.STOP_MOVING

        if preprocessed_action == RailEnvActions.DO_NOTHING:
            preprocessed_action = RailEnvActions.MOVE_FORWARD

        return preprocessed_action
