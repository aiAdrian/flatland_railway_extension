# Idea: Adrian Egli / Erik Nygren
# Permission to use - If you use this or any idea out of this code for a
# publication, you must credit the authors. No commerical
# use allowed.

'''
The flatland simulator has no train running dynamics implemented. Flatland Dynamics extends Flatland to train/agent simulation
Movement with dynamics/physics layer. This code is intended as a proof of concept to show how dynamics can be
integrated. The goal of this extension is that all trains (agents) follow the physical movement policy for acceleration/braking. The acceleration must be
based on real train parameters such as train weight, train length and traction power (rolling stock). Braking is carried out
with fixed physical delay. The physical delay is a parameter that can be set per train (agent).

In order to simulate the dynamics, the simulator must ensure that each train (agent) can break before colliding from one
other. This requires a new security layer. So far, Flatland only ensures that there is no more than a maximum of one
active agent per cell. And an agent can only be in one cell at a time. This must also be extended. Due to the
braking distance, which can vary from zero (train is not moving) to many meters (train is moving). The braking
distance is highly dependent on train speed. So Flatland has to make sure that the agent can assign many
cells and free them later. Allocation and freezing are no longer limited to one cell.
The agent can reserve, hold and free a cell later on. Reserved means that the train has not yet arrived
at the cell but the cell is still occupied due to their braking distance. The reserved state must not be explicitly implemented.
Once a cell is locked (reserved, occupied), it is occupied for all other agents and they cannot enter anymore.

Implementation idea: The simulator simulates the reservation point with the current agent position and direction - this one
is the current behavior in the Flatland. The agent can navigate freely with the standard Flatland actions. Instead of
real train we simulate virtual the train at end of reservation area - braking distance and train lenght (agent) are locked for all
other agents. But we have to change that manned handling. What is new is that we have to simulate the free point. The free point
simulates the end of the train. If the end of the turn leaves a cell, the occupied cell is frozen. The dynamic is calculated
the speed for the assignment (reservation point) – braking distance and train speed. Every cell needs the "distance"
or "length" information. There is a minimum cell length that depends on the simulation time step.
The reservation point cannot move faster than one cell per simulation step. If the reservation point can not be
moved because of a locked (occupied) cell, the train must pause.
'''

# import all flatland dependance

from typing import Dict, Union, Tuple, List

from flatland.core.env_observation_builder import ObservationBuilder
from flatland.envs.observations import GlobalObsForRailEnv
from flatland.envs.rail_env_action import RailEnvActions
from flatland.envs.step_utils import env_utils

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
