'''
The Flatland simulator does not have any train run dynamics implemented. Flatland Dynamics extends flatland to simulate
railway dynamics with a dynamics/physical layer. This code meant as a proof-of-concept to show how dynamics can be
integrated. The goal must be that all trains(agents) follow physical policy to accelerate. The acceleration must be
based on real train parameters such as train weight, train length and traction power. The braking will be implemented
with a fixed physical delay. The physical delay will be a parameter which can be set per train (agent).

To simulate dynamics the simulator must be ensure that each train(agent) can break just before it will collide by any
others. This requires a new safety layer. Up to now flatland just ensures that there will be no more than maximum one
agent per cell. And a train (agent) can only be at one cell at once. This must be extended as well. Due of the
braking distance which can vary from zero (train doesn’t move) up to many meters (train is moving). The braking
distance strongly depends on the train speed. Thus, flatland has to ensure that one train (agent) can allocate many
cells and free them later. The allocation and freeze will no longer be restricted to one cell. Thus a cell can have
more than occupied or free. It can also be in state reserved. Reserved will mean that the train is not yet in the
cell by the cell is still occupied due of it’s braking distance. The state reserved must not be implemented.
It can be implemented with occupied. But it’s important to understand that a train (agent) has more than two “cell
reservation state per cell”. Once a cell is locked (reserved, occupied) it will be for all other agents occupied
and they cannot enter.

Implementation idea: The simulator simulates with current agent position and direction the reservation point – this
is in flatland the current behavior. The agent can navigate as it will  with default flatland. Instead of the
real train run, we simulate virtual train run. braking distance and train (agent). But we must change the
occupied handling. A new feature is, that we must simulate the free point. The free point simulates the end of the
train. When the end of the train leaves a cell - it will be freezing the occupied cell. The dynamics will calculate
the speed for allocating (reservation point) – braking distance and train run speed. Each cell needs the "distance"
or "length" information. There will be a minimal cell length, which depends on the simulation time step.
The reservation point cannot move faster than one cell per simulation step. If the reservation point cannot be
moved due of a locked (occupied) cell, the train has to break.
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
