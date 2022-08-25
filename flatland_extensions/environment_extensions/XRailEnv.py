from typing import Dict, Union

from flatland.core.env_observation_builder import ObservationBuilder
from flatland.envs.observations import GlobalObsForRailEnv
from flatland.envs.rail_env import RailEnv
from flatland.envs.rail_env_action import RailEnvActions
from flatland.envs.step_utils import env_utils

from flatland_extensions.RailroadSwitchCluster import RailroadSwitchCluster
from flatland_extensions.environment_extensions.FlatlandResourceAllocator import FlatlandResourceAllocator


class XRailEnv(RailEnv):
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
        super(XRailEnv, self).__init__(width=width,
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
        self._flatland_resource_allocator: Union[FlatlandResourceAllocator, None] = None
        self._railroad_switch_cluster: Union[RailroadSwitchCluster, None] = None
        self._railroad_switch_cluster_switch_group_locking = False
        self._railroad_switch_cluster_connecting_edge_locking = False

    def step(self, action_dict_: Dict[int, RailEnvActions]):
        observations, all_rewards, done, info = super(XRailEnv, self).step(action_dict_=action_dict_)
        return observations, all_rewards, done, info

    def activate_flatland_resource_allocator(self, flatland_resource_allocator: FlatlandResourceAllocator):
        self._flatland_resource_allocator = flatland_resource_allocator

    def deactivate_flatland_resource_allocator(self):
        self._flatland_resource_allocator = None

    def activate_railroad_switch_cluster_locking(self, railroad_switch_cluster: RailroadSwitchCluster):
        self._railroad_switch_cluster = railroad_switch_cluster
        self._railroad_switch_cluster_switch_group_locking = True
        self._railroad_switch_cluster_connecting_edge_locking = True

    def deactivate_railroad_switch_cluster_locking(self):
        self._railroad_switch_cluster = None
        self._railroad_switch_cluster_switch_group_locking = False
        self._railroad_switch_cluster_connecting_edge_locking = False

    def preprocess_action(self, action, agent):
        preprocessed_action = super(XRailEnv, self).preprocess_action(action, agent)
        action_is_valid = True
        if self._flatland_resource_allocator is not None:
            # Try moving actions on current position
            current_position, current_direction = agent.position, agent.direction
            if current_position is None:  # Agent not added on map yet
                current_position, current_direction = agent.initial_position, agent.initial_direction

            new_position, new_direction = env_utils.apply_action_independent(preprocessed_action,
                                                                             self.rail,
                                                                             current_position,
                                                                             current_direction)
            resources = [new_position]
            if self._railroad_switch_cluster is not None:
                current_cluster_id = self._railroad_switch_cluster.get_cluster_id(current_position)
                cluster_id = self._railroad_switch_cluster.get_cluster_id(new_position)
                cluster_member = self._railroad_switch_cluster.get_cluster_cell_members(cluster_id)
                if current_cluster_id.switch_cluster_ref != cluster_id.switch_cluster_ref:
                    if self._railroad_switch_cluster_switch_group_locking:
                        for r in cluster_member.switch_cluster_cell_members:
                            resources.append(r)
                if current_cluster_id.connecting_edge_cluster_ref != cluster_id.connecting_edge_cluster_ref:
                    if self._railroad_switch_cluster_connecting_edge_locking:
                        for r in cluster_member.switch_cluster_cell_members:
                            resources.append(r)
            action_is_valid = self._flatland_resource_allocator.allocate_resource(agent.handle, resources)

            if action_is_valid:
                for r in resources:
                    self.motionCheck.addAgent(agent.handle, agent.position, r)

            if not action_is_valid:
                preprocessed_action = RailEnvActions.STOP_MOVING
        return preprocessed_action
