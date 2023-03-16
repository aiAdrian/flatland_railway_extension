from typing import Dict, Union, Tuple, List

from flatland.core.env_observation_builder import ObservationBuilder
from flatland.envs.observations import GlobalObsForRailEnv
from flatland.envs.rail_env import RailEnv
from flatland.envs.rail_env_action import RailEnvActions
from flatland.envs.step_utils import env_utils

from flatland_railway_extension.RailroadSwitchCluster import RailroadSwitchCluster
from flatland_railway_extension.environments.FlatlandResourceAllocator import FlatlandResourceAllocator
from flatland_railway_extension.environments.XAgent import XAgent


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
        super(XRailEnv, self).__init__(
            width=width,
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

    def get_active_flatland_resource_allocator(self) -> FlatlandResourceAllocator:
        return self._flatland_resource_allocator

    def is_flatland_resource_allocator_activated(self) -> bool:
        return self._flatland_resource_allocator is not None

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

    def _allocate_resources(self, agent: XAgent, positions: List[Tuple[int, int]]):
        if self._flatland_resource_allocator is None:
            return True
        resources = positions
        if self._railroad_switch_cluster is not None:
            for position in positions:
                if position is not None:
                    cluster_id = self._railroad_switch_cluster.get_cluster_id(position)
                    cluster_member = self._railroad_switch_cluster.get_cluster_cell_members(cluster_id)
                    if self._railroad_switch_cluster_switch_group_locking:
                        for r in cluster_member.switch_cluster_cell_members:
                            if r not in resources:
                                resources.append(r)
                    if self._railroad_switch_cluster_connecting_edge_locking:
                        for r in cluster_member.connecting_edge_cluster_cell_members:
                            if r not in resources:
                                resources.append(r)
        return self._flatland_resource_allocator.allocate_resource(agent.handle, resources)

    def allocate_resources_at_position(self, agent: XAgent, position: Tuple[int, int]) -> bool:
        return self._allocate_resources(agent, [position])

    def allocate_current_resources(self, agent: XAgent) -> bool:
        positions = agent.get_allocated_resource()
        if len(positions) == 0 and agent.position is not None:
            positions.append(agent.position)
        return self._allocate_resources(agent, positions)

    def reset_agents(self):
        super(XRailEnv, self).reset_agents()
        x_agents = []
        for agent in self.agents:
            x_agents.append(XAgent(agent))
        self.agents = x_agents

    def step(self, action_dict_: Dict[int, RailEnvActions]):
        if self._flatland_resource_allocator is not None:
            self._flatland_resource_allocator.reset_locks()
            for agent_handle, agent in enumerate(self.agents):
                agent.all_resource_ok(self.allocate_current_resources(agent))
        else:
            for agent_handle, agent in enumerate(self.agents):
                agent.all_resource_ok(True)

        observations, all_rewards, done, info = super(XRailEnv, self).step(action_dict_=action_dict_)

        for agent in self.agents:
            agent.update_agent()

        return observations, all_rewards, done, info

    def set_max_episode_steps(self, max_episode_steps):
        self._max_episode_steps = max_episode_steps

    def _handle_end_reward(self, agent):
        return 0

    def post_preprocess_action(self, action, agent):
        return action

    def preprocess_action(self, action, agent):
        preprocessed_action = super(XRailEnv, self).preprocess_action(action, agent)

        if agent.is_done():
            preprocessed_action = RailEnvActions.STOP_MOVING
        else:
            if self._flatland_resource_allocator is not None:
                # Try moving actions on current position
                current_position, current_direction = agent.position, agent.direction
                if current_position is None:  # Agent not added on map yet
                    current_position, current_direction = agent.initial_position, agent.initial_direction

                new_position, new_direction = env_utils.apply_action_independent(
                    preprocessed_action,
                    self.rail,
                    current_position,
                    current_direction)

                if preprocessed_action.is_moving_action():
                    if not self.allocate_resources_at_position(agent, new_position):
                        agent.all_resource_ok(False)
                        self.motionCheck.addAgent(agent.handle, agent.position, agent.position)
                        preprocessed_action = RailEnvActions.STOP_MOVING

        preprocessed_action = self.post_preprocess_action(preprocessed_action, agent)

        return preprocessed_action
