from typing import Dict, Union

from flatland.envs.rail_env import RailEnv
from flatland.envs.rail_env_action import RailEnvActions

from flatland_extensions.environment_extensions.FlatlandResourceAllocator import FlatlandResourceAllocator


class XRailEnv(RailEnv):
    def step(self, action_dict_: Dict[int, RailEnvActions]):
        super(XRailEnv).step(action_dict_)

        self._flatland_resource_allocator: Union[FlatlandResourceAllocator, None] = None

    def activate_flatland_resource_allocator(self, flatland_resource_allocator: FlatlandResourceAllocator):
        self._flatland_resource_allocator = flatland_resource_allocator

    def deactivate_flatland_resource_allocator(self):
        self._flatland_resource_allocator = None

    def preprocess_action(self, action, agent):
        preprocessed_action = super(XRailEnv).preprocess_action(action, agent)
        if self._flatland_resource_allocator is not None:
            locked_resource = self._flatland_resource_allocator.get_assigned_resources(agent.handle)
            for res in locked_resource:
                self.motionCheck.addAgent(agent.handle, res, res)
        return preprocessed_action
