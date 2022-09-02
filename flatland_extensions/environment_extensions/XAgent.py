from typing import Tuple, List

from flatland.envs.agent_utils import EnvAgent


class XAgent(EnvAgent):
    def __init__(self, original_env_agent: EnvAgent):
        super(XAgent, self).__init__(original_env_agent.initial_position,
                                     original_env_agent.initial_direction, original_env_agent.direction,
                                     original_env_agent.target)

        # copy all attributes data from original EnvAgent allocated in RailEnv
        self._copy_attribute_from_env_agent(original_env_agent)

    def _copy_attribute_from_env_agent(self, env_agent: EnvAgent):
        '''
        Copy all class attribute and it's value from EnvAgent to XDynamicAgent
        :param env_agent: The original agent created in the RailEnv
        '''
        for attribute, value in env_agent.__dict__.items():
            setattr(self, attribute, value)

    def get_allocated_resource(self) -> List[Tuple[int, int]]:
        if self.position is not None:
            return [self.position]
        return []

    def all_resource_ok(self, resource_allocation_ok):
        pass

    def update_agent(self):
        pass

    def reset(self):
        super(XAgent, self).reset()

    def do_debug_plot(self, idx=1, nbr_agents=1, show=True):
        pass
