import random

import numpy as np
# import all flatland dependances
from flatland.envs.rail_env import RailEnv
from flatland.envs.rail_generators import sparse_rail_generator

from flatland_railway_extensions.utils.ShortestPathNextStepObservation import ShortestPathNextStepObservation


class FlatlandEnvironmentHelper:
    def __init__(self, rail_env=RailEnv, grid_width=30, grid_height=40, number_of_agents=10, n_cities=3,
                 random_seed=0):
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.number_of_agents = number_of_agents
        self.n_cities = n_cities
        self._random_seed(random_seed)
        self.env = self._create_flatland_env(rail_env)
        self.env.reset()

    def _random_seed(self, random_seed):
        self.random_seed = random_seed
        np.random.seed(self.random_seed)
        random.seed(self.random_seed)

    def _create_flatland_env(self, rail_env, max_rails_between_cities=2, max_rails_in_city=4) -> RailEnv:
        return rail_env(
            width=self.grid_width,
            height=self.grid_height,
            rail_generator=sparse_rail_generator(
                max_num_cities=self.n_cities,
                seed=self.random_seed,
                grid_mode=True,
                max_rails_between_cities=max_rails_between_cities,
                max_rail_pairs_in_city=max_rails_in_city
            ),
            random_seed=self.random_seed,
            number_of_agents=self.number_of_agents,
            obs_builder_object=ShortestPathNextStepObservation()
        )

    def get_rail_env(self):
        return self.env

    def get_agent_position_and_direction(self, handle):
        '''
        Returns the agent position - if not yet started (active) it returns the initial position

        :param handle: agent reference (handle)

        :return: agent_pos, agent_dir, agent_state, agent_target, is_agent_off_map
        '''
        agent = self.env.agents[handle]
        agent_pos = agent.position
        agent_dir = agent.direction
        if agent_pos is None:
            agent_pos = agent.initial_position
            agent_dir = agent.initial_direction
        return agent_pos, agent_dir, agent.state, agent.target, agent.position == None
