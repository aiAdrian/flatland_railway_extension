# import all flatland dependance
import time

import numpy as np
from flatland.envs.rail_env import RailEnv

from flatland_railway_extension.FlatlandEnvironmentHelper import FlatlandEnvironmentHelper
from flatland_railway_extension.FlatlandGraphBuilder import FlatlandGraphBuilder
from flatland_railway_extension.RailroadSwitchAnalyser import RailroadSwitchAnalyser
from flatland_railway_extension.RailroadSwitchCluster import RailroadSwitchCluster
from flatland_railway_extension.environments.InfrastructureData import InfrastructureData
from flatland_railway_extension.utils.FlatlandRenderer import FlatlandRenderer


# http://recife.univ-eiffel.fr/sharedData/data_format_documentation/


def show_shortest_path(flatland_graph_builder: FlatlandGraphBuilder):
    # ---------- FlatlandGraphBuilder----------------------------------------------------------------------------------
    flatland_renderer = FlatlandRenderer(env=flatland_environment_helper.get_rail_env())
    for agent_handle in flatland_environment_helper.get_rail_env().get_agent_handles():
        print('Show shortest paths for agent {}'.format(agent_handle))
        agent_pos, agent_dir, agent_state, agent_target, agent_is_off_map = \
            flatland_environment_helper.get_agent_position_and_direction(handle=agent_handle)
        flatland_environment_helper.get_rail_env().agents[agent_handle].position = agent_pos
        flatland_graph_builder.prepare_observation_data_plot(agent_pos, agent_dir, agent_target, weight='length')

        flatland_renderer.render()
        time.sleep(0.5)

        flatland_environment_helper.get_rail_env().agents[agent_handle].position = None
    flatland_renderer.close()


def create_infrastructure_data(env: RailEnv, railroad_switch_analyser: RailroadSwitchAnalyser) -> InfrastructureData:
    infrastructure_data = InfrastructureData()
    cell_length_grid = np.ones((env.height, env.width)) * 400
    gradient_grid = np.zeros((env.height, env.width))
    velocity_grid = np.ones((env.height, env.width)) * 100
    for key in railroad_switch_analyser.railroad_switch_neighbours.keys():
        velocity_grid[key] = 80
    for key in railroad_switch_analyser.railroad_switches.keys():
        velocity_grid[key] = 60
    infrastructure_data.set_infrastructure_max_velocity_grid(velocity_grid / 3.6)
    infrastructure_data.set_infrastructure_cell_length_grid(cell_length_grid)
    infrastructure_data.set_infrastructure_gradient_grid(gradient_grid)
    return infrastructure_data


# =====================================================================================================================
# Flatland
flatland_environment_helper = FlatlandEnvironmentHelper(random_seed=2341)
railroad_switch_analyser = RailroadSwitchAnalyser(env=flatland_environment_helper.get_rail_env())
railroad_switch_cluster = RailroadSwitchCluster(railroad_switch_analyser=railroad_switch_analyser)
infrastructure_data = create_infrastructure_data(
    env=flatland_environment_helper.get_rail_env(),
    railroad_switch_analyser=railroad_switch_analyser)
flatland_graph_builder = FlatlandGraphBuilder(
    railroad_switch_analyser=railroad_switch_analyser,
    infrastructure_data=infrastructure_data)

print('==============================================================================================================')
print('Full graph')
print('==============================================================================================================')
flatland_graph_builder.activate_full_graph()
show_shortest_path(flatland_graph_builder)

print('==============================================================================================================')
print('Simplified graph')
print('==============================================================================================================')
flatland_graph_builder.activate_simplified()
show_shortest_path(flatland_graph_builder)
