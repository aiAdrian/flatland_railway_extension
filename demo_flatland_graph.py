# http://recife.univ-eiffel.fr/sharedData/data_format_documentation/

# import all flatland dependance
import time

from flatland_extensions.FlatlandEnvironmentHelper import FlatlandEnvironmentHelper
from flatland_extensions.FlatlandGraphBuilder import FlatlandGraphBuilder
from flatland_extensions.utils.FlatlandRenderer import FlatlandRenderer
from flatland_extensions.RailroadSwitchAnalyser import RailroadSwitchAnalyser
from flatland_extensions.RailroadSwitchCluster import RailroadSwitchCluster


def show_shortest_path(flatland_graph_builder: FlatlandGraphBuilder):
    # ---------- FlatlandGraphBuilder--------------------------------------------------------------------------------------
    flatland_renderer = FlatlandRenderer(env=flatland_environment_helper.get_rail_env())
    for agent_handle in flatland_environment_helper.get_rail_env().get_agent_handles():
        print('Show shortest paths for agent {}'.format(agent_handle))
        agent_pos, agent_dir, agent_state, agent_target, agent_is_off_map = \
            flatland_environment_helper.get_agent_position_and_direction(handle=agent_handle)
        flatland_environment_helper.get_rail_env().agents[agent_handle].position = agent_pos
        flatland_graph_builder.prepare_observation_data_plot(agent_pos, agent_dir, agent_target)

        flatland_renderer.render()
        time.sleep(0.5)

        flatland_environment_helper.get_rail_env().agents[agent_handle].position = None
    flatland_renderer.close()


# =====================================================================================================================
# Flatland
flatland_environment_helper = FlatlandEnvironmentHelper(random_seed=2341)
railroad_switch_analyser = RailroadSwitchAnalyser(env=flatland_environment_helper.get_rail_env())
railroad_switch_cluster = RailroadSwitchCluster(railroad_switch_analyser=railroad_switch_analyser)
flatland_graph_builder = FlatlandGraphBuilder(railroad_switch_analyser=railroad_switch_analyser)

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
