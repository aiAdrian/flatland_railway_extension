# http://recife.univ-eiffel.fr/sharedData/data_format_documentation/

# import all flatland dependance
import time

from data_exports.FlatlandRecifeExporter import FlatlandRecifeExporter
from flatland_railway_extension.FlatlandEnvironmentHelper import FlatlandEnvironmentHelper
from flatland_railway_extension.FlatlandGraphBuilder import FlatlandGraphBuilder
from flatland_railway_extension.RailroadSwitchAnalyser import RailroadSwitchAnalyser
from flatland_railway_extension.RailroadSwitchCluster import RailroadSwitchCluster
from flatland_railway_extension.utils.FlatlandRenderer import FlatlandRenderer


def print_agent_information(
        flatland_environment_helper: FlatlandEnvironmentHelper,
        railroad_switch_analyser: RailroadSwitchAnalyser):
    for handle in railroad_switch_analyser.env.get_agent_handles():
        agent_pos, agent_dir, agent_state, agent_target, agent_is_off_map = \
            flatland_environment_helper.get_agent_position_and_direction(handle=handle)
        agent_at_railroad_switch, agent_near_to_railroad_switch, \
        agent_at_railroad_switch_cell, agent_near_to_railroad_switch_cell = \
            railroad_switch_analyser.check_agent_decision(position=agent_pos, direction=agent_dir)

        print(
            'agent {}\n\tposition: {}\n\tstart_direction: {}\n\tstate: {}\n\ttarget: {}\n\tis agent off map: {}'.format(
                handle, agent_pos, agent_dir, agent_state, agent_target, agent_is_off_map))
        print('\tagent is at railroad switch: {}\n\tagent is near to a railroad switch: {}'.format(
            agent_at_railroad_switch, agent_near_to_railroad_switch))
        print('\tagent is on a railroad switch cell: {}\n\tagent is near to a railroad switch cell: {}'.format(
            agent_near_to_railroad_switch_cell, agent_at_railroad_switch_cell))


# =====================================================================================================================
# Flatland
# =====================================================================================================================
flatland_environment_helper = FlatlandEnvironmentHelper(random_seed=2341)
railroad_switch_analyser = RailroadSwitchAnalyser(env=flatland_environment_helper.get_rail_env())
railroad_switch_cluster = RailroadSwitchCluster(railroad_switch_analyser=railroad_switch_analyser)
flatland_graph_builder = FlatlandGraphBuilder(railroad_switch_analyser=railroad_switch_analyser)

# ---------------------------------------------------------------------------------------------------------------------
flatland_exporter = FlatlandRecifeExporter(flatland_graph_builder=flatland_graph_builder, filename='../filename.xml')
# ---------------------------------------------------------------------------------------------------------------------


# =====================================================================================================================
# Debug rendering / plots / print-outs
# =====================================================================================================================

# ---------- RailroadSwitchAnalyser------------------------------------------------------------------------------------
railroad_switch_analyser.prepare_observation_data_plot()

railroad_switch_analyser.do_debug_plot()

flatland_renderer = FlatlandRenderer(env=flatland_environment_helper.get_rail_env())
flatland_renderer.start_render_loop(show_rowcols=True, disable_background_rendering=True)
flatland_renderer.close()

print_agent_information(flatland_environment_helper, railroad_switch_analyser)

# ---------- RailroadSwitchCluster-------------------------------------------------------------------------------------
railroad_switch_cluster.do_debug_plot()

# ---------- FlatlandGraphBuilder--------------------------------------------------------------------------------------
flatland_renderer = FlatlandRenderer(env=flatland_environment_helper.get_rail_env())
for agent_handle in flatland_environment_helper.get_rail_env().get_agent_handles():
    agent_pos, agent_dir, agent_state, agent_target, agent_is_off_map = \
        flatland_environment_helper.get_agent_position_and_direction(handle=agent_handle)
    flatland_environment_helper.get_rail_env().agents[agent_handle].position = agent_pos
    flatland_graph_builder.prepare_observation_data_plot(agent_pos, agent_dir, agent_target)

    flatland_renderer.render(show_rowcols=True, disable_background_rendering=True)
    time.sleep(0.5)

    flatland_environment_helper.get_rail_env().agents[agent_handle].position = None

flatland_renderer.close()

# ---------- FlatlandGraphBuilder--------------------------------------------------------------------------------------
flatland_graph_builder.render()
