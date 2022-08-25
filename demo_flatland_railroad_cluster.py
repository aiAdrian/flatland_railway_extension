from flatland_extensions.FlatlandEnvironmentHelper import FlatlandEnvironmentHelper
from flatland_extensions.environment_extensions.FlatlandResourceAllocator import FlatlandResourceAllocator
from flatland_extensions.RailroadSwitchAnalyser import RailroadSwitchAnalyser
from flatland_extensions.RailroadSwitchCluster import RailroadSwitchCluster

flatland_environment_helper = FlatlandEnvironmentHelper(random_seed=2341)
flatland_resource_allocator = FlatlandResourceAllocator(flatland_environment_helper=flatland_environment_helper)
railroad_switch_analyser = RailroadSwitchAnalyser(env=flatland_environment_helper.get_rail_env())
railroad_switch_cluster = RailroadSwitchCluster(railroad_switch_analyser=railroad_switch_analyser)

agent_pos, agent_dir, agent_state, agent_target, is_agent_off_map = \
    flatland_environment_helper.get_agent_position_and_direction(handle=0)

# -----------------------------------------------------------------------------------------------------------------
print('Test: agent_pos', agent_pos)
agent_pos_cluster_id = railroad_switch_cluster.get_cluster_id(agent_pos)
agent_pos_cluster_cell_members = railroad_switch_cluster.get_cluster_cell_members(agent_pos_cluster_id)

print(agent_pos_cluster_id)
print(agent_pos_cluster_cell_members)

# -----------------------------------------------------------------------------------------------------------------
print('Test: agent_target', agent_target)
agent_target_cluster_id = railroad_switch_cluster.get_cluster_id(agent_target)
agent_target_cluster_cell_members = railroad_switch_cluster.get_cluster_cell_members(agent_target_cluster_id)

print(agent_target_cluster_id)
print(agent_target_cluster_cell_members)

# -----------------------------------------------------------------------------------------------------------------
switch_pos = list(railroad_switch_analyser.railroad_switches.keys())[0]
print('Test: switch', switch_pos)
switch_pos_cluster_id = railroad_switch_cluster.get_cluster_id(switch_pos)
switch_pos_cluster_cell_members = railroad_switch_cluster.get_cluster_cell_members(switch_pos_cluster_id)

print(switch_pos_cluster_id)
print(switch_pos_cluster_cell_members)

# -----------------------------------------------------------------------------------------------------------------
flatland_resource_allocator.do_debug_plot()
print(
    flatland_resource_allocator.allocate_resource(agent_handle=0,
                                                  positions=agent_pos_cluster_cell_members.switch_cluster_cell_members),
    flatland_resource_allocator.allocate_resource(agent_handle=0,
                                                  positions=agent_pos_cluster_cell_members.connecting_edge_cluster_cell_members)
)
flatland_resource_allocator.do_debug_plot()
