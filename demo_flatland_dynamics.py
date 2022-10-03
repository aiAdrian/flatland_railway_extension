import time

import numpy as np
from flatland.envs.rail_env import RailEnv
from flatland.envs.rail_env_action import RailEnvActions

from flatland_extensions.FlatlandEnvironmentHelper import FlatlandEnvironmentHelper
from flatland_extensions.RailroadSwitchAnalyser import RailroadSwitchAnalyser
from flatland_extensions.RailroadSwitchCluster import RailroadSwitchCluster
from flatland_extensions.environment_extensions.FlatlandDynamics import FlatlandDynamics
from flatland_extensions.environment_extensions.FlatlandResourceAllocator import FlatlandResourceAllocator
from flatland_extensions.environment_extensions.InfrastructureData import InfrastructureData
from flatland_extensions.utils.FlatlandDynamicsRenderer import FlatlandDynamicsRenderer


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


def run_simulation(flatland_environment_helper: FlatlandEnvironmentHelper,
                   railroad_switch_cluster: RailroadSwitchCluster,
                   railroad_switch_analyser: RailroadSwitchAnalyser,
                   enable_moving_block_resource_allocation_strategy=False):
    env = flatland_environment_helper.get_rail_env()
    observations, info = env.reset()

    flatland_renderer = FlatlandDynamicsRenderer(
        env=flatland_environment_helper.get_rail_env(),
        show_debug=True,
        show_agents=False)

    # Create a test infrastructure
    # ---------------------------------------------------------------------------------------------------------------
    # share the infrastructure with the agents ( train runs)
    for agent in env.agents:
        agent.set_infrastructure_data(
            create_infrastructure_data(flatland_environment_helper.get_rail_env(), railroad_switch_analyser)
        )
        agent.rolling_stock.a_max_braking = -0.15
        agent.set_mass(500)

    # ---------------------------------------------------------------------------------------------------------------
    # Start simulation
    # ---------------------------------------------------------------------------------------------------------------
    flatland_resource_allocator = FlatlandResourceAllocator(env=flatland_environment_helper.get_rail_env())
    flatland_resource_allocator.set_minimal_free_time_to_reallocate_other_agent(120)

    for step in range(10000):
        flatland_renderer.set_flatland_resource_allocator(flatland_resource_allocator)
        flatland_environment_helper.get_rail_env().activate_flatland_resource_allocator(flatland_resource_allocator)
        if not enable_moving_block_resource_allocation_strategy:
            flatland_environment_helper.get_rail_env().activate_railroad_switch_cluster_locking(railroad_switch_cluster)

        actions = {}
        for agent_handle in flatland_environment_helper.get_rail_env().get_agent_handles():
            obs = observations[agent_handle]
            actions.update({agent_handle: RailEnvActions(obs[0])})

        observations, all_rewards, dones, info = env.step(actions)

        if step % 10 == 0:
            flatland_renderer.render(show_observations=True)
            time.sleep(0.01)

        if dones["__all__"]:
            break
        if flatland_renderer.is_closed():
            break

    if not flatland_renderer.is_closed():
        print("Please close render window to exit")
        flatland_renderer.start_render_loop()
    flatland_renderer.close()

    flatland_environment_helper.get_rail_env().agents[3].do_debug_plot(1, 1, True)
    for i_agent, agent in enumerate(flatland_environment_helper.get_rail_env().agents):
        n_agents = flatland_environment_helper.get_rail_env().get_num_agents()
        agent.do_debug_plot(i_agent + 1, n_agents, i_agent + 1 == n_agents, i_agent == 0)


# -----------------------------------------------------------------------------------------------------------------
flatland_environment_helper = FlatlandEnvironmentHelper(rail_env=FlatlandDynamics, random_seed=2341)
railroad_switch_analyser = RailroadSwitchAnalyser(env=flatland_environment_helper.get_rail_env())
railroad_switch_cluster = RailroadSwitchCluster(railroad_switch_analyser=railroad_switch_analyser)

run_simulation(flatland_environment_helper,
               railroad_switch_cluster,
               railroad_switch_analyser,
               enable_moving_block_resource_allocation_strategy=False)
