import time
from ctypes import Union

import numpy as np
from flatland.envs.rail_env import RailEnv
from flatland.envs.rail_env_action import RailEnvActions

from flatland_railway_extension.FlatlandEnvironmentHelper import FlatlandEnvironmentHelper
from flatland_railway_extension.RailroadSwitchAnalyser import RailroadSwitchAnalyser
from flatland_railway_extension.RailroadSwitchCluster import RailroadSwitchCluster
from flatland_railway_extension.environments.DynamicAgent import DynamicAgent
from flatland_railway_extension.environments.FlatlandDynamics import FlatlandDynamics
from flatland_railway_extension.environments.FlatlandResourceAllocator import FlatlandResourceAllocator
from flatland_railway_extension.environments.InfrastructureData import InfrastructureData
from flatland_railway_extension.environments.MultiResourcesAllocationRailEnv import MultiResourcesAllocationRailEnv
from flatland_railway_extension.utils.FlatlandDynamicsRenderer import FlatlandDynamicsRenderer
from flatland_railway_extension.utils.FlatlandRenderer import FlatlandRenderer


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


def map_infrastructure_data(env: RailEnv, railroad_switch_analyser: RailroadSwitchAnalyser):
    # Create a test infrastructure
    # ---------------------------------------------------------------------------------------------------------------
    # share the infrastructure with the agents ( train runs)
    for agent in env.agents:
        if isinstance(agent, DynamicAgent):
            agent.set_infrastructure_data(
                create_infrastructure_data(env, railroad_switch_analyser)
            )
            agent.rolling_stock.set_max_braking_acceleration(-0.15)
            agent.set_mass(500)
            agent.set_tractive_effort_rendering(False)
    if isinstance(env, FlatlandDynamics):
        env.set_infrastructure_data(create_infrastructure_data(env, railroad_switch_analyser))


def run_simulation(flatland_environment_helper: FlatlandEnvironmentHelper,
                   railroad_switch_cluster: RailroadSwitchCluster,
                   railroad_switch_analyser: RailroadSwitchAnalyser,
                   enable_moving_block_resource_allocation_strategy=False,
                   enable_rendering=False):
    env: RailEnv = flatland_environment_helper.get_rail_env()
    observations, info = env.reset()

    flatland_renderer = None
    if enable_rendering:
        if isinstance(env, FlatlandDynamics):
            flatland_renderer = FlatlandDynamicsRenderer(
                env=flatland_environment_helper.get_rail_env(),
                show_debug=True,
                show_agents=False,
                fix_aspect_ration=False)
        else:
            flatland_renderer = FlatlandRenderer(
                env=flatland_environment_helper.get_rail_env(),
                show_debug=True,
                show_agents=True,
                fix_aspect_ration=False)

    map_infrastructure_data(env=flatland_environment_helper.get_rail_env(),
                            railroad_switch_analyser=railroad_switch_analyser)

    # ---------------------------------------------------------------------------------------------------------------
    # Start simulation
    # ---------------------------------------------------------------------------------------------------------------
    flatland_resource_allocator: Union(FlatlandResourceAllocator, None) = None
    if isinstance(env, MultiResourcesAllocationRailEnv):
        flatland_resource_allocator = env.get_active_flatland_resource_allocator()
        if flatland_resource_allocator is not None:
            flatland_resource_allocator.set_minimal_free_time_to_reallocate_other_agent(120)

    for step in range(10000):
        if flatland_renderer is not None and isinstance(flatland_renderer, FlatlandDynamicsRenderer):
            flatland_renderer.set_flatland_resource_allocator(flatland_resource_allocator)
        if not enable_moving_block_resource_allocation_strategy:
            if isinstance(env, MultiResourcesAllocationRailEnv):
                env.activate_railroad_switch_cluster_locking(railroad_switch_cluster)

        actions = {}
        for agent_handle in env.get_agent_handles():
            obs = observations[agent_handle]
            actions.update({agent_handle: RailEnvActions(obs[0])})
        observations, all_rewards, dones, info = env.step(actions)

        if flatland_renderer is not None:
            if step % 10 == 0 or not isinstance(env, FlatlandDynamics):
                flatland_renderer.render(show_observations=True, disable_background_rendering=True)
                time.sleep(0.01)

        if dones["__all__"]:
            break

        if flatland_renderer is not None:
            if flatland_renderer.is_closed():
                break

    if flatland_renderer is not None:
        if not flatland_renderer.is_closed():
            print("Please close render window to exit")
            flatland_renderer.start_render_loop()
        flatland_renderer.close()

    if enable_rendering:
        if isinstance(env, FlatlandDynamics):
            env.agents[3].do_debug_plot(1, 1, True)
            for i_agent, agent in enumerate(env.agents):
                n_agents = env.get_num_agents()
                agent.do_debug_plot(i_agent + 1, n_agents, i_agent + 1 == n_agents, i_agent == 0)


# -----------------------------------------------------------------------------------------------------------------
flatland_environment_helper = FlatlandEnvironmentHelper(rail_env=FlatlandDynamics,
                                                        number_of_agents=10,
                                                        random_seed=2341)
railroad_switch_analyser = RailroadSwitchAnalyser(env=flatland_environment_helper.get_rail_env())
railroad_switch_cluster = RailroadSwitchCluster(railroad_switch_analyser=railroad_switch_analyser)

run_simulation(flatland_environment_helper,
               railroad_switch_cluster,
               railroad_switch_analyser,
               enable_moving_block_resource_allocation_strategy=True,
               enable_rendering=True)
