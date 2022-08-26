import time

from flatland.envs.rail_env_action import RailEnvActions

from flatland_extensions.FlatlandEnvironmentHelper import FlatlandEnvironmentHelper
from flatland_extensions.FlatlandRenderer import FlatlandRenderer
from flatland_extensions.RailroadSwitchAnalyser import RailroadSwitchAnalyser
from flatland_extensions.RailroadSwitchCluster import RailroadSwitchCluster
from flatland_extensions.environment_extensions.FlatlandResourceAllocator import FlatlandResourceAllocator


def run_simulation(flatland_environment_helper: FlatlandEnvironmentHelper,
                   railroad_switch_cluster: RailroadSwitchCluster,
                   use_cluster_locking=False,
                   minimal_train_following_time=0):
    env = flatland_environment_helper.get_rail_env()
    observations, info = env.reset()

    flatland_renderer = FlatlandRenderer(env=flatland_environment_helper.get_rail_env())
    flatland_resource_allocator = FlatlandResourceAllocator(env=flatland_environment_helper.get_rail_env())
    flatland_resource_allocator.set_minimal_free_time_to_reallocate_other_agent(minimal_train_following_time)
    flatland_environment_helper.get_rail_env().activate_flatland_resource_allocator(flatland_resource_allocator)
    if use_cluster_locking:
        flatland_environment_helper.get_rail_env().activate_railroad_switch_cluster_locking(railroad_switch_cluster)

    for step in range(1000):
        actions = {}
        for agent_handle in flatland_environment_helper.get_rail_env().get_agent_handles():
            obs = observations[agent_handle]
            actions.update({agent_handle: RailEnvActions(obs[0])})

        observations, all_rewards, dones, info = env.step(actions)

        for agent_handle in flatland_environment_helper.get_rail_env().get_agent_handles():
            env = flatland_environment_helper.get_rail_env()
            env.dev_obs_dict.update(
                {agent_handle: flatland_resource_allocator.get_assigned_resources(agent_handle=agent_handle)})
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
    flatland_resource_allocator.do_debug_plot()


# -----------------------------------------------------------------------------------------------------------------
flatland_environment_helper = FlatlandEnvironmentHelper(random_seed=2341)
railroad_switch_analyser = RailroadSwitchAnalyser(env=flatland_environment_helper.get_rail_env())
railroad_switch_cluster = RailroadSwitchCluster(railroad_switch_analyser=railroad_switch_analyser)

run_simulation(flatland_environment_helper, railroad_switch_cluster,
               use_cluster_locking=False,
               minimal_train_following_time=0)
run_simulation(flatland_environment_helper, railroad_switch_cluster,
               use_cluster_locking=False,
               minimal_train_following_time=10)

run_simulation(flatland_environment_helper, railroad_switch_cluster,
               use_cluster_locking=True,
               minimal_train_following_time=0)
run_simulation(flatland_environment_helper, railroad_switch_cluster,
               use_cluster_locking=True,
               minimal_train_following_time=10)
