from functools import lru_cache
from typing import Tuple

from flatland_extensions.environment_extensions.DynamicsResourceData import DynamicsResourceData
from flatland_extensions.environment_extensions.InfrastructureData import InfrastructureData
from flatland_extensions.environment_extensions.RollingStock import RollingStock


infrastructure_lru_cache_functions = []
def enable_infrastructure_data_lru_cache(*args, **kwargs):
    def decorator(func):
        func = lru_cache(*args, **kwargs)(func)
        infrastructure_lru_cache_functions.append(func)
        return func

    return decorator


def reset_infrastructure_data_lru_cache():
    for func in infrastructure_lru_cache_functions:
        func.cache_clear()


@lru_cache()
def min_cached(a, b):
    return min(a, b)


@lru_cache()
def max_cached(a, b):
    return max(a, b)


@enable_infrastructure_data_lru_cache(maxsize=1_000_000)
def get_cached_dynamics_resource_data(res: Tuple[int, int],
                                       infrastructure_data: InfrastructureData) -> DynamicsResourceData:
    return DynamicsResourceData(res, infrastructure_data)


@lru_cache(maxsize=4096)
def get_cached_accelerations_and_tractive_effort(obj: RollingStock,
                                                 current_velocity,
                                                 max_allowed_velocity,
                                                 current_gradient,
                                                 train_total_mass,
                                                 simulation_time_step):
    return obj.get_accelerations_and_tractive_effort(current_velocity,
                                                     max_allowed_velocity,
                                                     current_gradient,
                                                     train_total_mass,
                                                     simulation_time_step)
