# Idea: Adrian Egli / Erik Nygren
# Permission to use - If you use this or any idea out of this code for a
# publication, you must credit the authors. No commerical
# use allowed.
from functools import lru_cache
from typing import Tuple, List, Union

import numpy as np
from flatland.envs.agent_utils import EnvAgent
from matplotlib import pyplot as plt

from flatland_railway_extension.environments.DynamicsResourceData import DynamicsResourceData
from flatland_railway_extension.environments.InfrastructureData import InfrastructureData
from flatland_railway_extension.environments.RollingStock import RollingStock
from flatland_railway_extension.environments.XAgent import XAgent
from flatland_railway_extension.utils.cached_methods import min_cached, max_cached

_infrastructure_lru_cache_functions = []


def enable_infrastructure_data_lru_cache(*args, **kwargs):
    def decorator(func):
        func = lru_cache(*args, **kwargs)(func)
        _infrastructure_lru_cache_functions.append(func)
        return func

    return decorator


def reset_infrastructure_data_lru_cache():
    for func in _infrastructure_lru_cache_functions:
        func.cache_clear()


class DynamicAgent(XAgent):
    def __init__(self, original_env_agent: EnvAgent):
        super(DynamicAgent, self).__init__(original_env_agent)

        # set the extended dynamic agent attributes
        self.set_length(100)
        self.set_mass(300)
        self.set_rolling_stock(RollingStock())

        # infrastructure
        self.v_max_simulation = 300 / 3.6
        self._infrastructure_data: Union[InfrastructureData, None] = None

        # set the internal simulation data (resource allocation, velocity, ... )

        # current pointer and data to visited cells/resources
        self.visited_cell_path: List[Tuple[int, int]] = []
        self.visited_direction_path: List[Tuple[int, int]] = []
        self.visited_cell_distance: List[float] = []
        self.visited_cell_path_reservation_point_index = 0
        self.visited_cell_path_end_of_agent_index = 0
        self.visited_cell_path_start_of_agent_index = 0
        self.visited_cell_path_reservation_point_distance = 0
        self.visited_cell_path_end_of_agent_distance = 0

        # current simulation data
        self.current_velocity_reservation_point: float = 0.0
        self.current_velocity_agent: float = 0.0
        self.current_acceleration_agent: float = 0.0
        self.current_max_velocity: float = 0.0
        self.current_distance_reservation_point: float = 0.0
        self.current_distance_agent: float = 0.0
        self.current_tractive_effort: float = 0.0

        # signal to enforce immediate braking
        self.hard_brake = False

        # simulation data storage (history)
        self.distance_reservation_point_simulation_data = []
        self.distance_agent_tp_simulation_data = []
        self.tractive_effort_agent_tp_simulation_data = []
        self.velocity_agent_tp_simulation_data = []
        self.max_velocity_agent_tp_simulation_data = []
        self.acceleration_agent_tp_simulation_data = []
        self.hard_brake_data = []
        self.is_malfunction_state_data = []

        self._removed_from_board = False

        # debug plot
        self._enabled_tractive_effort_rendering = False

    def get_max_agent_velocity(self):
        return min_cached(self.rolling_stock.max_velocity, self.v_max_simulation)

    def set_infrastructure_data(self, infrastructure_data: InfrastructureData):
        reset_infrastructure_data_lru_cache()
        self._infrastructure_data = infrastructure_data

    def remove_agent_from_board(self):
        self._removed_from_board = True

    def is_removed_from_board(self):
        return self._removed_from_board

    def get_allocated_resource(self) -> List[Tuple[int, int]]:
        if self.is_removed_from_board():
            return []
        if len(self.visited_cell_path) <= self.visited_cell_path_end_of_agent_index:
            return self.visited_cell_path
        if len(self.visited_cell_path) < self.visited_cell_path_reservation_point_index:
            if self.position is None:
                return []
            return [self.position]
        if self.visited_cell_path_end_of_agent_index == self.visited_cell_path_reservation_point_index:
            return [self.visited_cell_path[self.visited_cell_path_end_of_agent_index]]
        return self.visited_cell_path[
               self.visited_cell_path_end_of_agent_index:self.visited_cell_path_reservation_point_index]

    def get_allocated_train_point_resource(self) -> Union[Tuple[int, int], None]:
        if self.is_removed_from_board():
            return None
        if len(self.visited_cell_path) <= self.visited_cell_path_end_of_agent_index:
            return self.position
        return self.visited_cell_path[self.visited_cell_path_end_of_agent_index]

    def get_allocated_reservation_point_resource(self) -> Union[Tuple[int, int], None]:
        if self.is_removed_from_board():
            return None
        if len(self.visited_cell_path) == 0:
            return None
        return self.visited_cell_path[len(self.visited_cell_path) - 1]

    @staticmethod
    @enable_infrastructure_data_lru_cache(maxsize=1_000_000)
    def get_cached_dynamics_resource_data(res: Tuple[int, int],
                                          infrastructure_data: InfrastructureData) -> DynamicsResourceData:
        return DynamicsResourceData(res, infrastructure_data)

    @staticmethod
    @lru_cache(maxsize=4096)
    def get_cached_accelerations_and_tractive_effort(obj: RollingStock,
                                                     current_velocity,
                                                     max_allowed_velocity,
                                                     current_gradient,
                                                     train_total_mass,
                                                     simulation_time_step):
        return obj.get_accelerations_and_tractive_effort(
            current_velocity,
            max_allowed_velocity,
            current_gradient,
            train_total_mass,
            simulation_time_step)

    def update_movement_dynamics(self):
        if self.position is None:
            return True
        time_step = 1.0

        velocity_reservation_point = self.current_velocity_reservation_point
        velocity_agent_tp = self.current_velocity_agent

        edge_train_point = \
            DynamicAgent.get_cached_dynamics_resource_data(
                self.get_allocated_train_point_resource(),
                self._infrastructure_data)
        edge_reservation_point = \
            DynamicAgent.get_cached_dynamics_resource_data(
                self.get_allocated_reservation_point_resource(),
                self._infrastructure_data)

        self.current_max_velocity = edge_train_point.max_velocity

        max_velocity = min_cached(
            min_cached(edge_train_point.max_velocity, edge_reservation_point.max_velocity),
            self.get_max_agent_velocity())

        pos_on_edge = self.visited_cell_path_end_of_agent_distance - self.current_distance_agent
        distance_between_cs_rp_cs_tp = max_cached(0.0, edge_train_point.distance - pos_on_edge)
        allocated_resources_list = self.get_allocated_resource()
        intern_max_velocity = min_cached(edge_train_point.max_velocity, self.get_max_agent_velocity())
        distance_update_allowed = True

        # ---------------------------------------------------------------------------------------------------
        # TODO: gradient : weighted sum over train length
        # https://github.com/aiAdrian/flatland_railway_extension/issues/24
        mean_gradient = 0
        # ---------------------------------------------------------------------------------------------------

        for i_res, res in enumerate(allocated_resources_list):
            edge = DynamicAgent.get_cached_dynamics_resource_data(res, self._infrastructure_data)
            intern_max_velocity = min_cached(intern_max_velocity, edge.max_velocity)
            if velocity_agent_tp > intern_max_velocity:
                distance_update_allowed = False
            if distance_update_allowed and i_res > 0:
                distance_between_cs_rp_cs_tp += edge.distance

        max_velocity = min_cached(max_velocity, intern_max_velocity)

        # get gradient (orientation)
        current_tp_gradient = mean_gradient
        if edge_train_point.backward:
            current_tp_gradient = - mean_gradient

        acceleration_train_point, max_braking_acceleration, current_tractive_effort = \
            DynamicAgent.get_cached_accelerations_and_tractive_effort(
                self.rolling_stock,
                velocity_agent_tp,
                max_velocity,
                current_tp_gradient,
                self.mass,
                time_step)

        acceleration_reservation_point = max_cached(0.0, acceleration_train_point)
        acceleration_reservation_point = acceleration_reservation_point + \
                                         acceleration_reservation_point * acceleration_reservation_point / \
                                         abs(max_braking_acceleration)

        self.current_tractive_effort = current_tractive_effort

        # --------------------------------------------------------------------------------------------------------
        # Braking has to be decoupled from this code (do a refactoring) -> method/object which allows to
        # change braking strategy (behaviour)
        # --------------------------------------------------------------------------------------------------------
        # https://github.com/aiAdrian/flatland_railway_extension/issues/23

        # braking: yes - but ....
        do_brake = velocity_agent_tp > max_velocity  # i.e. brake - if and only if coasting is not enough
        # (vTP + aTP * timeStep)

        # coasting ?
        coasting = True
        if coasting:
            if do_brake and self.current_acceleration_agent >= 0:
                delta_braking_distance = 0.5 * (velocity_agent_tp * velocity_agent_tp - max_velocity * max_velocity) \
                                         / abs(max_braking_acceleration) + self.length
                if (distance_between_cs_rp_cs_tp - delta_braking_distance) > (
                        edge_train_point.max_velocity * time_step):
                    do_brake = False
                    max_velocity = velocity_agent_tp

        # --------------------------------------------------------------------------------------------------------

        # overwrite max_velocity if hard_brake is set
        if self.hard_brake:
            max_velocity = 0.0

        # check what the train driver has to do
        if do_brake or self.hard_brake:
            acceleration_train_point = max_braking_acceleration
            acceleration = (velocity_agent_tp - max_velocity) / time_step
            if acceleration < abs(acceleration_train_point):
                acceleration_train_point = -acceleration

            if self.current_acceleration_agent >= 0:
                velocity_agent_tp += acceleration_train_point * time_step

            acceleration_reservation_point = 0.0
            velocity_reservation_point = 0.0

        if velocity_agent_tp < max_velocity:
            # accelerate
            if velocity_reservation_point < velocity_agent_tp:
                velocity_reservation_point = velocity_agent_tp

        if velocity_agent_tp == max_velocity:
            # hold velocity
            velocity_reservation_point = velocity_agent_tp
            acceleration_train_point = 0.0
            acceleration_reservation_point = 0.0

        # avoid backwards
        if velocity_agent_tp < 0.0:
            acceleration_train_point = 0.0
            velocity_agent_tp = 0.0
            acceleration_reservation_point = 0.0
            velocity_reservation_point = 0.0

        # euler step: train point
        delta_pos_tp = velocity_agent_tp * time_step
        self.current_distance_agent += delta_pos_tp
        self.current_velocity_agent = velocity_agent_tp + acceleration_train_point * time_step
        self.current_acceleration_agent = acceleration_train_point

        # euler step: reservation point
        # Correction of position from braking distance and current position. Note the position varies with the
        # Behavior of the train, reservation point position air return, in case of full braking.
        current_braking_distance = 0.5 * (
                self.current_velocity_agent * self.current_velocity_agent) / abs(max_braking_acceleration) + self.length
        delta_pos_rp = max_cached(
            0.0, (self.current_distance_agent + current_braking_distance) - self.current_distance_reservation_point)
        self.current_distance_reservation_point += delta_pos_rp
        self.current_velocity_reservation_point = velocity_reservation_point + \
                                                  acceleration_reservation_point * time_step

        # check and allow reservation point move forward
        move_reservation_point = \
            self.current_distance_reservation_point > self.visited_cell_path_reservation_point_distance

        return move_reservation_point

    def update_agent_positions(self):
        self._max_episode_steps = 10000
        pos = self.position
        if pos is not None:
            # The resource is only added if it was not the last one added - i.e. the agent switched to a new one.
            allocated_resource = self.get_allocated_resource()
            if len(allocated_resource) > 0:
                allocated_resource = [allocated_resource[len(allocated_resource) - 1]]
            if pos not in allocated_resource:
                self.visited_cell_path.append(self.position)
                self.visited_direction_path.append((self.direction, self.old_direction))
                cell_data = DynamicAgent.get_cached_dynamics_resource_data(pos, self._infrastructure_data)
                self.visited_cell_path_reservation_point_distance += cell_data.distance
                self.visited_cell_distance.append(self.visited_cell_path_reservation_point_distance)
                self.visited_cell_path_reservation_point_index = len(self.visited_cell_path)

            # update current position of the agent's end with respect to visited cells
            self.visited_cell_path_end_of_agent_index = DynamicAgent.get_first_element_index_greater_than(
                input_list=self.visited_cell_distance,
                cmp_value=self.current_distance_agent,
                start_index=self.visited_cell_path_end_of_agent_index
            )

            if self.visited_cell_path_end_of_agent_index >= self.visited_cell_path_reservation_point_index:
                self.visited_cell_path_end_of_agent_index = self.visited_cell_path_reservation_point_index - 1

            # update current position of the agent' start with respect to visited cells
            self.visited_cell_path_start_of_agent_index = DynamicAgent.get_first_element_index_greater_than(
                input_list=self.visited_cell_distance,
                cmp_value=self.current_distance_agent + self.length,
                start_index=self.visited_cell_path_start_of_agent_index
            )

            if self.visited_cell_path_start_of_agent_index >= self.visited_cell_path_reservation_point_index:
                self.visited_cell_path_start_of_agent_index = self.visited_cell_path_reservation_point_index - 1

            # update positions and distances
            self.visited_cell_path_end_of_agent_distance = \
                self.visited_cell_distance[self.visited_cell_path_end_of_agent_index]
            self.distance_reservation_point_simulation_data.append(self.current_distance_reservation_point)
            self.distance_agent_tp_simulation_data.append(self.current_distance_agent)
            self.acceleration_agent_tp_simulation_data.append(self.current_acceleration_agent)
            self.velocity_agent_tp_simulation_data.append(self.current_velocity_agent)
            self.max_velocity_agent_tp_simulation_data.append(self.current_max_velocity)
            self.tractive_effort_agent_tp_simulation_data.append(self.current_tractive_effort)
            self.hard_brake_data.append(self.hard_brake)
            self.is_malfunction_state_data.append(self.state.is_malfunction_state())
        else:
            self.set_hard_brake(True)

    def set_length(self, length: float):
        '''
        Sets the physical total length in meter
        :param length: in meter
        '''
        self.length: float = length

    def set_mass(self, mass: float):
        '''
        Sets the physical total mass in tonne
        :param mass:
        '''
        self.mass: float = mass

    def set_rolling_stock(self, rolling_stock: RollingStock):
        '''
        Sets the rolling stock information / traction data
        :param rolling_stock: a reference to the rolling stock object
        '''
        self.rolling_stock = rolling_stock

    def all_resource_ok(self, resource_allocation_ok):
        self.set_hard_brake(not resource_allocation_ok)

    def update_agent(self):
        self.update_agent_positions()

    def set_hard_brake(self, hard_brake):
        self.hard_brake = hard_brake

    def reset(self):
        super(DynamicAgent, self).reset()

    def set_tractive_effort_rendering(self, enable=True):
        self._enabled_tractive_effort_rendering = enable

    @staticmethod
    def get_first_element_index_greater_than(input_list, cmp_value, start_index=0):
        '''
        This methods calculates the same as ...

        indices = np.argwhere(np.array(input_list) > cmp_value)
        if len(indices) == 0:
            if len(self.visited_cell_distance) == 0:
                return 0
        else:
            return indices[0][0]

        ... but 20x faster for small input_lists
        '''
        idx = start_index
        while idx < (len(input_list) - 1):
            val = input_list[idx]
            if val > cmp_value:
                return idx
            idx += 1
        return idx

    def do_debug_plot(self, idx=1, nbr_agents=1, show=True, show_title=True):
        plt.rc('font', size=6)

        nbr_features = 2
        if self._enabled_tractive_effort_rendering:
            nbr_features = 4

        mal_func_signal = np.array(self.is_malfunction_state_data[1:]).copy().astype(float)
        mal_func_signal[mal_func_signal == 0] = np.nan

        ax1 = plt.subplot(nbr_agents, nbr_features, 1 + (idx - 1) * nbr_features)
        plt.plot(self.distance_agent_tp_simulation_data[1:], np.array(self.velocity_agent_tp_simulation_data[1:]) * 3.6)
        plt.plot(
            np.array(self.distance_agent_tp_simulation_data[1:]) - self.length,
            np.array(self.max_velocity_agent_tp_simulation_data[1:]) * 3.6)
        plt.plot(
            np.array(self.distance_agent_tp_simulation_data[1:]),
            (np.array(self.velocity_agent_tp_simulation_data[1:]) * 3.6) * mal_func_signal, 'r')
        if show_title:
            ax1.set_title('Distance vs. velocity', fontsize=10)

        ax2 = plt.subplot(nbr_agents, nbr_features, 2 + (idx - 1) * nbr_features)
        plt.plot(self.distance_agent_tp_simulation_data[1:], self.acceleration_agent_tp_simulation_data[1:])
        plt.plot(
            self.distance_agent_tp_simulation_data[1:],
            np.array(self.acceleration_agent_tp_simulation_data[1:]) * mal_func_signal, 'r')
        if show_title:
            ax2.set_title('Distance vs. acceleration', fontsize=10)

        if self._enabled_tractive_effort_rendering:
            ax3 = plt.subplot(nbr_agents, nbr_features, 3 + (idx - 1) * nbr_features)
            plt.plot(self.distance_agent_tp_simulation_data[1:], self.hard_brake_data[1:])
            if show_title:
                ax3.set_title('Distance vs. hard_brake', fontsize=10)

        if self._enabled_tractive_effort_rendering:
            ax4 = plt.subplot(nbr_agents, nbr_features, 4 + (idx - 1) * nbr_features)
            plt.plot(
                np.array(self.velocity_agent_tp_simulation_data[1:]) * 3.6,
                np.array(self.tractive_effort_agent_tp_simulation_data[1:]) / 1000.0, 'b.')
            if show_title:
                ax4.set_title('Velocity vs. tractive effort', fontsize=10)
            ax4.set_xlim([0, self.rolling_stock.max_velocity * 3.6 + 10])
            ax4.set_ylim([0, self.rolling_stock.max_traction / 1000 + 10])

        if show:
            plt.show()
