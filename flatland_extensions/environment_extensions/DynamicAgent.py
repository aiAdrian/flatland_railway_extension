# Idea: Adrian Egli / Erik Nygren
# Permission to use - If you use this or any idea out of this code for a
# publication, you must credit the authors. No commerical
# use allowed.
from typing import Tuple, List, Union

import numpy as np
from flatland.envs.agent_utils import EnvAgent
from matplotlib import pyplot as plt

from flatland_extensions.environment_extensions.DynamicsResourceData import DynamicsResourceData
from flatland_extensions.environment_extensions.InfrastructureData import InfrastructureData
from flatland_extensions.environment_extensions.RollingStock import RollingStock
from flatland_extensions.environment_extensions.XAgent import XAgent


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
        self.visited_cell_path: List[Tuple[int, int]] = []
        self.visited_direction_path: List[Tuple[int, int]] = []
        self.visited_cell_distance: List[float] = []
        self.visited_cell_path_reservation_point_index = 0
        self.visited_cell_path_end_of_agent_index = 0
        self.visited_cell_path_start_of_agent_index = 0
        self.visited_cell_path_reservation_point_distance = 0
        self.visited_cell_path_end_of_agent_distance = 0

        self.current_velocity_reservation_point: float = 0.0
        self.current_velocity_agent: float = 0.0
        self.current_acceleration_agent: float = 0.0
        self.current_max_velocity: float = 0.0
        self.current_distance_reservation_point: float = 0.0
        self.current_distance_agent: float = 0.0

        self.hard_brake = False

        self.distance_reservation_point_simulation_data = []
        self.distance_agent_tp_simulation_data = []
        self.velocity_agent_tp_simulation_data = []
        self.max_velocity_agent_tp_simulation_data = []
        self.acceleration_agent_tp_simulation_data = []

    def set_infrastructure_data(self, infrastructure_data: InfrastructureData):
        self._infrastructure_data = infrastructure_data

    def get_allocated_resource(self) -> List[Tuple[int, int]]:
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
        if len(self.visited_cell_path) <= self.visited_cell_path_end_of_agent_index:
            return self.position
        return self.visited_cell_path[self.visited_cell_path_end_of_agent_index]

    def get_allocated_reservation_point_resource(self) -> Union[Tuple[int, int], None]:
        if len(self.visited_cell_path) == 0:
            return None
        return self.visited_cell_path[len(self.visited_cell_path) - 1]

    def update_movement_dynamics(self):
        if self.position is None:
            return True
        time_step = 1.0

        velocity_reservation_point = self.current_velocity_reservation_point
        velocity_agent_tp = self.current_velocity_agent

        a_max_acceleration = self.rolling_stock.a_max_acceleration
        a_max_braking = self.rolling_stock.a_max_braking

        edge_train_point = \
            DynamicsResourceData(self.get_allocated_train_point_resource(), self._infrastructure_data)
        edge_reservation_point = \
            DynamicsResourceData(self.get_allocated_reservation_point_resource(), self._infrastructure_data)

        self.current_max_velocity = edge_train_point.max_velocity

        velocity_max_array = np.array([edge_train_point.max_velocity, edge_reservation_point.max_velocity,
                                       self.rolling_stock.velocity_max_traction, self.v_max_simulation])
        max_velocity = np.min(velocity_max_array)

        pos_on_edge = self.visited_cell_path_end_of_agent_distance - self.current_distance_agent
        distance_between_cs_rp_cs_tp = max(0.0, edge_train_point.distance - pos_on_edge)
        allocated_resources_list = self.get_allocated_resource()
        intern_max_velocity_array = np.array([edge_train_point.max_velocity, self.rolling_stock.velocity_max_traction,
                                              self.v_max_simulation])
        intern_max_velocity = np.min(intern_max_velocity_array)
        distance_update_allowed = True

        # ---------------------------------------------------------------------------------------------------
        # TODO: gradient : weighted sum over train length
        # https://github.com/aiAdrian/flatland_railway_extension/issues/24
        mean_gradient = 0
        # ---------------------------------------------------------------------------------------------------

        for i_res, res in enumerate(allocated_resources_list):
            edge = DynamicsResourceData(res, self._infrastructure_data)
            intern_max_velocity = min(intern_max_velocity, edge.max_velocity)
            if velocity_agent_tp > intern_max_velocity:
                distance_update_allowed = False
            if distance_update_allowed and i_res > 0:
                distance_between_cs_rp_cs_tp += edge.distance

        max_velocity = min(max_velocity, intern_max_velocity)

        # run resistances
        train_run_resistance = self.rolling_stock.C + self.rolling_stock.K * velocity_agent_tp * velocity_agent_tp * 0.01296

        # slop gradient
        if edge_train_point.backward:
            train_run_resistance = train_run_resistance - mean_gradient
        else:
            train_run_resistance = train_run_resistance + mean_gradient

        # accelerate
        acceleration_train_point = a_max_acceleration
        acceleration = max(float(0.0), max_velocity - velocity_agent_tp) / time_step
        if acceleration < acceleration_train_point:
            acceleration_train_point = acceleration

        # resistance
        total_resistance = train_run_resistance

        if acceleration_train_point > 0.0:
            total_resistance = total_resistance + self.rolling_stock.mass_factor * acceleration_train_point * 100.0

        # traction
        total_resistance = total_resistance * self.mass * 9.81
        max_traction_train_point = self.rolling_stock.max_traction
        if velocity_agent_tp > self.rolling_stock.velocity_max_traction:
            max_traction_train_point = \
                self.rolling_stock.max_traction * self.rolling_stock.velocity_max_traction / velocity_agent_tp

        if total_resistance < max_traction_train_point:
            max_traction_train_point = total_resistance

        acceleration_train_point = \
            (max_traction_train_point / self.mass - train_run_resistance * 9.81) * ( \
                        0.001 / self.rolling_stock.mass_factor)
        if acceleration_train_point < 0.0:
            a_max_braking = a_max_braking + acceleration_train_point
        acceleration_reservation_point = max(0.0, acceleration_train_point)
        acceleration_reservation_point = acceleration_reservation_point + \
                                         acceleration_reservation_point * acceleration_reservation_point / \
                                         abs(a_max_braking)

        # --------------------------------------------------------------------------------------------------------
        # Braking has to be decoupled from this code (do a refactoring) -> method/object which allows to
        # change braking strategy (behaviour)
        # --------------------------------------------------------------------------------------------------------
        # https://github.com/aiAdrian/flatland_railway_extension/issues/23

        # braking: yes - but ....
        do_brake = velocity_agent_tp > max_velocity  # i.e. brake - if and only if coasting is not enough (vTP + aTP * timeStep)

        # coasting ?
        coasting = True
        if coasting:
            if do_brake and self.current_acceleration_agent >= 0:
                delta_braking_distance = 0.5 * (velocity_agent_tp * velocity_agent_tp - max_velocity * max_velocity) \
                                         / abs(a_max_braking) + self.length
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
            acceleration_train_point = a_max_braking
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
        bremsweg = 0.5 * (self.current_velocity_agent * self.current_velocity_agent) / abs(a_max_braking) + self.length
        delta_pos_rp = max(0.0, (self.current_distance_agent + bremsweg) - self.current_distance_reservation_point)
        self.current_distance_reservation_point += delta_pos_rp
        self.current_velocity_reservation_point = velocity_reservation_point + acceleration_reservation_point * time_step

        # check and allow reservation point move forward
        move_reservation_point = \
            self.current_distance_reservation_point > self.visited_cell_path_reservation_point_distance

        return move_reservation_point

    def update_agent_positions(self):
        self._max_episode_steps = 10000
        pos = self.position
        if pos is not None:
            if pos not in self.visited_cell_path:
                self.visited_cell_path.append(self.position)
                self.visited_direction_path.append((self.direction, self.old_direction))
                cell_data = DynamicsResourceData(pos, self._infrastructure_data)
                self.visited_cell_path_reservation_point_distance += cell_data.distance
                self.visited_cell_distance.append(self.visited_cell_path_reservation_point_distance)
                self.visited_cell_path_reservation_point_index = len(self.visited_cell_path)

            # update current position of the agent's end with respect to visited cells
            end_of_agent_idx = np.argwhere(np.array(self.visited_cell_distance) > self.current_distance_agent)
            if len(end_of_agent_idx) == 0:
                if len(self.visited_cell_distance) == 0:
                    self.visited_cell_path_end_of_agent_index = 0
            else:
                self.visited_cell_path_end_of_agent_index = end_of_agent_idx[0][0]
            if self.visited_cell_path_end_of_agent_index >= self.visited_cell_path_reservation_point_index:
                self.visited_cell_path_end_of_agent_index = self.visited_cell_path_reservation_point_index - 1

            # update current position of the agent' start with respect to visited cells
            start_of_agent_idx = np.argwhere(np.array(self.visited_cell_distance) >
                                             self.current_distance_agent + self.length)
            if len(start_of_agent_idx) == 0:
                if len(self.visited_cell_distance) == 0:
                    self.visited_cell_path_start_of_agent_index = 0
            else:
                self.visited_cell_path_start_of_agent_index = start_of_agent_idx[0][0]
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

    def do_debug_plot(self, idx=1, nbr_agents=1, show=True, show_title=True):
        plt.rc('font', size=6)

        ax1 = plt.subplot(nbr_agents, 2, 1 + (idx - 1) * 2)
        plt.plot(self.distance_agent_tp_simulation_data[1:], np.array(self.velocity_agent_tp_simulation_data[1:]) * 3.6)
        plt.plot(np.array(self.distance_agent_tp_simulation_data[1:]) - self.length,
                 np.array(self.max_velocity_agent_tp_simulation_data[1:]) * 3.6)
        if show_title:
            ax1.set_title('Distance vs. velocity', fontsize=10)

        ax2 = plt.subplot(nbr_agents, 2, 2 + (idx - 1) * 2)
        plt.plot(self.distance_agent_tp_simulation_data[1:], self.acceleration_agent_tp_simulation_data[1:])
        if show_title:
            ax2.set_title('Distance vs. Acceleration', fontsize=10)
        if show:
            plt.show()
