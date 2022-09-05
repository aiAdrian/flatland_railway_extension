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
        self.visited_cell_distance: List[float] = []
        self.visited_cell_path_reservation_point_index = 0
        self.visited_cell_path_end_of_agent_index = 0
        self.visited_cell_path_reservation_point_distance = 0
        self.visited_cell_path_end_of_agent_distance = 0

        self.current_velocity_reservation_point: float = 0.0
        self.current_velocity_agent: float = 0.0
        self.current_acceleration_agent: float = 0.0
        self.current_max_velocity: float = 0.0
        self.current_distance_reservation_point: float = 0.0
        self.current_distance_agent: float = 0.0

        self.hard_brake = False

        self.distance_RP = []
        self.distance_TP = []
        self.velocity_TP = []
        self.max_velocity_TP = []
        self.a_TP = []

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
        timeStep = 1.0

        vRP = self.current_velocity_reservation_point
        vTP = self.current_velocity_agent

        aMax_acceleration = self.rolling_stock.a_max_acceleration
        aMax_break = self.rolling_stock.a_max_break

        edgeTP = DynamicsResourceData(self.get_allocated_train_point_resource(), self._infrastructure_data)
        edgeRP = DynamicsResourceData(self.get_allocated_reservation_point_resource(), self._infrastructure_data)

        self.current_max_velocity = edgeTP.vMax

        vMax_array = np.array([edgeTP.vMax, edgeRP.vMax, self.rolling_stock.vMaxTraction, self.v_max_simulation])
        vMax = np.min(vMax_array)

        pos_on_edge = self.current_distance_agent - self.visited_cell_path_end_of_agent_distance
        distanceBetween_csRP_csTP = max(0.0, edgeTP.distance - pos_on_edge)
        allocted_ressources_list = self.get_allocated_resource()
        internVMax = edgeTP.vMax
        distanceUpdateAllowed = True
        # TODO: gradient : weighted sum over train length
        meanGradient = 0
        for i_res, res in enumerate(allocted_ressources_list):
            edge = DynamicsResourceData(res, self._infrastructure_data)
            internVMax = min(internVMax, edge.vMax)
            if vTP > internVMax:
                distanceUpdateAllowed = False
            if distanceUpdateAllowed and i_res > 0:
                distanceBetween_csRP_csTP += edge.distance

        vMax = min(vMax, internVMax)

        # Resistances / Laufwiderstand
        wRun = self.rolling_stock.C + self.rolling_stock.K * vTP * vTP * 0.01296

        # Steigung
        if edgeTP.backward:
            wRun = wRun - meanGradient
        else:
            wRun = wRun + meanGradient

        # Accelerate
        aTP = aMax_acceleration
        a = max(float(0.0), vMax - vTP) / timeStep
        if (a < aTP):
            aTP = a
        #  Resistance
        W = wRun

        if aTP > 0.0:
            W = W + self.rolling_stock.massFactor * aTP * 100.0

        # Traction    needed
        W = W * self.mass * 9.81
        tTP = self.rolling_stock.maxTraction
        if vTP > self.rolling_stock.vMaxTraction:
            tTP = self.rolling_stock.maxTraction * self.rolling_stock.vMaxTraction / vTP

        if W < tTP:
            tTP = W

        aTP = (tTP / self.mass - wRun * 9.81) * 0.001 / self.rolling_stock.massFactor
        if aTP < 0.0:
            aMax_break = aMax_break + aTP
        aRP = max(0.0, aTP)
        aRP = aRP + aRP * aRP / abs(aMax_break)

        # Break
        doBreak = vTP > vMax  # i.e. break - if and only if coasting is not enough (vTP + aTP * timeStep)
        if doBreak and self.current_acceleration_agent >= 0:
            deltaBremsweg = 0.5 * (vTP * vTP - vMax * vMax) / abs(aMax_break) + self.length
            # float restDistanz = (edgeTP->distance - trasseSecTP->current_distance_agent);
            if (distanceBetween_csRP_csTP - deltaBremsweg) > (edgeTP.vMax * timeStep):
                doBreak = False
                vMax = vTP

        # overwrite vMax if hardBreak is set
        if self.hard_brake:
            vMax = 0.0

        # check what the train driver has to do
        if doBreak or self.hard_brake:
            aTP = aMax_break
            a = (vTP - vMax) / timeStep
            if a < abs(aTP):
                aTP = -a

            if self.current_acceleration_agent >= 0:
                vTP += aTP * timeStep

            aRP = 0.0
            vRP = 0.0

        if vTP < vMax:
            # accelerate
            if vRP < vTP:
                vRP = vTP

        if vTP == vMax:
            # hold velocity
            vRP = vTP
            aTP = 0.0
            aRP = 0.0

        # avoid backwards
        if vTP < 0.0:
            aTP = 0.0
            vTP = 0.0
            aRP = 0.0
            vRP = 0.0

        # euler step: train point
        delta_pos_tp = vTP * timeStep
        self.current_distance_agent += delta_pos_tp
        self.current_velocity_agent = vTP + aTP * timeStep
        self.current_acceleration_agent = aTP

        # euler step: reservation point
        #   Korrektur der Position aus Bremsweg und aktueller Position. Beachte die Position variiert mit dem
        # 	Verhalten des Zuges, Reservationspunkt Position luft retour, beim Vollbremung.
        bremsweg = 0.5 * (self.current_velocity_agent * self.current_velocity_agent) / abs(aMax_break) + self.length
        delta_pos_rp = max(0.0, (self.current_distance_agent + bremsweg) - self.current_distance_reservation_point)
        self.current_distance_reservation_point += delta_pos_rp
        self.current_velocity_reservation_point = vRP + aRP * timeStep

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
                cell_data = DynamicsResourceData(pos, self._infrastructure_data)
                self.visited_cell_path_reservation_point_distance += cell_data.distance
                self.visited_cell_distance.append(self.visited_cell_path_reservation_point_distance)
                self.visited_cell_path_reservation_point_index = len(self.visited_cell_path)

            # update current position of the agent with respect to visited cells
            end_of_agent_idx = np.argwhere(np.array(self.visited_cell_distance) > self.current_distance_agent)
            if len(end_of_agent_idx) == 0:
                if len(self.visited_cell_distance) == 0:
                    self.visited_cell_path_end_of_agent_index = 0
            else:
                self.visited_cell_path_end_of_agent_index = end_of_agent_idx[0][0]
            if self.visited_cell_path_end_of_agent_index >= self.visited_cell_path_reservation_point_index:
                self.visited_cell_path_end_of_agent_index = self.visited_cell_path_reservation_point_index - 1
            self.visited_cell_path_end_of_agent_distance = \
                self.visited_cell_distance[self.visited_cell_path_end_of_agent_index]
            self.distance_RP.append(self.current_distance_reservation_point)
            self.distance_TP.append(self.current_distance_agent)
            self.a_TP.append(self.current_acceleration_agent)
            self.velocity_TP.append(self.current_velocity_agent)
            self.max_velocity_TP.append(self.current_max_velocity)
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
        plt.rc('font', size=12)

        ax1 = plt.subplot(nbr_agents, 2, 1 + (idx - 1) * 2)
        plt.plot(self.distance_TP[1:], np.array(self.velocity_TP[1:]) * 3.6)
        plt.plot(np.array(self.distance_TP[1:]) - self.length, np.array(self.max_velocity_TP[1:]) * 3.6)
        if show_title:
            ax1.set_title('Distance vs. velocity', fontsize=10)

        ax2 = plt.subplot(nbr_agents, 2, 2 + (idx - 1) * 2)
        plt.plot(self.distance_TP[1:], self.a_TP[1:])
        if show_title:
            ax2.set_title('Distance vs. Acceleration', fontsize=10)
        if show:
            plt.show()
