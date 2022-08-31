from typing import Tuple, List, Union

import numpy as np
from flatland.envs.agent_utils import EnvAgent
from matplotlib import pyplot as plt


class RollingStock:
    def __init__(self,
                 max_traction=210000.0,
                 v_max_traction=50.0,
                 a_max_acceleration=10.0,
                 a_max_break=-0.1,
                 mass_factor=1.05,
                 k=0.5,
                 c=2.5):
        self.maxTraction: float = max_traction
        self.vMaxTraction: float = v_max_traction
        self.set_a_max_acceleration(a_max_acceleration)
        self.set_a_max_break(a_max_break)
        self.massFactor: float = mass_factor
        self.K: float = k
        self.C: float = c

    def set_a_max_acceleration(self, a_max_acceleration):
        self.a_max_acceleration = a_max_acceleration

    def set_a_max_break(self, a_max_break):
        self.a_max_break = a_max_break


class InfrastructureData:
    def __init__(self):
        # infrastructure
        self._infrastructure_max_velocity_grid: Union[np.array, None] = None
        self._infrastructure_cell_length_grid: Union[np.array, None] = None

    def set_infrastructure_max_velocity_grid(self, infrastructure_max_velocity_grid: np.array):
        self._infrastructure_max_velocity_grid = infrastructure_max_velocity_grid

    def set_infrastructure_cell_length_grid(self, infrastructure_cell_length_grid: np.array):
        self._infrastructure_cell_length_grid = infrastructure_cell_length_grid

    def get_velocity(self, res: Tuple[int, int]):
        if res is None:
            return 200 / 3.6
        if self._infrastructure_max_velocity_grid is not None:
            return self._infrastructure_max_velocity_grid[res[0], res[1]]
        return 200 / 3.6

    def get_cell_length(self, res: Tuple[int, int]):
        if res is None:
            return 400
        if self._infrastructure_cell_length_grid is not None:
            return self._infrastructure_cell_length_grid[res[0], res[1]]
        return 400


class Edge:
    def __init__(self, res: Tuple[int, int], infrastructure_data: InfrastructureData):
        self.gradient: float = 0.0
        self.distance: float = 400
        self.vMax: float = 200 / 3.6
        self.backward = False
        self.rndBreakFactor = 1.0

        if infrastructure_data is not None:
            self.distance = infrastructure_data.get_cell_length(res)
            self.vMax = infrastructure_data.get_velocity(res)


class XDynamicAgent(EnvAgent):
    def __init__(self, original_env_agent: EnvAgent):
        super(XDynamicAgent, self).__init__(original_env_agent.initial_position,
                                            original_env_agent.initial_direction, original_env_agent.direction,
                                            original_env_agent.target)

        # copy all attributes data from original EnvAgent allocated in RailEnv
        self._copy_attribute_from_env_agent(original_env_agent)

        # set the extended dynamic agent attributes
        self.set_length(400)
        self.set_mass(1000)
        self.set_rolling_stock(RollingStock())

        # infrastructure
        self.v_max_simulation = 300 / 3.6
        self._infrastructure_data: Union[InfrastructureData, None] = None

        # set the internal simulation data (resource allocation, velocity, ... )
        self.train_run_path: List[Tuple[int, int]] = []
        self.train_run_distance: List[float] = []
        self.currentSection_ReservationPoint = 0
        self.currentSection_Train = 0

        self.currentVelocity_ReservationPoint: float = 0.0
        self.currentDistance_ReservationPoint: float = 0.0
        self.currentVelocity_Train: float = 0.0
        self.currentAcceleration_Train: float = 0.0
        self.currentDistance_Train: float = 0.0
        self.current_max_velocity: float = 0.0

        self.hard_brake = False
        self.move_reservation_point = True

        self.distance_RP = []
        self.distance_TP = []
        self.velocity_TP = []
        self.max_velocity_TP = []
        self.a_TP = []

    def set_infrastructure_data(self, infrastructure_data: InfrastructureData):
        self._infrastructure_data = infrastructure_data

    def get_allocated_resource(self) -> List[Tuple[int, int]]:
        if len(self.train_run_path) <= self.currentSection_Train:
            return self.train_run_path
        if len(self.train_run_path) < self.currentSection_ReservationPoint:
            return []
        if self.currentSection_Train == self.currentSection_ReservationPoint:
            return [self.train_run_path[self.currentSection_Train]]
        return self.train_run_path[self.currentSection_Train:self.currentSection_ReservationPoint]

    def get_allocated_train_point_resource(self) -> Union[Tuple[int, int], None]:
        if len(self.train_run_path) <= self.currentSection_Train:
            return self.position
        return self.train_run_path[self.currentSection_Train]

    def get_allocated_reservation_point_resource(self) -> Union[Tuple[int, int], None]:
        if len(self.train_run_path) == 0:
            return self.position
        return self.position  # self.train_run_path[len(self.train_run_path) - 1]

    def _update_movement_dynamics(self):
        timeStep = 1.0

        vRP = self.currentVelocity_ReservationPoint
        vTP = self.currentVelocity_Train

        aMax_acceleration = self.rolling_stock.a_max_acceleration
        aMax_break = self.rolling_stock.a_max_break

        vMax = self.v_max_simulation

        # calculate the minimal vMax between TrainPoint(TP) and ReservationPoint(RP) and estimate as well
        # the distance with const current velocity
        csRP = self.currentSection_ReservationPoint
        csTP = self.currentSection_Train

        edgeTP = Edge(self.get_allocated_train_point_resource(), self._infrastructure_data)
        edgeRP = Edge(self.get_allocated_reservation_point_resource(), self._infrastructure_data)

        vMax = np.min(np.array([edgeTP.vMax, edgeRP.vMax, vMax, vMax]))

        # gradient : weighted sum over train length
        gradientTrasseLen = max(self.length, float(1.0))
        gradientSplitLen = min(gradientTrasseLen, max(float(0.0), edgeTP.distance - self.currentDistance_Train))
        meanGradient = (edgeTP.gradient * gradientSplitLen + edgeRP.gradient * (
                gradientTrasseLen - gradientSplitLen)) / gradientTrasseLen

        distanceBetween_csRP_csTP = max(float(0.0), edgeTP.distance - self.currentDistance_Train)
        if (csRP - csTP) > 1:
            distanceUpdateAllowed = True
            internVMax: float = edgeTP.vMax
            trasseDistanceGradient: float = 0
            if (csTP < csRP):
                meanGradient = 0.0

            for k, res in enumerate(self.get_allocated_resource()):
                internVMax = min(self.rolling_stock.vMaxTraction, internVMax)
                ts = Edge(res, self._infrastructure_data)
                internVMax = min(ts.vMax, internVMax)
                if vTP > internVMax:
                    distanceUpdateAllowed = False

                if distanceUpdateAllowed and (k > csTP):
                    distanceBetween_csRP_csTP += ts.distance

                aMax_break = min(aMax_break, self.rolling_stock.a_max_break * ts.rndBreakFactor)
                aMax_acceleration = min(aMax_acceleration, self.rolling_stock.a_max_acceleration * ts.rndBreakFactor)
            vMax = min(internVMax, vMax)
            if trasseDistanceGradient > 0:
                meanGradient /= trasseDistanceGradient

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
        if doBreak and self.currentAcceleration_Train >= 0:
            deltaBremsweg = 0.5 * (vTP * vTP - vMax * vMax) / abs(aMax_break) + self.length
            # float restDistanz = (edgeTP->distance - trasseSecTP->currentDistance_Train);
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

            if self.currentAcceleration_Train >= 0:
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

        self.current_max_velocity = edgeTP.vMax

        # euler step: train point
        delta_pos_tp = vTP * timeStep
        self.currentDistance_Train += delta_pos_tp
        self.currentVelocity_Train = vTP + aTP * timeStep
        self.currentAcceleration_Train = aTP

        # euler step: reservation point
        #   Korrektur des Position aus Bremsweg und aktueller Position. Beachte die Position variiert mit dem
        # 	Verhalten des Zuges, Reservationspunkt Position luft retour, beim Vollbremung.
        bremsweg = 0.5 * (self.currentVelocity_Train * self.currentVelocity_Train) / abs(aMax_break) + self.length
        delta_pos_rp = (self.currentDistance_Train + bremsweg - self.currentDistance_ReservationPoint)
        self.currentDistance_ReservationPoint += delta_pos_rp
        self.currentVelocity_ReservationPoint = vRP + aRP * timeStep

        return delta_pos_tp, delta_pos_rp

    def update_dynamics(self):
        self._max_episode_steps = 10000
        pos = self.position
        if pos is not None:
            if pos not in self.train_run_path:
                self.train_run_path.append(self.position)
                self.currentSection_ReservationPoint = len(self.train_run_path)

            delta_pos_tp, delta_pos_rp = self._update_movement_dynamics()
            if delta_pos_rp == 0:
                self.move_reservation_point = False
            else:
                self.move_reservation_point = self.currentDistance_ReservationPoint >= len(self.train_run_path) * 400
            self.currentSection_Train = min(len(self.train_run_path) - 1,
                                            int(np.floor(self.currentDistance_Train / 400)))

            self.distance_RP.append(self.currentDistance_ReservationPoint)
            self.distance_TP.append(self.currentDistance_Train)
            self.a_TP.append(self.currentAcceleration_Train)
            self.velocity_TP.append(self.currentVelocity_Train)
            self.max_velocity_TP.append(self.current_max_velocity)

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

    def _copy_attribute_from_env_agent(self, env_agent: EnvAgent):
        '''
        Copy all class attribute and it's value from EnvAgent to XDynamicAgent
        :param env_agent: The original agent created in the RailEnv
        '''
        for attribute, value in env_agent.__dict__.items():
            setattr(self, attribute, value)

    def set_hard_brake(self, hard_brake):
        self.hard_brake = hard_brake

    def reset(self):
        super(XDynamicAgent, self).reset()

    def do_debug_plot(self):
        plt.rc('font', size=12)
        ax1 = plt.subplot(3, 1, 1)
        plt.plot(self.distance_TP[1:], np.array(self.velocity_TP[1:]) * 3.6)
        plt.plot(self.distance_TP[1:], np.array(self.max_velocity_TP[1:]) * 3.6)
        ax1.set_title('Distance vs. velocity', fontsize=10)

        ax2 = plt.subplot(3, 1, 2)
        plt.plot(self.distance_TP[1:], np.array(self.distance_RP[1:]) - np.array(self.distance_TP[1:]))
        ax2.set_title('Distance vs. braking distance + train length', fontsize=10)

        ax3 = plt.subplot(3, 1, 3)
        plt.plot(self.distance_TP[1:], self.a_TP[1:])
        ax3.set_title('Distance vs. Acceleration', fontsize=10)

        plt.show()
