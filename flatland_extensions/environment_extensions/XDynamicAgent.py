from typing import Tuple, List

from flatland.envs.agent_utils import EnvAgent


class RollingStock:
    def __init__(self,
                 max_traction=210000.0,
                 v_max_traction=50.0,
                 a_max_acceleration=1.0,
                 a_max_break=-1.0,
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


class XDynamicAgent(EnvAgent):
    def __init__(self, original_env_agent: EnvAgent):
        super(XDynamicAgent, self).__init__(original_env_agent.initial_position,
                                            original_env_agent.initial_direction, original_env_agent.direction,
                                            original_env_agent.target)
        self._copy_original_env_agent(original_env_agent)
        self.set_length(400)
        self.set_mass(1000)
        self.set_rolling_stock(RollingStock())

        self.current_allocated_resources: List[Tuple[int, int]] = []
        self.current_resource_reservation_point = Tuple[int, int]

        self.currentVelocity_ReservationPoint: float = 0.0
        self.currentDistance_ReservationPoint: float = 0.0
        self.currentVelocity_Train: float = 0.0
        self.currentAcceleration_Train: float = 0.0
        self.currentDistance_Train: float = 0.0

    def update_dynamics(self):
        print(self.handle, 'update dynamics', self.position)

    def set_length(self, length=400):
        '''
        Sets the physical total length in meter
        :param length: in meter
        '''
        self.length = length

    def set_mass(self, mass):
        '''
        Sets the physical total mass in tonne
        :param mass:
        '''
        self.mass = mass

    def set_rolling_stock(self, rolling_stock: RollingStock):
        '''
        Sets the rolling stock information / traction data
        :param rolling_stock: a reference to the rolling stock object
        '''
        self.rolling_stock = rolling_stock

    def _copy_original_env_agent(self, env_agent: EnvAgent):
        self.initial_position = env_agent.initial_position
        self.initial_direction = env_agent.initial_direction
        self.direction = env_agent.direction
        self.target = env_agent.target
        self.moving = env_agent.moving

        # NEW : EnvAgent - Schedule properties
        self.earliest_departure = env_agent.earliest_departure
        self.latest_arrival = env_agent.latest_arrival

        self.handle = env_agent.handle

        # Env step facelift
        self.speed_counter = env_agent.speed_counter
        self.action_saver = env_agent.action_saver
        self.state_machine = env_agent.state_machine
        self.malfunction_handler = env_agent.malfunction_handler

        self.position = env_agent.position

        # NEW : EnvAgent Reward Handling
        self.arrival_time = env_agent.arrival_time

        # used in rendering
        self.old_direction = env_agent.old_direction
        self.old_position = env_agent.old_position

    def reset(self):
        super(XDynamicAgent, self).reset()
