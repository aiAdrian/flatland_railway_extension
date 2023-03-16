class RollingStock:
    def __init__(self,
                 max_traction=210000.0,
                 velocity_max_traction=90 / 3.6,
                 max_acceleration=10.0,
                 max_braking_acceleration=-0.1,
                 max_velocity=200 / 3.6,
                 mass_factor=1.05,
                 k=0.5,
                 c=2.5):
        self.max_traction: float = max_traction
        self.velocity_max_traction: float = velocity_max_traction
        self.max_velocity: float = max_velocity
        self.set_max_acceleration(max_acceleration)
        self.set_max_braking_acceleration(max_braking_acceleration)
        self.mass_factor: float = mass_factor
        self.K: float = k
        self.C: float = c

    def get_max_tractive_effort(self, current_velocity: float) -> float:
        if current_velocity <= self.velocity_max_traction:
            return self.max_traction
        if current_velocity > self.max_velocity:
            return 0
        return self.max_traction_power() / current_velocity

    def max_traction_power(self) -> float:
        return self.velocity_max_traction * self.max_traction

    def set_max_acceleration(self, a_max_acceleration: float):
        self.a_max_acceleration = a_max_acceleration

    def set_max_braking_acceleration(self, max_braking_acceleration: float):
        self.max_braking_acceleration = max_braking_acceleration

    def get_current_tractive_effort(self, total_resistance: float, max_traction: float) -> float:
        if total_resistance < max_traction:
            return total_resistance
        return max_traction

    def get_current_resistance(self,
                               current_gradient: float,
                               current_velocity: float,
                               train_total_mass: float,
                               max_allowed_velocity: float,
                               simulation_time_step: float):

        # total resistance = what the traction should perform  -> tractive effort

        # current gradient ( oriented ) equals to gradient_resistance
        gradient_resistance = current_gradient

        # run resistances - air resistance / drag
        train_run_resistance = gradient_resistance + \
                               self.C + \
                               self.K * current_velocity * current_velocity * 0.01296

        total_resistance = train_run_resistance

        # accelerate
        acceleration_train_point = self.a_max_acceleration
        acceleration = max(float(0.0), max_allowed_velocity - current_velocity) / simulation_time_step
        if acceleration < acceleration_train_point:
            acceleration_train_point = acceleration

        if acceleration_train_point > 0.0:
            total_resistance = total_resistance + self.mass_factor * acceleration_train_point * 100.0

        total_resistance = total_resistance * train_total_mass * 9.81

        return total_resistance, train_run_resistance

    def get_accelerations_and_tractive_effort(self,
                                              current_velocity,
                                              max_allowed_velocity,
                                              current_gradient,
                                              train_total_mass,
                                              simulation_time_step):
        '''
        This method calculates the current train acceleration, the maximal braking (acceleration) which is currently
        possible and finally it returns the tractive effort of the traction
        :param current_velocity: current train speed (m/s)
        :param max_allowed_velocity: current maximal allowed speed at agent (train)
        :param current_gradient: current gradient (topology)
        :param train_total_mass: total train mass including all vehicles
        :param simulation_time_step: the simulation time step (euler step duration)
        :return: train_acceleration, max_braking_acceleration, max_tractive_effort
        '''
        total_resistance, train_run_resistance = self.get_current_resistance(
            current_gradient,
            current_velocity,
            train_total_mass,
            max_allowed_velocity,
            simulation_time_step)

        max_tractive_effort = self.get_current_tractive_effort(
            total_resistance,
            self.get_max_tractive_effort(current_velocity))
        # calculate the train acceleration
        train_acceleration = \
            (max_tractive_effort / train_total_mass - train_run_resistance * 9.81) * (0.001 / self.mass_factor)

        # update max braking acceleration
        max_braking_acceleration = self.max_braking_acceleration
        if train_acceleration < 0.0:
            max_braking_acceleration = max_braking_acceleration + train_acceleration

        return train_acceleration, max_braking_acceleration, max_tractive_effort
