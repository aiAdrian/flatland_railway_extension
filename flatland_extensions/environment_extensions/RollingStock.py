class RollingStock:
    def __init__(self,
                 max_traction=210000.0,
                 v_max_traction=200/3.6,
                 a_max_acceleration=10.0,
                 a_max_braking=-0.1,
                 mass_factor=1.05,
                 k=0.5,
                 c=2.5):
        self.max_traction: float = max_traction
        self.velocity_max_traction: float = v_max_traction
        self.set_a_max_acceleration(a_max_acceleration)
        self.set_a_max_braking(a_max_braking)
        self.mass_factor: float = mass_factor
        self.K: float = k
        self.C: float = c

    def set_a_max_acceleration(self, a_max_acceleration):
        self.a_max_acceleration = a_max_acceleration

    def set_a_max_braking(self, a_max_braking):
        self.a_max_braking = a_max_braking