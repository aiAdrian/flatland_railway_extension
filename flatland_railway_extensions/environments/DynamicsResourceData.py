from typing import Tuple

from flatland_railway_extensions.environments.InfrastructureData import InfrastructureData


class DynamicsResourceData:
    def __init__(self, res: Tuple[int, int], infrastructure_data: InfrastructureData):
        self.gradient: float = 0.0
        self.distance: float = 400
        self.max_velocity: float = 200 / 3.6
        self.backward = False
        self.random_braking_factor = 1.0

        if infrastructure_data is not None:
            self.distance = infrastructure_data.get_cell_length(res)
            self.max_velocity = infrastructure_data.get_velocity(res)
            self.gradient = infrastructure_data.get_gradient(res)
