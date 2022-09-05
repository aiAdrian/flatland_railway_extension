from typing import Tuple

from flatland_extensions.environment_extensions.InfrastructureData import InfrastructureData


class DynamicsResourceData:
    def __init__(self, res: Tuple[int, int], infrastructure_data: InfrastructureData):
        self.gradient: float = 0.0
        self.distance: float = 400
        self.vMax: float = 200 / 3.6
        self.backward = False
        self.rndBreakFactor = 1.0

        if infrastructure_data is not None:
            self.distance = infrastructure_data.get_cell_length(res)
            self.vMax = infrastructure_data.get_velocity(res)
            self.gradient = infrastructure_data.get_gradient(res)