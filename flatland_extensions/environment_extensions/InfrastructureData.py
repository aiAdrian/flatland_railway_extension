from typing import Union, Tuple

import numpy as np


class InfrastructureData:
    def __init__(self):
        # infrastructure
        self._infrastructure_max_velocity_grid: Union[np.array, None] = None
        self._infrastructure_cell_length_grid: Union[np.array, None] = None
        self._infrastructure_gradient_grid: Union[np.array, None] = None

    def set_infrastructure_max_velocity_grid(self, infrastructure_max_velocity_grid: np.array):
        self._infrastructure_max_velocity_grid = infrastructure_max_velocity_grid

    def set_infrastructure_cell_length_grid(self, infrastructure_cell_length_grid: np.array):
        self._infrastructure_cell_length_grid = infrastructure_cell_length_grid

    def set_infrastructure_gradient_grid(self, infrastructure_gradient_grid: np.array):
        self._infrastructure_gradient_grid = infrastructure_gradient_grid

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

    def get_gradient(self, res: Tuple[int, int]):
        if res is None:
            return 0
        if self._infrastructure_gradient_grid is not None:
            return self._infrastructure_gradient_grid[res[0], res[1]]
        return 0
