from typing import List, Union, Tuple

from flatland.core.grid.grid4_utils import get_new_position
from flatland.core.transition_map import GridTransitionMap
from flatland.envs.distance_map import DistanceMap

from flatland_railway_extension.environments.DynamicAgent import DynamicAgent
from flatland_railway_extension.environments.InfrastructureData import InfrastructureData


class FlatlandDynamicsDistanceMap(DistanceMap):
    def __init__(self, agents: List[DynamicAgent], env_height: int, env_width: int):
        super(FlatlandDynamicsDistanceMap, self).__init__(agents, env_height, env_width)
        self._infrastructure_data: Union[InfrastructureData, None] = None

    def set_infrastructure_data(self, infrastructure_data: Union[InfrastructureData, None]):
        """
        Set the infrastructure data which plays a key role with FlatlandDynamics
        """
        self._infrastructure_data = infrastructure_data

    def estimate_edge_len(self, pos: Tuple[int, int]) -> float:
        """
        Estimate the cell distance ending in the distance map. The currently implemented heuristic is very simple:
        If the infrastructure data is missing, the cell length becomes one - otherwise the cell length is approximated
        with the physical distance (in meters) and the maximum allowed speed (in meters per second). So the edge length
        is the expected time spent on the item assuming the agent is traveling with maximal allowed speed.
        """
        if self._infrastructure_data is not None:
            edge_len = self._infrastructure_data.get_cell_length(pos)
            edge_vel = self._infrastructure_data.get_velocity(pos)
            return edge_len / edge_vel
        return 1.0

    def _get_and_update_neighbors(self, rail: GridTransitionMap, position, target_nr, current_distance,
                                  enforce_target_direction=-1):

        """
        Utility function used by _distance_map_walker to perform a BFS walk over the rail, filling in the
        minimum distances from each target cell.
        """
        neighbors = []

        possible_directions = [0, 1, 2, 3]
        if enforce_target_direction >= 0:
            # The agent must land into the current cell with orientation `enforce_target_direction'.
            # This is only possible if the agent has arrived from the cell in the opposite direction!
            possible_directions = [(enforce_target_direction + 2) % 4]

        for neigh_direction in possible_directions:
            new_cell = get_new_position(position, neigh_direction)

            if new_cell[0] >= 0 and new_cell[0] < self.env_height and new_cell[1] >= 0 and new_cell[1] < self.env_width:

                desired_movement_from_new_cell = (neigh_direction + 2) % 4

                # Check all possible transitions in new_cell
                for agent_orientation in range(4):
                    # Is a transition along movement `desired_movement_from_new_cell' to the current cell possible?
                    is_valid = rail.get_transition((new_cell[0], new_cell[1], agent_orientation),
                                                   desired_movement_from_new_cell)

                    if is_valid:
                        """
                        # TODO: check that it works with deadends! -- still bugged!
                        movement = desired_movement_from_new_cell
                        if isNextCellDeadEnd:
                            movement = (desired_movement_from_new_cell+2) % 4
                        """
                        new_distance = min(self.distance_map[target_nr, new_cell[0], new_cell[1], agent_orientation],
                                           current_distance + self.estimate_edge_len(new_cell))
                        neighbors.append((new_cell[0], new_cell[1], agent_orientation, new_distance))
                        self.distance_map[target_nr, new_cell[0], new_cell[1], agent_orientation] = new_distance

        return neighbors
