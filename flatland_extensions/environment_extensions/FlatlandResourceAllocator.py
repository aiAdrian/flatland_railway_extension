from typing import Tuple, List

import numpy as np
from flatland.envs.rail_env import RailEnv
from matplotlib import pyplot as plt


class FlatlandResourceAllocator:
    def __init__(self, env: RailEnv):

        self.env = env
        self._resource_lock_grid = \
            np.ones((self.env.height, self.env.width)) * FlatlandResourceAllocator._free_resource_holder_handle()

    def reset(self):
        self._resource_lock_grid = \
            np.ones((self.env.height, self.env.width)) * FlatlandResourceAllocator._free_resource_holder_handle()

    @staticmethod
    def _free_resource_holder_handle() -> int:
        '''
        If this reference is stored in the resource lock grid, then the resource is not allocated by any agent
        :return: fix value == -1
        '''
        return -1

    def is_resource_locked(self, pos: Tuple[int, int]) -> bool:
        '''
        Checks the resource whether it's locked or not (allocated by an agent or not)
        :param pos: resource as cell pos
        :return: if false the resource is free otherwise it is locked
        '''
        return self._resource_lock_grid[pos] != FlatlandResourceAllocator._free_resource_holder_handle()

    def _get_resource_holder(self, pos: Tuple[int, int]) -> int:
        '''
        :param pos: resource as cell pos
        :return: the resource_holder or  FlatlandResourceAllocator._free()
        '''
        return self._resource_lock_grid[pos]

    def _check_resources_before_allocate(self, agent_handle, positions: List[Tuple[int, int]]) -> bool:
        '''
        This methods checks whether all resources are hold by the agent or are free. If only one resource is not free
        nor hold be passed agent the methods returns false
        :param agent_handle:
        :param positions: all list of resources passed as cell pos which has to be free
        :return: True if all resources are free or hold by passed agent otherwise returns false
        '''
        for pos in positions:
            if self.is_resource_locked(pos):
                if self._get_resource_holder(pos) != agent_handle:
                    return False
        return True

    def get_assigned_resources(self, agent_handle: int) -> List[Tuple[int, int]]:
        locked_ref = np.argwhere(self._resource_lock_grid == agent_handle)
        positions = []
        for res in locked_ref:
            positions.append(tuple(res))
        return positions

    def allocate_resource(self, agent_handle: int, positions: List[Tuple[int, int]]) -> bool:
        '''
        Allocates a resource to an agent only if it is free or self-held
        :param agent_handle: the agent handle reference to the resource_holder (or the agent who likes to allocate
        the resource
        :param positions: all list of resources passed as cell pos
        :return: True if the cell is owned by the agent (resource_holder) - either is still holds the resource or
        get it new
        '''
        if not self._check_resources_before_allocate(agent_handle, positions):
            return False
        for pos in positions:
            self._resource_lock_grid[pos] = agent_handle
        return True

    def deallocate_resource(self, agent_handle: int, positions: List[Tuple[int, int]]) -> bool:
        '''
        Deallocates a resource only if hold by passed agent
        :param agent_handle: the agent handle reference to the resource_holder
        :param positions: all list of resources passed as cell pos
        :return: true if the resource was succefully deallocated other wise false
        '''
        if not self._check_resources_before_allocate(agent_handle, positions):
            return False
        for pos in positions:
            self._resource_lock_grid[pos] = FlatlandResourceAllocator._free_resource_holder_handle()
        return True

    def do_debug_plot(self):
        # Setup renderer
        resource_lock_grid_image = np.copy(self._resource_lock_grid)

        for h in range(self.env.height):
            for w in range(self.env.width):
                if self.env.rail.grid[h][w] == 0:
                    resource_lock_grid_image[h][w] = np.nan

        plt.rc('font', size=4)
        ax1 = plt.subplot(1, 1, 1)
        plt.imshow(resource_lock_grid_image)
        for (j, i), label in np.ndenumerate(self._resource_lock_grid):
            if label > 0:
                ax1.text(i, j, int(label), ha='center', va='center', color='white')
        ax1.set_title('FlatlandResourceAllocator: resource_lock_grid', fontsize=10)
        plt.show()
