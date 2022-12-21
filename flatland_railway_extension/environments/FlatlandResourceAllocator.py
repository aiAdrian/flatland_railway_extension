from typing import Tuple, List, Union

import numpy as np
from flatland.envs.rail_env import RailEnv
from matplotlib import pyplot as plt


class FlatlandResourceAllocator:
    def __init__(self, env: RailEnv):
        self.env = env
        self._min_time_step_difference = -np.inf
        self._resource_lock_grid: Union[np.array, None] = None
        self._reallocate_resource_lock_grid: Union[np.array, None] = None
        self._resource_lock_timestamp: Union[np.array, None] = None
        self.reset()

    def reset(self):
        '''
        This method reset whole internal data
        '''
        self._min_time_step_difference = -np.inf
        self._resource_lock_timestamp = \
            np.ones((self.env.height, self.env.width)) * (-np.inf)
        self._reallocate_resource_lock_grid = \
            np.ones((self.env.height, self.env.width)) * FlatlandResourceAllocator._free_resource_holder_handle()
        self.reset_locks()

    def reset_locks(self):
        '''
        This method reset only the lock grid (map) -> allocated resources.
        '''
        self._resource_lock_grid = \
            np.ones((self.env.height, self.env.width)) * FlatlandResourceAllocator._free_resource_holder_handle()

    @staticmethod
    def _free_resource_holder_handle() -> int:
        '''
        If this reference is stored in the resource lock grid, then the resource is not allocated by any agent
        :return: fix value == -1
        '''
        return -1

    def set_minimal_free_time_to_reallocate_other_agent(self, time: int):
        '''
        Set the global resource reallocation time
        :param time: time
        '''
        self._min_time_step_difference = time

    def get_minimal_free_time(self) -> int:
        return self._min_time_step_difference

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

    def _check_resources_before_allocate_or_deallocate(self, agent_handle, positions: List[Tuple[int, int]]) -> bool:
        '''
        This methods checks whether all resources are hold by the agent or are free. If only one resource is not free
        nor hold be passed agent the methods returns false
        :param agent_handle: the agent handle reference to the resource_holder
        :param positions: all list of resources passed as cell pos which has to be free
        :return: True if all resources are free or hold by passed agent otherwise returns false
        '''
        for pos in positions:
            if self.is_resource_locked(pos):
                if self._get_resource_holder(pos) != agent_handle:
                    return False
        return True

    def get_resources_free_time(self, position: Tuple[int, int]) -> float:
        '''
        This methods calculates the resource free-time (delta-time since last reservation), this means the duration in
        time steps since last reservation.
        :param agent_handle: the agent handle reference to the resource_holder
        :param position: the resource to calculate the delta time
        :return: delta-time since last reservation
        '''
        delta_time = np.inf
        lock = self._reallocate_resource_lock_grid[position]
        if lock != FlatlandResourceAllocator._free_resource_holder_handle():
            lock_time = self._resource_lock_timestamp[position]
            if lock_time != np.inf:
                delta_time = self.env._elapsed_steps - lock_time
        return delta_time

    def _check_resources_timestamp_before_allocate(self, agent_handle, positions: List[Tuple[int, int]]) -> bool:
        '''
        This method checks whether the last lock ( free ) is elder than minimal free time to reallocate other agent
        :param agent_handle: the agent handle reference to the resource_holder
        :param positions: all list of resources passed as cell pos which has to be free
        :return: True if delta time is ok otherwise false
        '''
        for pos in positions:
            lock = self._reallocate_resource_lock_grid[pos]
            if agent_handle != lock and lock != FlatlandResourceAllocator._free_resource_holder_handle():
                lock_time = self._resource_lock_timestamp[pos]
                if lock_time != np.inf:
                    delta_time = self.env._elapsed_steps - lock_time
                    if delta_time < self._min_time_step_difference:
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
        if not self._check_resources_before_allocate_or_deallocate(agent_handle, positions):
            return False
        if not self._check_resources_timestamp_before_allocate(agent_handle, positions):
            return False
        for pos in positions:
            self._resource_lock_grid[pos] = agent_handle
            self._reallocate_resource_lock_grid[pos] = agent_handle
            self._resource_lock_timestamp[pos] = self.env._elapsed_steps
        return True

    def deallocate_resource(self, agent_handle: int, positions: List[Tuple[int, int]]) -> bool:
        '''
        Deallocates a resource only if hold by passed agent
        :param agent_handle: the agent handle reference to the resource_holder
        :param positions: all list of resources passed as cell pos
        :return: true if the resource was succefully deallocated other wise false
        '''
        if not self._check_resources_before_allocate_or_deallocate(agent_handle, positions):
            return False
        for pos in positions:
            self._resource_lock_grid[pos] = FlatlandResourceAllocator._free_resource_holder_handle()
            self._reallocate_resource_lock_grid[pos] = agent_handle
            self._resource_lock_timestamp[pos] = self.env._elapsed_steps
        return True

    def get_resource_lock_timestamp(self) -> np.array:
        '''
        :return: a copy of the lock timestamp map
        '''
        return np.copy(self._resource_lock_timestamp)

    def get_resource_lock_grid(self) -> np.array:
        '''
        :return: a copy of the lock map
        '''
        return np.copy(self._resource_lock_grid)

    def do_debug_plot(self):
        '''
        Open a new window and render debug data
        '''
        resource_lock_grid_image = self.get_resource_lock_grid()
        resource_lock_timestamp_image = self.get_resource_lock_timestamp()

        for h in range(self.env.height):
            for w in range(self.env.width):
                if self.env.rail.grid[(h, w)] == 0:
                    resource_lock_grid_image[(h, w)] = np.nan
                    resource_lock_timestamp_image[(h, w)] = np.nan

        plt.rc('font', size=4)
        ax1 = plt.subplot(1, 2, 1)
        plt.imshow(resource_lock_grid_image)
        for (j, i), label in np.ndenumerate(self._resource_lock_grid):
            if label > -1:
                ax1.text(i, j, int(label), ha='center', va='center', color='white')
        ax1.set_title('FlatlandResourceAllocator: resource_lock_grid', fontsize=10)
        ax2 = plt.subplot(1, 2, 2)
        plt.imshow(resource_lock_timestamp_image)
        for (j, i), label in np.ndenumerate(self._resource_lock_timestamp):
            if label > -1:
                ax2.text(i, j, '{:5.1f}'.format(label), ha='center', va='center', color='black')
        ax2.set_title('FlatlandResourceAllocator: resource_lock_timestamp', fontsize=10)

        plt.show()
