from typing import Tuple

import numpy as np
# import all flatland dependance
from flatland.core.grid.grid4_utils import get_new_position
from flatland.envs.fast_methods import fast_count_nonzero
from flatland.envs.rail_env import RailEnv
from matplotlib import pyplot as plt


class RailroadSwitchAnalyser:
    def __init__(self, env: RailEnv, handle_diamond_crossing_as_a_switch=True, handle_dead_end_as_a_switch=True):
        self.env = env

        self.handle_diamond_crossing_as_a_switch = handle_diamond_crossing_as_a_switch
        self.handle_dead_end_as_a_switch = handle_dead_end_as_a_switch

        # reset the internal data structures used for agent can choose
        self.railroad_switches = {}
        self.railroad_switch_neighbours = {}

        # prepare the memory - collect all cells where the agent can choose more than FORWARD/STOP.
        self._find_all_railroad_switches()
        self._find_all_railroad_switch_neighbours()

    def _find_all_railroad_switches(self):
        '''
        Search the environment (rail grid) for all railroad_switch cells. A railroad_switch is a cell
        where more than one transition exists and collect all start_direction where the railroad_switch is a
        railroad_switch.

        :returns
        '''
        self.railroad_switches = {}
        self.railroad_dead_end = []
        self.railroad_diamond_crossing = []
        diamond = int('1000010000100001', 2)
        for h in range(self.env.height):
            for w in range(self.env.width):
                pos = (h, w)
                for direction in range(4):
                    possible_transitions = self.env.rail.get_transitions((pos, direction))
                    num_transitions = fast_count_nonzero(possible_transitions)
                    if num_transitions > 1:
                        directions = self.railroad_switches.get(pos, [])
                        directions.append(direction)
                        self.railroad_switches.update({pos: directions})
                if self.env.rail.is_dead_end(pos):
                    # dead-end
                    self.railroad_dead_end.append(pos)
                    if self.handle_dead_end_as_a_switch:
                        self.railroad_switches.update({pos: [i for i in range(4)]})
                if self.env.rail.grid[(h, w)] == diamond:
                    # diamond crossing
                    self.railroad_diamond_crossing.append(pos)
                    if self.handle_diamond_crossing_as_a_switch:
                        self.railroad_switches.update({pos: [i for i in range(4)]})

    def _find_all_railroad_switch_neighbours(self):
        '''
        Collect all cells where is a neighbour to a railroad_switch cell. All cells are neighbour where the agent can
        make just one step and he stands on a railroad_switch. A railroad_switch is a cell where the agents has more
        than one transition.

        :return:
        '''
        self.railroad_switch_neighbours = {}
        for h in range(self.env.height):
            for w in range(self.env.width):
                # look one step forward
                for direction in range(4):
                    pos = (h, w)
                    if pos not in self.railroad_switches.keys():
                        possible_transitions = self.env.rail.get_transitions((pos, direction))
                        for d in range(4):
                            if possible_transitions[d] == 1:
                                new_cell = get_new_position(pos, d)
                                if new_cell in self.railroad_switches.keys():
                                    directions = self.railroad_switch_neighbours.get(pos, [])
                                    directions.append(direction)
                                    self.railroad_switch_neighbours.update({pos: directions})

    def prepare_observation_data_plot(self):
        '''
        Prepare the classified cells (railroad_switches, railroad_switch_neighbours) to render. The rendering used
        the observation rendering data structure.

        :return:
        '''
        for h in self.env.get_agent_handles():
            self.env.dev_obs_dict.update({h: []})
            if h == 0:
                self.env.dev_obs_dict.update({0: self.railroad_switch_neighbours.keys()})
            elif h == 1:
                self.env.dev_obs_dict.update({1: self.railroad_switches.keys()})

    def check_agent_decision(self, position, direction):
        '''
         Decide whether the agent is
         - on a railroad_switch
         - at a railroad_switch neighbour (near to railroad_switch). The railroad_switch must be a railroad_switch
         where the agent has more option than FORWARD/STOP
         - all railroad_switch : doesn't matter whether the agent has more options than FORWARD/STOP
         - all railroad_switch neightbors : doesn't matter the agent has more then one options (transition) when
           the agent reachs the railroad_switch

        :param position: (x,y) cell coordinate
        :param direction: flatland start_direction

        :return: agent_at_railroad_switch, agent_near_to_railroad_switch,
                 agent_at_railroad_switch_cell, agent_near_to_railroad_switch_cell
        '''
        agent_at_railroad_switch = False
        agent_at_railroad_switch_cell = False
        agent_near_to_railroad_switch = False
        agent_near_to_railroad_switch_cell = False
        if position in self.railroad_switches.keys():
            agent_at_railroad_switch = direction in self.railroad_switches[position]
            agent_at_railroad_switch_cell = True

        if position in self.railroad_switch_neighbours.keys():
            new_cell = get_new_position(position, direction)
            if new_cell in self.railroad_switches.keys():
                if not (direction in self.railroad_switches[new_cell]):
                    agent_near_to_railroad_switch = direction in self.railroad_switch_neighbours[position]
            else:
                agent_near_to_railroad_switch = direction in self.railroad_switch_neighbours[position]

            agent_near_to_railroad_switch_cell = True

        return agent_at_railroad_switch, agent_near_to_railroad_switch, \
               agent_at_railroad_switch_cell, agent_near_to_railroad_switch_cell

    def get_rail_env(self) -> RailEnv:
        return self.env

    def is_diamond_crossing(self, pos: Tuple[int, int]) -> bool:
        return pos in self.railroad_diamond_crossing

    def is_dead_end(self, pos: Tuple[int, int]) -> bool:
        return pos in self.railroad_dead_end

    def is_switch_neighbor(self, pos: Tuple[int, int]) -> bool:
        return pos in self.railroad_switch_neighbours.keys()

    def do_debug_plot(self):
        # Setup renderer
        switch_image = np.zeros((self.env.height, self.env.width)) * np.nan
        switches_neighbours_image = np.zeros((self.env.height, self.env.width)) * np.nan
        for h in range(self.env.height):
            for w in range(self.env.width):
                # look one step forward
                if self.env.rail.grid[(h, w)] > 0:
                    switch_image[(h, w)] = 1
                    switches_neighbours_image[(h, w)] = 1

        for key in self.railroad_switches.keys():
            switch_image[key] = 2

        for key in self.railroad_switch_neighbours.keys():
            switches_neighbours_image[key] = 2

        plt.rc('font', size=4)
        ax1 = plt.subplot(1, 2, 1)
        plt.imshow(switch_image)
        ax1.set_title('railroad_switches', fontsize=10)
        ax2 = plt.subplot(1, 2, 2)
        plt.imshow(switches_neighbours_image)
        ax2.set_title('railroad_switches_neighbours', fontsize=10)
        plt.show()
