import collections
from typing import Tuple

import numpy as np
# import all flatland dependance
from flatland.core.grid.grid4_utils import get_new_position
from flatland.envs.fast_methods import fast_count_nonzero, fast_argmax
from matplotlib import pyplot as plt

from flatland_extensions.FlatlandGraphBuilder import FlatlandGraphBuilder
from flatland_extensions.RailroadSwitchAnalyser import RailroadSwitchAnalyser

ClusterRefID = collections.namedtuple('ClusterRefID',
                                      'switch_cluster_ref '
                                      'connecting_edge_cluster_ref')
ClusterCellMembers = collections.namedtuple('ClusterInformation',
                                            'switch_cluster_cell_members '
                                            'connecting_edge_cluster_cell_members')


class RailroadSwitchCluster:
    def __init__(self, railroad_switch_analyser: RailroadSwitchAnalyser):
        self.railroad_switch_analyser = railroad_switch_analyser
        self.env = self.railroad_switch_analyser.get_rail_env()
        self.railroad_switch_clusters = {}
        self.connecting_edge_clusters = {}
        self._cluster_connecting_edge()
        self._cluster_all_switches()

    def _cluster_connecting_edge(self):
        self.connecting_edge_cluster_grid = np.zeros((self.env.height, self.env.width))

        flatland_graph_builder = FlatlandGraphBuilder(railroad_switch_analyser=self.railroad_switch_analyser)
        flatland_graph_builder.activate_simplified()
        edges = flatland_graph_builder.get_edges()
        cluster_id = 1
        for idx_edge, edge in enumerate(edges):
            resources = flatland_graph_builder.get_edge_resource(edge)
            update_cluster_id = 0
            for res in resources:
                if res in self.railroad_switch_analyser.railroad_switches.keys():
                    continue
                v = self.connecting_edge_cluster_grid[res]
                if v == 0:
                    self.connecting_edge_cluster_grid[res] = cluster_id
                    update_cluster_id = 1
            if update_cluster_id == 1:
                self.connecting_edge_clusters.update({cluster_id: resources})
            cluster_id += update_cluster_id

    def get_cluster_id(self, pos: Tuple[int, int]) -> ClusterRefID:
        return ClusterRefID(switch_cluster_ref=self.railroad_switch_cluster_grid[pos],
                            connecting_edge_cluster_ref=self.connecting_edge_cluster_grid[pos])

    def get_cluster_cell_members(self, cluster_id: ClusterRefID) -> ClusterCellMembers:
        switch_members = self.railroad_switch_clusters.get(cluster_id.switch_cluster_ref, [])
        connecting_edge_members = self.connecting_edge_clusters.get(cluster_id.connecting_edge_cluster_ref, [])
        return ClusterCellMembers(switch_cluster_cell_members=switch_members,
                                  connecting_edge_cluster_cell_members=connecting_edge_members)

    def _find_cluster_label(self, in_label) -> int:
        label = int(in_label)
        while 0 != self.label_dict[label]:
            label = self.label_dict[label]
        return label

    def _union_cluster_label(self, root, slave) -> None:
        root_label = self._find_cluster_label(root)
        slave_label = self._find_cluster_label(slave)
        if slave_label != root_label:
            self.label_dict[slave_label] = root_label

    def _find_connected_clusters_and_label(self, binary_image):
        self.railroad_switch_clusters = {}
        self.railroad_switch_cluster_grid = None

        padded_binary_image = np.pad(binary_image, ((1, 0), (1, 0)), 'constant', constant_values=(0, 0))
        w = np.size(binary_image, 1)
        h = np.size(binary_image, 0)
        self.label_dict = [int(i) for i in np.zeros(w * h)]
        label = 1
        #  first pass
        for cow in range(1, h + 1):
            for col in range(1, w + 1):
                working_position = (cow, col)
                working_pixel = padded_binary_image[working_position]
                if working_pixel != 0:
                    left_pixel_pos = (cow, col - 1)
                    up_pixel_pos = (cow - 1, col)

                    left_pixel = padded_binary_image[left_pixel_pos]
                    up_pixel = padded_binary_image[up_pixel_pos]

                    # Use connections (rails) for clustering (only real connected pixels builds a real cluster)
                    if (cow < self.env.height) and (col < self.env.width):
                        left_ok = 0
                        up_ok = 0
                        # correct padded image position (railenv)
                        t_working_position = (working_position[0] - 1, working_position[1] - 1)
                        t_left_pixel_pos = (left_pixel_pos[0] - 1, left_pixel_pos[1] - 1)
                        t_up_pixel_pos = (up_pixel_pos[0] - 1, up_pixel_pos[1] - 1)
                        for direction_loop in range(4):
                            possible_transitions = self.env.rail.get_transitions(*t_working_position,
                                                                                 direction_loop)
                            orientation = direction_loop
                            if fast_count_nonzero(possible_transitions) == 1:
                                orientation = fast_argmax(possible_transitions)
                            for dir_loop, new_direction in enumerate(
                                    [(orientation + dir_loop) % 4 for dir_loop in range(-1, 3)]):
                                if possible_transitions[new_direction] == 1:
                                    new_pos = get_new_position(t_working_position, new_direction)
                                    if new_pos == t_left_pixel_pos:
                                        left_ok = 1
                                    if new_pos == t_up_pixel_pos:
                                        up_ok = 1
                        left_pixel *= left_ok
                        up_pixel *= up_ok

                    # build clusters
                    if left_pixel == 0 and up_pixel == 0:
                        padded_binary_image[working_position] = label
                        label += 1

                    if left_pixel != 0 and up_pixel != 0:
                        smaller = left_pixel if left_pixel < up_pixel else up_pixel
                        bigger = left_pixel if left_pixel > up_pixel else up_pixel
                        padded_binary_image[working_position] = smaller
                        self._union_cluster_label(smaller, bigger)

                    if up_pixel != 0 and left_pixel == 0:
                        padded_binary_image[working_position] = up_pixel

                    if up_pixel == 0 and left_pixel != 0:
                        padded_binary_image[working_position] = left_pixel

        for cow in range(1, h + 1):
            for col in range(1, w + 1):
                root = self._find_cluster_label(padded_binary_image[cow][col])
                padded_binary_image[cow][col] = root

        self.railroad_switch_cluster_grid = padded_binary_image[1:, 1:]
        for h in range(self.env.height):
            for w in range(self.env.width):
                working_position = (h, w)
                root = self.railroad_switch_cluster_grid[working_position]
                if root > 0:
                    pos_data = self.railroad_switch_clusters.get(root, [])
                    pos_data.append(working_position)
                    self.railroad_switch_clusters.update({root: pos_data})

    def _cluster_all_switches(self):
        # mark railroad switches
        info_image = np.zeros((self.env.height, self.env.width))
        for key in self.railroad_switch_analyser.railroad_switches.keys():
            info_image[key] = 1

        # build clusters
        self._find_connected_clusters_and_label(info_image)

    def do_debug_plot(self):
        # Setup renderer
        connecting_edge_cells = []
        connecting_edge_cluster_grid_image = np.copy(self.connecting_edge_cluster_grid)
        railroad_switch_cluster_grid_image = np.copy(self.railroad_switch_cluster_grid)

        for h in range(self.env.height):
            for w in range(self.env.width):
                # look one step forward
                if self.env.rail.grid[h][w] > 0:
                    if self.connecting_edge_cluster_grid[(h, w)] > 0:
                        connecting_edge_cells.append((h, w))
                else:
                    connecting_edge_cluster_grid_image[h][w] = np.nan
                    railroad_switch_cluster_grid_image[h][w] = np.nan

        print('railroad_switch_cluster_grid')
        print(self.railroad_switch_cluster_grid)
        print('connecting_edge_cluster_grid')
        print(self.connecting_edge_cluster_grid)

        print('railroad_switch_clusters')
        print(self.railroad_switch_clusters)
        print('connecting_edge_clusters')
        print(self.connecting_edge_clusters)

        plt.rc('font', size=6)
        ax1 = plt.subplot(1, 2, 1)
        plt.imshow(connecting_edge_cluster_grid_image)
        for (j, i), label in np.ndenumerate(self.connecting_edge_cluster_grid):
            if label > 0:
                ax1.text(i, j, int(label), ha='center', va='center', color='white')
        ax1.set_title('connecting_edge_clusters', fontsize=10)
        ax2 = plt.subplot(1, 2, 2)
        plt.imshow(railroad_switch_cluster_grid_image)
        for (j, i), label in np.ndenumerate(self.railroad_switch_cluster_grid):
            if label > 0:
                ax2.text(i, j, int(label), ha='center', va='center', color='white')
        ax2.set_title('railroad_switch_clusters', fontsize=10)
        plt.show()
