import numpy as np
# import all flatland dependance
from flatland.core.grid.grid4_utils import get_new_position
from flatland.envs.fast_methods import fast_count_nonzero, fast_argmax
from matplotlib import pyplot as plt

from flatland_extensions.RailroadSwitchAnalyser import RailroadSwitchAnalyser


class RailroadSwitchCluster:
    def __init__(self, railroad_switch_analyser: RailroadSwitchAnalyser):
        self.railroad_switch_analyser = railroad_switch_analyser
        self.env = self.railroad_switch_analyser.get_rail_env()
        self.railroad_switch_clusters = {}
        self.connecting_edge_clusters = {}
        self._cluster_all_non_switches()
        self._cluster_all_switches()

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
                            possible_transitions = self.env.rail.get_transitions(*t_working_position, direction_loop)
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

    def _cluster_all_non_switches(self):
        info_image = np.zeros((self.env.height, self.env.width))
        for h in range(self.env.height):
            for w in range(self.env.width):
                # look one step forward
                if self.env.rail.grid[h][w] > 0:
                    info_image[(h, w)] = 1

        for key in self.railroad_switch_analyser.railroad_switches.keys():
            info_image[key] = 0
        for key in self.railroad_switch_analyser.railroad_switch_neighbours.keys():
            info_image[key] = 0

        # build clusters
        self._find_connected_clusters_and_label(info_image)
        self.connecting_edge_cluster_grid = self.railroad_switch_cluster_grid.copy()
        self.connecting_edge_clusters = self.railroad_switch_clusters.copy()

        data2upate = []
        updatedEdge = []
        for edge in self.railroad_switch_analyser.railroad_switch_neighbours.keys():
            for idir in range(4):
                possible_transitions = self.env.rail.get_transitions(*edge, idir)
                for new_direction in range(4):
                    if possible_transitions[new_direction] == 1:
                        new_position = get_new_position(edge, new_direction)
                        cid = self.connecting_edge_cluster_grid[new_position]
                        if (cid > 0) and (not (edge in updatedEdge)):
                            data2upate.append([edge, cid])
                            updatedEdge.append(edge)

        for d in data2upate:
            pos = d[0]
            cid = d[1]
            self.connecting_edge_cluster_grid[pos] = cid
            c = self.connecting_edge_clusters.get(cid, [])
            c.append(pos)
            self.connecting_edge_clusters.update({cid: c})

        info_image = np.zeros((self.env.height, self.env.width))
        for e in self.railroad_switch_analyser.railroad_switch_neighbours.keys():
            if not (e in updatedEdge):
                info_image[e] = 1
        self._find_connected_clusters_and_label(info_image)
        connecting_edge_cluster2 = self.railroad_switch_clusters.copy()
        for ce in connecting_edge_cluster2.keys():
            es = connecting_edge_cluster2.get(ce)
            m = np.max(self.connecting_edge_cluster_grid) + 1
            self.connecting_edge_clusters.update({m: es})
            for e in es:
                self.connecting_edge_cluster_grid[e] = m

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

        plt.rc('font', size=4)
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
