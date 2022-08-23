import networkx as nx
import numpy as np
# import all flatland dependance
from flatland.core.grid.grid4_utils import get_new_position
from matplotlib import pyplot as plt

from flatland_extensions.RailroadSwitchAnalyser import RailroadSwitchAnalyser


class FlatlandGraphBuilder:
    def __init__(self, railroad_switch_analyser: RailroadSwitchAnalyser):
        self.railroad_switch_analyser = railroad_switch_analyser
        self.graph, self.nodes = self._create_graph()

    def _create_graph(self):
        graph = nx.DiGraph(directed=True)
        env = self.railroad_switch_analyser.get_rail_env()
        nodes = {}
        for h in range(env.height):
            for w in range(env.width):
                pos = (h, w)
                for dir in range(4):
                    possible_transitions = env.rail.get_transitions(*pos, dir)

                    agent_at_railroad_switch, agent_near_to_railroad_switch, \
                    agent_at_railroad_switch_cell, agent_near_to_railroad_switch_cell = \
                        self.railroad_switch_analyser.check_agent_decision(position=pos, direction=dir)

                    for to_direction in range(4):
                        if possible_transitions[to_direction] == 1:
                            new_position = get_new_position(pos, to_direction)
                            from_vertex_name = '{}_{}_{}'.format(pos[0], pos[1], dir)
                            to_vertex_name = '{}_{}_{}'.format(new_position[0], new_position[1], to_direction)
                            graph.add_edge(from_vertex_name, to_vertex_name,
                                           from_direction=dir,
                                           to_direction=to_direction,
                                           resource_pos=pos,
                                           resource_id='{}_{}'.format(pos[0], pos[1]),
                                           from_is_railroad_switch=agent_at_railroad_switch,
                                           from_near_to_railroad_switch=agent_near_to_railroad_switch,
                                           from_railroad_switch_cell=agent_at_railroad_switch_cell,
                                           from_agent_near_to_railroad_switch_cell=agent_near_to_railroad_switch_cell)
                            nodes.update({from_vertex_name: (pos[0], pos[1], dir)})
                            nodes.update({to_vertex_name: (new_position[0], new_position[1], to_direction)})

        return graph, nodes

    def get_edge_weight(self, e) -> float:
        return 1

    def make_node_label(self, node) -> str:
        return node

    def make_edge_label(self, edge) -> str:
        return self.graph.get_edge_data(edge[0], edge[1])['resource_pos']

    def render(self):
        nodes = self.graph.nodes()
        positions = {}
        for n in nodes:
            pos_xy_dir = self.nodes.get(n)
            pos = pos_xy_dir[:2]
            dir = pos_xy_dir[2]
            x = pos[0] + self.railroad_switch_analyser.get_rail_env().width * (dir % 2)
            y = pos[1] + self.railroad_switch_analyser.get_rail_env().height * (dir // 2)
            positions.update({n: (y, -x)})

        plt.figure()
        nx.draw(
            self.graph, positions, edge_color='black', width=1, linewidths=1,
            node_size=200, node_color='lightgray', alpha=1.0, font_size=8,
            labels={node: self.make_node_label(node) for node in self.graph.nodes()}
        )

        nx.draw_networkx_edge_labels(
            self.graph, positions,
            edge_labels={e: self.make_edge_label(e) for e in self.graph.edges()},
            font_color='red', font_size=8
        )
        plt.axis('off')
        plt.show()

    @staticmethod
    def get_coordinate_direction_from_node_id(node_name: str):
        xy_d = node_name.split('_')
        xy = (int(xy_d[0]), int(xy_d[1]))
        direction = int(xy_d[1])
        return xy, direction

    @staticmethod
    def get_resource_id_from_node_id(node_name: str):
        xy_d = node_name.split('_')
        resource_id = '{}_{}'.format(int(xy_d[0]), int(xy_d[1]))
        return resource_id

    def get_shortest_path(self, start_position, start_direction, target_position):
        '''
        This methods traverse the graph to get shortest path. The target gets scaned for all four incoming directions.
        :param start_position: 2d coordinate
        :param start_direction: orientation passed a agent moving direction
        :param target_position: 2d coordinate of target position
        :return: shortest path, all paths, len of all found paths
        '''
        from_node = '{}_{}_{}'.format(start_position[0], start_position[1], start_direction)
        paths = []
        paths_len = []
        for target_direction in range(4):
            path = []
            to_node = '{}_{}_{}'.format(target_position[0], target_position[1], target_direction)
            fn = self.graph.nodes.get(from_node, None)
            tn = self.graph.nodes.get(to_node, None)
            if fn is not None and tn is not None:
                s_path = nx.shortest_path(self.graph, from_node, to_node)
                for p in s_path:
                    xy, _ = FlatlandGraphBuilder.get_coordinate_direction_from_node_id(p)
                    path.append(xy)
            paths.append(path)
            path_len = len(path)
            if path_len == 0:
                path_len = np.inf
            paths_len.append(path_len)
        arg_sorted_path_len = np.argsort(paths_len)
        return paths[arg_sorted_path_len[0]], paths, paths_len

    def prepare_observation_data_plot(self, start_position, start_direction, target_position):
        '''
        Prepare the classified cells (railroad_switches, railroad_switch_neighbours) to render. The rendering used
        the observation rendering data structure.
        :param start_position: 2d coordinate
        :param start_direction: orientation passed a agent moving direction
        :param target_position: 2d coordinate of target position
        :return:
        '''
        path, paths, paths_len = self.get_shortest_path(start_position, start_direction, target_position)

        env = self.railroad_switch_analyser.get_rail_env()
        arg_sorted_path_len = np.argsort(paths_len)

        for h in env.get_agent_handles():
            env.dev_obs_dict.update({h: []})
            if h < 4:
                env.dev_obs_dict.update({h: paths[arg_sorted_path_len[h]]})

    def get_nodes(self):
        nodes = self.graph.nodes()
        return nodes
