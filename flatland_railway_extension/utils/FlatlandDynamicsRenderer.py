from typing import Union

import numpy as np
from flatland.envs.rail_env import RailEnv
from flatland.utils.rendertools import AgentRenderVariant
from flatland.utils.rendertools import RenderLocal

from flatland_railway_extension.environments import FlatlandResourceAllocator
from flatland_railway_extension.environments.DynamicAgent import DynamicAgent
from flatland_railway_extension.utils.FlatlandRenderer import FlatlandRenderer


class FlatlandDynamicsRenderer(FlatlandRenderer):
    def __init__(self, env: RailEnv, show_debug=False, show_agents=True,
                 agent_render_variant: AgentRenderVariant = AgentRenderVariant.BOX_ONLY,
                 cell_size=40, fix_aspect_ration=False):
        super(FlatlandDynamicsRenderer, self).__init__(env, show_debug, show_agents, agent_render_variant,
                                                       cell_size, fix_aspect_ration)
        self.flatland_resource_allocator: Union[FlatlandResourceAllocator, None] = None

    def set_flatland_resource_allocator(self, flatland_resource_allocator: FlatlandResourceAllocator):
        self.flatland_resource_allocator = flatland_resource_allocator

    def render(self, show=True, show_observations=True, show_predictions=False,
               disable_background_rendering=False, show_rowcols=False):
        super(FlatlandDynamicsRenderer, self).render(show, show_observations, show_predictions,
                                                     disable_background_rendering, show_rowcols)

        if self.flatland_resource_allocator is not None:
            self.env_renderer.renderer.gl.clear_layer(4)
            for row in range(self.env.height):
                for col in range(self.env.width):
                    res = (row, col)
                    mft = self.flatland_resource_allocator.get_minimal_free_time()
                    if mft > 0:
                        dt = self.flatland_resource_allocator.get_resources_free_time(res) / mft
                        if dt < 1.0:
                            self.draw_box(res, color=[dt, 0.8 + 0.2 * dt, dt], linewidth=2, draw_cell_size=0.8)

            for agent_handle, agent in enumerate(self.env.agents):
                if not isinstance(agent, DynamicAgent):
                    break
                color_idx = agent_handle % self.env_renderer.gl.n_agent_colors
                agent_color = self.env_renderer.gl.agent_colors[color_idx]

                allocated_resource = self.flatland_resource_allocator.get_assigned_resources(agent_handle=agent_handle)
                for res in allocated_resource:
                    self.draw_box(res, color=[1.0, 0.8, 0.0], linewidth=3, draw_cell_size=0.8)
                    self.draw_box(res, color=agent_color, linewidth=3, draw_cell_size=0.05)

                if len(agent.visited_cell_path) > 0 and not agent.is_removed_from_board():
                    braking_res = agent.visited_cell_path[
                                  agent.visited_cell_path_end_of_agent_index:
                                  agent.visited_cell_path_reservation_point_index + 1]
                    for res in braking_res:
                        self.draw_box(res, color=[1, 0, 0], linewidth=3, draw_cell_size=0.8)
                        self.draw_box(res, color=agent_color, linewidth=3, draw_cell_size=0.05)

                    train_resources = agent.visited_cell_path[
                                      agent.visited_cell_path_end_of_agent_index:
                                      agent.visited_cell_path_start_of_agent_index + 1]
                    train_directions = agent.visited_direction_path[
                                       agent.visited_cell_path_end_of_agent_index:
                                       agent.visited_cell_path_start_of_agent_index + 1]
                    for res_idx, res in enumerate(train_resources):
                        self.draw_box(res, color=[0, 0, 0], linewidth=3)
                        self.draw_agent(agent_handle, res, train_directions[res_idx])

    def draw_agent(self, agent_handle, cell, dir_info):
        in_direction = dir_info[0]
        out_direction = dir_info[1]
        delta_dir = (out_direction - in_direction) % 4
        color_idx = agent_handle % self.env_renderer.gl.n_agent_colors
        # when flipping direction at a dead end, use the "out_direction" direction.
        if delta_dir == 2:
            in_direction = out_direction
        pil_zug = self.env_renderer.gl.pil_zug[(in_direction % 4, out_direction % 4, color_idx)]
        self.env_renderer.gl.draw_image_row_col(pil_zug, cell, layer=4)

    def draw_box(self, cell, color, draw_cell_size=0.8, linewidth=3, opacity=255):
        cell_coord_trans0 = np.matmul((cell[0] - 0.5 * draw_cell_size, cell[1] - 0.5 * draw_cell_size),
                                      RenderLocal.row_col_to_xy) + RenderLocal.x_y_half
        cell_coord_trans1 = np.matmul((cell[0] + 0.5 * draw_cell_size, cell[1] + 0.5 * draw_cell_size),
                                      RenderLocal.row_col_to_xy) + RenderLocal.x_y_half

        x0 = cell_coord_trans0[0]
        y0 = cell_coord_trans0[1]
        x1 = cell_coord_trans1[0]
        y1 = cell_coord_trans1[1]
        self.env_renderer.gl.plot([x0, x1, x1, x0, x0], [y0, y0, y1, y1, y0],
                                  color=color, layer=4, opacity=opacity,
                                  linewidth=linewidth)
