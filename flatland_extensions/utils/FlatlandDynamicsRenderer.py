from typing import Union

import numpy as np
from flatland.envs.rail_env import RailEnv
from flatland.utils.rendertools import AgentRenderVariant
from flatland.utils.rendertools import RenderLocal

from flatland_extensions.environment_extensions import FlatlandResourceAllocator
from flatland_extensions.utils.FlatlandRenderer import FlatlandRenderer


class FlatlandDynamicsRenderer(FlatlandRenderer):
    def __init__(self, env: RailEnv, show_debug=False, show_agents=True,
                 agent_render_variant: AgentRenderVariant = AgentRenderVariant.BOX_ONLY):
        super(FlatlandDynamicsRenderer, self).__init__(env, show_debug, show_agents, agent_render_variant)
        self.flatland_resource_allocator: Union[FlatlandResourceAllocator, None] = None

    def set_flatland_resource_allocator(self, flatland_resource_allocator: FlatlandResourceAllocator):
        self.flatland_resource_allocator = flatland_resource_allocator

    def render(self, show=True, show_observations=True, show_predictions=False, disable_background_rendering=False):
        super(FlatlandDynamicsRenderer, self).render(show, show_observations, show_predictions,
                                                     disable_background_rendering)

        if self.flatland_resource_allocator is not None:
            self.env_renderer.renderer.gl.clear_layer(4)
            for row in range(self.env.height):
                for col in range(self.env.width):
                    res = (row, col)
                    mft = self.flatland_resource_allocator.get_minimal_free_time()
                    if mft > 0:
                        dt = self.flatland_resource_allocator.get_resources_free_time(res) / mft
                        if dt < 1.0:
                            self.draw_box(res, color=[dt, 0.8 + 0.2 * dt, dt])

            for agent_handle, agent in enumerate(self.env.agents):
                allocated_resource = self.flatland_resource_allocator.get_assigned_resources(agent_handle=agent_handle)
                for res in allocated_resource:
                    self.draw_box(res, color=[1.0, 0.8, 0.0])

                if len(agent.visited_cell_path) > 0:
                    for res in agent.visited_cell_path[
                               agent.visited_cell_path_end_of_agent_index:
                               agent.visited_cell_path_reservation_point_index + 1]:
                        self.draw_box(res, color=[1, 0, 0])

                    for res in agent.visited_cell_path[
                               agent.visited_cell_path_end_of_agent_index:
                               agent.visited_cell_path_start_of_agent_index + 1]:
                        self.draw_box(res, color=[0, 0, 0])

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
