import time

import numpy as np
# import all flatland dependance
from flatland.envs.rail_env import RailEnv
from flatland.utils.rendertools import RenderTool, AgentRenderVariant


class FlatlandRenderer:
    def __init__(self, env: RailEnv, show_debug=False, show_agents=True,
                 agent_render_variant: AgentRenderVariant = AgentRenderVariant.BOX_ONLY,
                 cell_size=40, fix_aspect_ration=False):
        self.env = env
        self.show_agents = show_agents
        self.cell_size = cell_size

        if fix_aspect_ration:
            aspect_ratio = self.env.width / self.env.height
            screen_width_scale = np.round(aspect_ratio * cell_size)
            screen_height_scale = np.round(aspect_ratio * cell_size)
            self._create_renderer(show_debug, agent_render_variant=agent_render_variant,
                                  screen_width_scale=screen_width_scale, screen_height_scale=screen_height_scale)
        else:
            self._create_renderer(show_debug, agent_render_variant=agent_render_variant)
        self.update_window_size_for_fix_aspect_ration = fix_aspect_ration

    def set_env(self, env: RailEnv):
        self.env = env
        self.env_renderer.env = self.env
        self.env_renderer.reset()

    def _create_renderer(self, show_debug=False,
                         agent_render_variant=AgentRenderVariant.BOX_ONLY,
                         screen_width_scale=40, screen_height_scale=25):
        self.env_renderer = RenderTool(self.env,
                                       agent_render_variant=agent_render_variant,
                                       show_debug=show_debug,
                                       screen_width=int(np.round(self.env.width * screen_width_scale)),
                                       screen_height=int(np.round(self.env.height * screen_height_scale)))
        self.env_renderer.reset()

    def _disable_background_rendering(self):
        try:
            # this only works when ["PILSVG", "PGL"] rendering is enabled
            empty = self.env_renderer.renderer.gl.scenery_background_white
            self.env_renderer.renderer.gl.scenery = [empty] * len(self.env_renderer.renderer.gl.scenery)
            self.env_renderer.renderer.gl.scenery_d2 = [empty] * len(self.env_renderer.renderer.gl.scenery_d2)
            self.env_renderer.renderer.gl.scenery_d3 = [empty] * len(self.env_renderer.renderer.gl.scenery_d3)
            self.env_renderer.renderer.gl.scenery_water = [empty] * len(self.env_renderer.renderer.gl.scenery_water)
            self.env_renderer.renderer.gl.lBuildings = [empty] * len(self.env_renderer.renderer.gl.lBuildings)
        except:
            pass

    def render(self, show=True, show_observations=True, show_predictions=False,
               disable_background_rendering=False, show_rowcols=False):
        if self.env_renderer is None:
            self._create_renderer()

        if disable_background_rendering:
            self._disable_background_rendering()

        if len(self.env.dev_obs_dict) < 1:
            for h in self.env.get_agent_handles():
                self.env.dev_obs_dict.update({h: []})

        self.env_renderer.render_env(show=show,
                                     show_agents=self.show_agents,  # whether to include agents
                                     show_inactive_agents=False,  # whether to show agents before they start
                                     show_observations=show_observations,
                                     show_predictions=show_predictions,
                                     show_rowcols=show_rowcols,  # label the rows and columns
                                     frames=False,  # frame counter to show (intended since invocation)
                                     episode=None,  # int episode number to show
                                     step=None,  # int step number to show in image
                                     selected_agent=None,  # indicate which agent is "selected" in the editor):
                                     return_image=False)

        if self.update_window_size_for_fix_aspect_ration:

            h, w = self.env_renderer.gl.window.get_size()
            display = self.env_renderer.gl.window.display
            screens = display.get_screens()
            min_h = np.inf
            min_w = np.inf
            margin = 64
            for s in screens:
                min_w = min(min_w, s.width - 2 * margin)
                min_h = min(min_h, s.height - 2 * margin)

            h = min(h, min_h)
            w = min(w, min_w)
            a = h / self.env_renderer.gl.xPx
            if h > w:
                a = w / self.env_renderer.gl.yPx
            width = int(np.round(self.env_renderer.gl.xPx * a))
            height = int(np.round(self.env_renderer.gl.yPx * a))
            self.env_renderer.gl.window.set_size(width, height)
            self.env_renderer.gl.window.set_location(int(np.round((margin + max(0, min_w - width) / 2))),
                                                     int(np.round((margin + max(0, min_h - height) / 2))))

            self.update_window_size_for_fix_aspect_ration = False

    def start_render_loop(self,
                          show_observations=True, show_predictions=False,
                          disable_background_rendering=False, show_rowcols=False):
        while not self.is_closed():
            self.render(show_observations=show_observations,
                        show_predictions=show_predictions,
                        disable_background_rendering=disable_background_rendering,
                        show_rowcols=show_rowcols)
            time.sleep(0.001)

    def is_closed(self) -> bool:
        return self.env_renderer.gl.closed

    def close(self):
        if self.env_renderer is None:
            return
        self.env_renderer.close_window()
        self.env_renderer = None
