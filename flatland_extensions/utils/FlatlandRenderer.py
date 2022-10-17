import time

# import all flatland dependance
from flatland.envs.rail_env import RailEnv
from flatland.utils.rendertools import RenderTool, AgentRenderVariant


class FlatlandRenderer:
    def __init__(self, env: RailEnv, show_debug=False, show_agents=True):
        self.env = env
        self.show_agents = show_agents
        self._create_renderer(show_debug)

    def set_env(self, env: RailEnv):
        self.env = env
        self.env_renderer.env = self.env
        self.env_renderer.reset()

    def _create_renderer(self, show_debug):
        self.env_renderer = RenderTool(self.env,
                                       agent_render_variant=AgentRenderVariant.BOX_ONLY,
                                       show_debug=show_debug,
                                       screen_width=self.env.width * 40,
                                       screen_height=self.env.height * 25)
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
