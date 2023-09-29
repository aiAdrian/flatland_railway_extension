import pyglet as pgl
from flatland.utils.graphics_pgl import PGLGL

_window = None
reuse_window = True


def get_window(width, height):
    if reuse_window:
        global _window
        if _window is None:
            _window = pgl.window.Window(
                resizable=True, vsync=False, width=width, height=height
            )
        else:
            _window.set_size(width, height)
        return _window
    else:
        return pgl.window.Window(
            resizable=True, vsync=False, width=width, height=height
        )


# hack PGLGL.show() to avoid image resize and put event loop behind frame render.
def show(self, block=False, from_event=False):
    if not self.window_open:
        self.open_window()

    if self.close_requested:
        if not self.closed:
            self.close_window()
        return

    # tStart = time.time()

    pil_img = self.alpha_composite_layers()

    # convert our PIL image to pyglet:
    bytes_image = pil_img.tobytes()
    pgl_image = pgl.image.ImageData(
        pil_img.width,
        pil_img.height,
        # self.window.width, self.window.height,
        "RGBA",
        bytes_image,
        pitch=-pil_img.width * 4,
    )

    pgl_image.blit(0, 0)
    self._processEvents()
    # tEnd = time.time()
    # print("show time: ", tEnd - tStart)


# hack PGLGL.open_window().
# set its window size to (self.widthPx, self.heightPx)
# alow to reuse a global window object
def open_window(self):
    # print("open_window - pyglet")
    assert self.window_open is False, "Window is already open!"
    self.window = get_window(width=self.widthPx, height=self.heightPx)
    # self.__class__.window.title("Flatland")
    # self.__class__.window.configure(background='grey')
    self.window_open = True

    @self.window.event
    def on_draw():
        # print("pyglet draw event")
        self.window.clear()
        self.show(from_event=True)
        # print("pyglet draw event done")

    @self.window.event
    def on_resize(width, height):
        # print(f"The window was resized to {width}, {height}")
        self.show(from_event=True)
        self.window.dispatch_event("on_draw")
        # print("pyglet resize event done")

    @self.window.event
    def on_close():
        self.close_requested = True


# Overwrite PGLGL.open_window() and PGLGL.show() by our customized ones
PGLGL.show = show
PGLGL.open_window = open_window


def call_flatland_pglgl_patch():
    print('call_flatland_pglgl_patch')
