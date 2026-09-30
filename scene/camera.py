"""World-space camera. Pixel axes: right/down; camera position is view center."""
import numpy as np


class Camera:
    def __init__(self, x=0.0, y=0.0, zoom=1.0):
        self.x, self.y = float(x), float(y)
        self.zoom = zoom

    @property
    def zoom(self):
        return self._zoom

    @zoom.setter
    def zoom(self, value):
        if not np.isfinite(value) or value <= 0:
            raise ValueError('Camera zoom must be finite and positive')
        self._zoom = float(value)

    @property
    def position(self):
        return np.array([self.x, self.y])

    @position.setter
    def position(self, value):
        self.x, self.y = map(float, value)

    def world_to_screen(self, world_pos, screen_size, depth=1.0):
        # step5 convention: distance from camera multiplied by layer depth.
        return np.asarray(screen_size)/2 + (np.asarray(world_pos)-self.position)*depth*self.zoom

    def screen_to_world(self, screen_pos, screen_size, depth=1.0):
        if depth <= 0:
            raise ValueError('Parallax depth must be positive')
        return self.position + (np.asarray(screen_pos)-np.asarray(screen_size)/2)/(depth*self.zoom)

    def set_focus(self, world_pos, screen_size=None):
        self.position = world_pos
