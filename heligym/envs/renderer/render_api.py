from . import py_renderer_api as api
import numpy as np
import os

class Renderer(object):
    def __init__(self, w = 800, h = 600, title="heligym-renderer"):
        self.window = api.create_window(w, h, title)
        api.create_shader(self.window, os.path.dirname(os.path.realpath(__file__)) + "/resources/shaders")
        self.camera = self.get_camera()

    def render(self):
        api.render(self.window)

    def close(self):
        api.close(self.window)

    def is_close(self):
        return api.is_close(self.window)
    
    def terminate(self):
        api.terminate()

    def create_model(self, rel_path = None, abs_path = None): 
        if rel_path:
            return api.create_model(os.path.dirname(os.path.realpath(__file__)) + rel_path)
        else:
            return api.create_model(abs_path)

    def add_permanent_object_to_window(self, model):
        api.add_permanent_to_window(self.window, model)

    def add_instantanous_object_to_window(self, model):
        api.add_instantaneous_to_window(self.window, model)

    def translate_model(self, model, x, y, z):
        # Because of the vector direction notation difference 
        # we set proper angle to proper axis.
        # In Body-frame Y-axis is perpendicular to X-axis and Z-axis is cross
        # product of the axes. On the other hand, in OpenGL (or Computer Graphics)
        # Z-axis is Y-axis is altered. 
        api.translate_model(model, x, -z, y)

    def rotate_model(self, model, phi, theta, psi):
        # we need to divide the angle to FPS to have proper angle.
        # also because of the vector direction notation difference 
        # we set proper angle to proper axis.

        # In Body-frame Y-axis is perpendicular to X-axis and Z-axis is cross
        # product of the axes. On the other hand, in OpenGL (or Computer Graphics)
        # Z-axis is Y-axis is altered. 

        api.rotate_model(model, -psi   / 180 * np.pi, 0, 1, 0)
        api.rotate_model(model,  theta / 180 * np.pi, 0, 0, 1)
        api.rotate_model(model,  phi   / 180 * np.pi, 1, 0, 0)

    def scale_model(self, model, x, y, z):
        # Because of the vector direction notation difference 
        # we set proper angle to proper axis.
        # In Body-frame Y-axis is perpendicular to X-axis and Z-axis is cross
        # product of the axes. On the other hand, in OpenGL (or Computer Graphics)
        # Z-axis is Y-axis is altered. 
        api.scale_model(model, x, -z, y)

    def get_fps(self):
        return api.get_fps(self.window)

    def set_fps(self, fps):
        api.set_fps(self.window, fps)

    def get_camera(self):
        return api.get_camera(self.window)

    def set_camera_pos(self, x, y, z):
        # Because of the vector direction notation difference 
        # we set proper angle to proper axis.
        # In Body-frame Y-axis is perpendicular to X-axis and Z-axis is cross
        # product of the axes. On the other hand, in OpenGL (or Computer Graphics)
        # Z-axis is Y-axis is altered. 
        api.set_camera_pos(self.camera, x, -z, y)

    def get_camera_pos(self):
        return api.get_camera_pos(self.camera)

    def coord_from_graphics_to_ned(self, x, y, z):
        """
            Convert OpenGL coordinates to North, East and Down (NED)
            frame coordinates.
        """
        return x, z, -y
