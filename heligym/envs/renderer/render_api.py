from . import py_renderer_api as api
import numpy as np
import os
import ctypes

class Renderer(object):
    def __init__(self, w = 800, h = 600, title="heligym-renderer"):
        self.window = api.create_window(w, h, title)
        
        self.camera = self.get_camera()

    def render(self):
        """
            Render the `Renderer's` window.
        """
        api.render(self.window)

    def close(self):
        """
            Close the `Renderer's` window.
        """
        api.close(self.window)

    def is_close(self):
        """
            Check whether the `Renderer's` window is closed or not.
        """
        return api.is_close(self.window)
    
    def terminate(self):
        """
            Terminate OpenGL.
        """
        api.terminate()

    def create_model(self, rel_path = None, 
                vertex_shader_path = "/resources/shaders/vertex_base.vs", 
                fragment_shader_path = "/resources/shaders/frag_base.fs", 
                abs_path = None): 
        """
            Create model with shaders. `rel_path` means relative path of `obj` file. 
            `abs_path` means absolute path of `obj` file. 

            Note : If `abs_path` is used, shaders' path should be also absolute path.
        """
    
        if rel_path:
            return api.create_model(os.path.dirname(os.path.realpath(__file__)) + rel_path, 
                                    os.path.dirname(os.path.realpath(__file__)) + vertex_shader_path,
                                    os.path.dirname(os.path.realpath(__file__)) + fragment_shader_path)
        else:
            return api.create_model(abs_path, 
                                    vertex_shader_path,
                                    fragment_shader_path)

    def add_permanent_object_to_window(self, model):
        """
            Add model to `Renderer's` window as permanent drawable objects.
        """
        api.add_permanent_to_window(self.window, model)

    def add_instantanous_object_to_window(self, model):
        """
            Add model to `Renderer's` window as instantaneous drawable objects.
        """
        api.add_instantaneous_to_window(self.window, model)

    def translate_model(self, model, x, y, z):
        """
            Translate model to x, y and z locations. Locations are in NED (North-East-Down)
            frame. 
        """
        # Because of the vector direction notation difference 
        # we set proper location to proper axis.
        # In Body-frame Y-axis is perpendicular to X-axis and Z-axis is cross
        # product of the axes. On the other hand, in OpenGL (or Computer Graphics)
        # Z-axis is Y-axis is altered. 
        api.translate_model(model, x, -z, y)

    def rotate_model(self, model, phi, theta, psi):
        """
            Rotate model w.r.t Euler angles. 
        """
        # In Body-frame Y-axis is perpendicular to X-axis and Z-axis is cross
        # product of the axes. On the other hand, in OpenGL (or Computer Graphics)
        # Z-axis is Y-axis is altered. 
        api.rotate_model(model, -psi   , 0, 1, 0)
        api.rotate_model(model,  theta , 0, 0, 1)
        api.rotate_model(model,  phi   , 1, 0, 0)

    def scale_model(self, model, x, y, z):
        """
            Scale model to x, y and z axes. Axes are in NED (North-East-Down)
            frame. 
        """
        # Because of the vector direction notation difference 
        # we set proper angle to proper axis.
        # In Body-frame Y-axis is perpendicular to X-axis and Z-axis is cross
        # product of the axes. On the other hand, in OpenGL (or Computer Graphics)
        # Z-axis is Y-axis is altered. 
        api.scale_model(model, x, -z, y)

    def rotate_MR(self, model, phi, theta, psi):
        """
            Rotate Main Rotor of Model with its Euler Angle.
        """
        # In Body-frame Y-axis is perpendicular to X-axis and Z-axis is cross
        # product of the axes. On the other hand, in OpenGL (or Computer Graphics)
        # Z-axis is Y-axis is altered. 
        api.rotate_MR(model, phi, -psi, theta)    

    def rotate_TR(self, model, phi, theta, psi):
        """
            Rotate Tail Rotor of Model with its Euler Angle.
        """
        # In Body-frame Y-axis is perpendicular to X-axis and Z-axis is cross
        # product of the axes. On the other hand, in OpenGL (or Computer Graphics)
        # Z-axis is Y-axis is altered. 
        api.rotate_TR(model, phi, -psi, theta)    

    def get_fps(self):
        """
            Get FPS of `Renderer's` window.
            Return will be float.
        """
        return api.get_fps(self.window)

    def set_fps(self, fps):
        """
            Set FPS of `Renderer's` window.
        """
        api.set_fps(self.window, fps)

    def get_camera(self):
        """
            Get Camera of `Renderer's` window.
        """
        return api.get_camera(self.window)

    def set_camera_pos(self, x, y, z):
        """
            Set Camera position of `Renderer's` window.
        """
        # Because of the vector direction notation difference 
        # we set proper angle to proper axis.
        # In Body-frame Y-axis is perpendicular to X-axis and Z-axis is cross
        # product of the axes. On the other hand, in OpenGL (or Computer Graphics)
        # Z-axis is Y-axis is altered. 
        api.set_camera_pos(self.camera, x, -z, y)

    def get_camera_pos(self):
        """
            Get Camera position of `Renderer's` window.
            Return will be float array.
        """
        return api.get_camera_pos(self.camera)

    def coord_from_graphics_to_ned(self, x, y, z):
        """
            Convert OpenGL coordinates to North, East and Down (NED)
            frame coordinates.
        """
        return x, z, -y

    def is_visible(self):
        """
            Check whether `Renderer's` window is visible or not. 
            Return will be boolean.
        """
        return api.is_visible(self.window)

    def hide_window(self):
        """
            Hide `Renderer's` window.
        """
        api.hide_window(self.window)

    def show_window(self):
        """
            Show `Renderer's` window.
        """
        api.show_window(self.window)

    def add_guiOBS(self, str, val):
        """
            Add guiOBS text. 
            Format should be in printf-style format.
            \n * str should be encoded by bytes with `utf-8`.
            \n * There should be 1 val for printing and vals should be numpy array.
            
            \n Ex 1:
            >>> string_to_gui = [bytes('hello', 'utf-8')]
            >>> add_guiOBS(string_to_gui, 0) # 0 for non values

            \n Ex 2:
            >>> string_to_gui = [bytes('the value %3.2f', 'utf-8')]
            >>> value_to_gui = [5.2]
            >>> add_guiOBS(string_to_gui, value_to_gui)

            str and val are should be list.
        """
        api.add_guiOBS(self.window, str, val)

    def set_guiOBS(self, str, val):
        """
            Set guiOBS text.
            str and val are should be list.
        """
        api.set_guiOBS(self.window, str, val)

    
