import os
import sys
import ctypes
import numpy as np

"""
    Load shadered library w.r.t OS.
"""
if sys.platform == 'win32':
    lib = ctypes.cdll.LoadLibrary(os.environ['HELIGYM_PYTHON_DIR'] + '/Heligym.dll')
elif sys.platform == 'linux':
    lib = ctypes.cdll.LoadLibrary(os.environ['HELIGYM_PYTHON_DIR'] + '/libHeligym.so')

###################################################################################
def _to_encode(str_to_encode, encode_type = 'utf-8'):
    """
        Encoding the string into `encode_type` for char pointer of ctypes.
    """
    return str_to_encode.encode(encode_type)

###################################################################################
lib.create_window.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_char_p]
lib.create_window.restype = ctypes.c_void_p

def create_window(w, h, title):
    """
        Create window with parameters which are
        >>> w     : Width of window
        >>> h     : Height of window
        >>> title : Title of window.
    """
    return lib.create_window(w, h, ctypes.c_char_p(_to_encode(title)))  

###################################################################################
lib.render.argtypes = [ctypes.c_void_p]
lib.render.restype = ctypes.c_void_p

def render(window):
    """
        Render window.
    """
    lib.render(window)

###################################################################################
lib.close.argtypes = [ctypes.c_void_p]
lib.close.restype = ctypes.c_void_p

def close(window):
    """
        Close window.
    """
    lib.close(window)

###################################################################################
lib.is_close.argtypes = [ctypes.c_void_p]
lib.is_close.restype = ctypes.c_bool

def is_close(window):
    """
        Check whether window is closed or not.
    """
    return lib.is_close(window)  

###################################################################################
lib.terminate.argtypes = []
lib.terminate.restype = ctypes.c_void_p

def terminate():
    """
        Terminate the OpenGL.
    """
    lib.terminate()

###################################################################################
lib.create_model.argtypes = [ctypes.c_char_p, ctypes.c_char_p, ctypes.c_char_p]
lib.create_model.restype = ctypes.c_void_p

def create_model(object_path, vertex_shader_path, fragment_shader_path):
    """
        Create model with its shaders.
    """
    return lib.create_model(ctypes.c_char_p(_to_encode(object_path)),
                            ctypes.c_char_p(_to_encode(vertex_shader_path)),
                            ctypes.c_char_p(_to_encode(fragment_shader_path)))

###################################################################################
lib.add_permanent_to_window.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
lib.add_permanent_to_window.restypes = ctypes.c_void_p

def add_permanent_to_window(window, model):
    """
        Add model as permanent drawables object to window.
    """
    lib.add_permanent_to_window(window, model)

###################################################################################
lib.add_instantaneous_to_window.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
lib.add_instantaneous_to_window.restypes = ctypes.c_void_p

def add_instantaneous_to_window(window, model):
    """ 
        Add model as instantaneous drawables object to window.
    """
    lib.add_instantaneous_to_window(window, model)
    
###################################################################################
lib.translate_model.argtypes = [ctypes.c_void_p, ctypes.c_float, 
                                ctypes.c_float, ctypes.c_float]
lib.translate_model.restype = ctypes.c_void_p

def translate_model(model, x, y, z):
    """
        Translate model to locations. Locations will be in OpenGL coordinates.
    """
    lib.translate_model(model, x, y, z)

###################################################################################
lib.rotate_model.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float, 
                                ctypes.c_float, ctypes.c_float]
lib.rotate_model.restype = ctypes.c_void_p

def rotate_model(model, angle, x, y, z):
    """
        Rotate model to angle. From will be an rotation around arbitrary angle.
    """
    lib.rotate_model(model, angle, x, y, z)

###################################################################################
lib.scale_model.argtypes = [ctypes.c_void_p, ctypes.c_float, 
                                ctypes.c_float, ctypes.c_float]
lib.scale_model.restype = ctypes.c_void_p

def scale_model(model, x, y, z):
    """
        Scale model to the ratios for each axis.
    """
    lib.scale_model(model, x, y, z)

###################################################################################
lib.get_fps.argtypes = [ctypes.c_void_p]
lib.get_fps.restype = ctypes.c_float

def get_fps(window):
    """
        Get FPS from the window.
    """
    return np.round(lib.get_fps(window), 2)

###################################################################################
lib.set_fps.argtypes = [ctypes.c_void_p, ctypes.c_float]
lib.set_fps.restype = ctypes.c_void_p

def set_fps(window, fps):
    """
        Set FPS of the window.
    """
    lib.set_fps(window, fps)

###################################################################################
lib.get_camera.argtypes = [ctypes.c_void_p]
lib.get_camera.restype = ctypes.c_void_p

def get_camera(window):
    """
        Get Camera pointer of the window.
    """
    return lib.get_camera(window)

###################################################################################
lib.set_camera_pos.argtypes = [ctypes.c_void_p, ctypes.c_float,
                                ctypes.c_float, ctypes.c_float]
lib.set_camera_pos.restype = ctypes.c_void_p

def set_camera_pos(camera, x, y, z):
    """
        Set Camera position of each axis.
    """
    lib.set_camera_pos(camera, x, y, z)

###################################################################################
lib.get_camera_pos.argtypes = [ctypes.c_void_p]
lib.get_camera_pos.restype = ctypes.POINTER(ctypes.c_float)

def get_camera_pos(camera):
    """
        Get camera position as float array.
    """
    return lib.get_camera_pos(camera)

###################################################################################
lib.is_visible.argtypes = [ctypes.c_void_p]
lib.is_visible.restype = ctypes.c_bool

def is_visible(window):
    """
        Check whether the window is visible or not.
    """
    return lib.is_visible(window)

###################################################################################
lib.hide_window.argtypes = [ctypes.c_void_p]
lib.hide_window.restype = ctypes.c_void_p

def hide_window(window):
    """
        Hide the window.
    """
    lib.hide_window(window)

###################################################################################
lib.show_window.argtypes = [ctypes.c_void_p]
lib.show_window.restype = ctypes.c_void_p

def show_window(window):
    """
        Show the window.
    """
    lib.show_window(window)

###################################################################################
lib.create_guiTextVector.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float]
lib.create_guiTextVector.restype = ctypes.c_int

def create_guiText(window, title, pos, size):
    """
        Create guiText vector for guiTextSection.        
    """
    assert type(pos) == tuple and len(pos) == 2, 'Make sure that position of guiText should be tuple and has 2 element.'
    assert type(size) == tuple and len(size) == 2, 'Make sure that size of guiText should be tuple and has 2 element.'
    return lib.create_guiTextVector(window, ctypes.c_char_p(_to_encode(title)), pos[0], pos[1], size[0], size[1])

###################################################################################
lib.add_guiText.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_char_p), np.ctypeslib.ndpointer(dtype=np.float32, flags='C_CONTIGUOUS')]
lib.add_guiText.restype = ctypes.c_void_p

def add_guiText(window, guiText, strn, val):
    """
        Add the str and val to guiText on window.
    """
    assert type(strn) == list, 'Please make sure that `str` values in form of list.'
    for ind, item in enumerate(strn):
        strn[ind] = bytes(str(item), 'utf-8')
    strn_arr = (ctypes.c_char_p * (len(strn)))()    
    strn_arr[:] = strn

    if type(val) != np.ndarray:
        val = np.array(val)

    lib.add_guiText(window, guiText, len(strn), strn_arr, val.astype(np.float32) )

###################################################################################
lib.set_guiText.argtypes = [ctypes.c_void_p, ctypes.c_int, np.ctypeslib.ndpointer(dtype=np.float32, flags='C_CONTIGUOUS')]
lib.set_guiText.restype = ctypes.c_void_p

def set_guiText(window, guiText, str, val):
    """
        Set the str and val to guiText on window.
    """
    lib.set_guiText(window, guiText, val.astype(np.float32) )

###################################################################################
lib.rotate_MR.argtypes = [ctypes.c_void_p, ctypes.c_float, 
                                ctypes.c_float, ctypes.c_float]
lib.rotate_MR.restype = ctypes.c_void_p

def rotate_MR(model, x, y, z):
    """
        Rotate the model Main Rotor with each angle which are in radians.
    """
    lib.rotate_MR(model, x, y, z)

###################################################################################
lib.rotate_TR.argtypes = [ctypes.c_void_p, ctypes.c_float, 
                                ctypes.c_float, ctypes.c_float]
lib.rotate_TR.restype = ctypes.c_void_p

def rotate_TR(model, x, y, z):
    """
        Rotate the model Tail Rotor with each angle which are in radians.
    """
    lib.rotate_TR(model, x, y, z)

