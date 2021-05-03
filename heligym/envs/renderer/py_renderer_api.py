import os
import sys
import ctypes
import numpy as np


if sys.platform == 'win32':
    lib = ctypes.cdll.LoadLibrary('Heligym.dll')
elif sys.platform == 'Linux':
    lib = ctypes.cdll.LoadLibrary('libHeligym.so')

###################################################################################
def _to_encode(str_to_encode, encode_type = 'utf-8'):
    return str_to_encode.encode(encode_type)

###################################################################################
lib.create_shader.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
lib.create_shader.restype = ctypes.c_void_p

def create_shader(window, path):
    lib.create_shader(window, ctypes.c_char_p(_to_encode(path)))

###################################################################################
lib.create_window.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_char_p]
lib.create_window.restype = ctypes.c_void_p

def create_window(w, h, title):
    return lib.create_window(w, h, ctypes.c_char_p(_to_encode(title)))  

###################################################################################
lib.render.argtypes = [ctypes.c_void_p]
lib.render.restype = ctypes.c_void_p

def render(window):
    lib.render(window)

###################################################################################
lib.close.argtypes = [ctypes.c_void_p]
lib.close.restype = ctypes.c_void_p

def close(window):
    lib.close(window)

###################################################################################
lib.is_close.argtypes = [ctypes.c_void_p]
lib.is_close.restype = ctypes.c_bool

def is_close(window):
    return lib.is_close(window)  

###################################################################################
lib.terminate.argtypes = [ctypes.c_void_p]
lib.terminate.restype = ctypes.c_void_p

def terminate():
    lib.terminate()

###################################################################################
lib.create_model.argtypes = [ctypes.c_char_p]
lib.create_model.restype = ctypes.c_void_p

def create_model(object_path):
    return lib.create_model(ctypes.c_char_p(_to_encode(object_path)))

###################################################################################
lib.add_permanent_to_window.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
lib.add_permanent_to_window.restypes = ctypes.c_void_p

def add_permanent_to_window(window, model):
    lib.add_permanent_to_window(window, model)

###################################################################################
lib.add_instantaneous_to_window.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
lib.add_instantaneous_to_window.restypes = ctypes.c_void_p

def add_instantaneous_to_window(window, model):
    lib.add_instantaneous_to_window(window, model)
    
###################################################################################
lib.translate_model.argtypes = [ctypes.c_void_p, ctypes.c_float, 
                                ctypes.c_float, ctypes.c_float]
lib.translate_model.restype = ctypes.c_void_p

def translate_model(model, x, y, z):
    lib.translate_model(model, x, y, z)

###################################################################################
lib.rotate_model.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float, 
                                ctypes.c_float, ctypes.c_float]
lib.rotate_model.restype = ctypes.c_void_p

def rotate_model(model, angle, x, y, z):
    lib.rotate_model(model, angle, x, y, z)

###################################################################################
lib.scale_model.argtypes = [ctypes.c_void_p, ctypes.c_float, 
                                ctypes.c_float, ctypes.c_float]
lib.scale_model.restype = ctypes.c_void_p

def scale_model(model, x, y, z):
    lib.scale_model(model, x, y, z)

###################################################################################
lib.get_fps.argtypes = [ctypes.c_void_p]
lib.get_fps.restype = ctypes.c_float

def get_fps(window):
    return np.round(lib.get_fps(window), 2)

###################################################################################
lib.get_camera.argtypes = [ctypes.c_void_p]
lib.get_camera.restype = ctypes.c_void_p

def get_camera(window):
    return lib.get_camera(window)

###################################################################################
lib.set_camera_pos.argtypes = [ctypes.c_void_p, ctypes.c_float,
                                ctypes.c_float, ctypes.c_float]
lib.set_camera_pos.restype = ctypes.c_void_p

def set_camera_pos(window, x, y, z):
    lib.set_camera_pos(window, x, y, z)

###################################################################################
lib.get_camera_pos.argtypes = [ctypes.c_void_p]
lib.get_camera_pos.restype = ctypes.POINTER(ctypes.c_float)

def get_camera_pos(camera):
    return lib.get_camera_pos(camera)

###################################################################################

