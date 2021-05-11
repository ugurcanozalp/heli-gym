#!/usr/bin/python3
import os 
import sys

packagedir = os.path.dirname(os.path.abspath(__file__))
os.environ['HELIGYM_LIBS_DIR'] = os.path.join(packagedir, 'libs')
os.environ['HELIGYM_PYTHON_DIR'] = os.path.join(packagedir, 'python')
os.environ['HELIGYM_MODEL_DIR'] = os.path.join(packagedir, 'resources/models')    

if sys.platform == 'win32':
    os.environ['PATH'] += os.pathsep + os.environ['HELIGYM_LIBS_DIR']
    os.environ['PATH'] += os.pathsep + os.environ['HELIGYM_PYTHON_DIR']
    os.environ['PATH'] += os.pathsep + os.environ['HELIGYM_MODEL_DIR']

elif sys.platform == 'linux':
    if 'LD_LIBRARY_PATH' not in os.environ:
        os.environ['LD_LIBRARY_PATH']  = os.pathsep + os.environ['HELIGYM_LIBS_DIR']
        os.environ['LD_LIBRARY_PATH'] += os.pathsep + os.environ['HELIGYM_PYTHON_DIR']
        os.environ['LD_LIBRARY_PATH'] += os.pathsep + os.environ['HELIGYM_MODEL_DIR']
        try:
            os.execv(sys.executable, [sys.executable] + sys.argv)
        except Exception as e:
            print(e)
            sys.exit(1)

