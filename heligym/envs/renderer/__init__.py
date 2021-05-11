import os 
import sys

if sys.platform == 'win32':
    os.environ['PATH'] += os.pathsep + os.path.dirname(os.path.realpath(__file__)) + "/libs"
    os.environ['PATH'] += os.pathsep + os.path.dirname(os.path.realpath(__file__)) + "/python"
    os.environ['PATH'] += os.pathsep + os.path.dirname(os.path.realpath(__file__)) + "/resources/models"

packagedir = os.path.dirname(os.path.abspath(__file__))
os.environ['HELIGYM_PYTHON_DIR'] = os.path.join(packagedir, 'python')
os.environ['HELIGYM_MODEL_DIR'] = os.path.join(packagedir, 'resources/models')    


