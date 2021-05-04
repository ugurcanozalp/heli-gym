import os 

os.environ['PATH'] += os.pathsep + os.path.dirname(os.path.realpath(__file__)) + "/python" + os.pathsep
os.environ['PATH'] += os.pathsep + os.path.dirname(os.path.realpath(__file__)) + "/resources/models" + os.pathsep
