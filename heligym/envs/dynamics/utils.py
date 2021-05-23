import numpy as np

def pi_bound(x):
    return (x + np.pi) % (2 * np.pi) - np.pi