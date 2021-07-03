import numpy as np

def pi_bound(x):
    return (x + np.pi) % (2 * np.pi) - np.pi

def cross_product(a, b):
    assert len(a) == len(b) == 3, 'Vectors a, b must be three-dimensional'
    out = np.array(
    		[a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0]]
        )

    return out