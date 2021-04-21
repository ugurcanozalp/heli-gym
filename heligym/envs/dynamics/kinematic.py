import numpy as np

def euler_to_rotmat(euler):
    sin = np.sin(euler)
    cos = np.cos(euler)

    psi_rot     =   [ [ cos[2]  ,   -sin[2] ,   0   ],
                      [ sin[2]  ,   cos[2]  ,   0   ], 
                      [ 0       ,   0       ,   1   ] ]

    theta_rot   =   [ [ cos[1]  ,   0   ,   -sin[1] ],
                      [ 0       ,   1   ,   0       ], 
                      [ sin[1]  ,   0   ,   cos[2]  ] ]

    phi_rot     =   [ [ 1   ,   0       ,   0       ],
                      [ 0   ,   cos[0]  ,   sin[0]  ],
                      [ 0   ,   -sin[0] ,   cos[0]  ] ]

    rot_mat = np.array(phi_rot)@np.array(theta_rot)@np.array(psi_rot)
    return rot_mat

def pqr_to_eulerdot_mat(euler):
    sin = np.sin(euler)
    cos = np.cos(euler)
    mat         =   [ [ 1   , sin[0]*sin[1]/cos[1]  ,   cos[0]*sin[1]/cos[1]  ],
                      [ 0   , cos[0]                ,   sin[0]                ], 
                      [ 0   , sin[0]/cos[1]         ,   cos[0]/cos[1]         ] ]

    return np.array(mat)