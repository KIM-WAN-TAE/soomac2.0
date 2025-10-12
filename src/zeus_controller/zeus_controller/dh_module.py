import numpy as np

def dh_transform(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    
    T = np.array([[ ct, -st*ca,  st*sa, a*ct],
                  [ st,  ct*ca, -ct*sa, a*st],
                  [  0,      sa,     ca,    d],
                  [  0,       0,      0,    1]])
    return T

def fk(dh_params):
    T = np.eye(4)
    for params in dh_params:
        T = T @ dh_transform(params['theta'], params['d'], params['a'], params['alpha'])
    return T

def pos_as_T(P):
    T = np.eye(4)
    T[:3, 3] = np.asarray(P, float).reshape(3)
    return T

def rot_to_euler_zyx(R):
    r20 = R[2,0]
    if abs(r20) < 1.0 - 1e-9:
        ry = np.arcsin(-r20)
        rz = np.arctan2(R[1,0], R[0,0])
        rx = np.arctan2(R[2,1], R[2,2])
    else:
        ry = np.pi/2 if r20 <= -1.0 else -np.pi/2
        rz = 0.0
        rx = np.arctan2(-R[0,1], R[1,1])
    return np.rad2deg(rz), np.rad2deg(ry), np.rad2deg(rx)