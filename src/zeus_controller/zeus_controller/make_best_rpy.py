import numpy as np

def wrap180(a_deg: float) -> float:
    a = (a_deg + 180.0) % 360.0
    return a - 360.0 if a > 180.0 else a

def Rz(a): 
    c,s=np.cos(a),np.sin(a)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def Ry(a): 
    c,s=np.cos(a),np.sin(a)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def Rx(a): 
    c,s=np.cos(a),np.sin(a)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]])

def euler_zyx_to_R(yaw_deg, pitch_deg, roll_deg):
    z,y,x = np.deg2rad([yaw_deg, pitch_deg, roll_deg])
    return Rz(z) @ Ry(y) @ Rx(x)

def geodesic_angle(Ra, Rb):
    t = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return np.arccos(np.clip(t, -1.0, 1.0))

def yaw_flip_zyx_deg(yaw, pitch, roll):
    return wrap180(yaw + 180.0), -pitch, wrap180(roll + 180.0)

def pick_best_rpy(yaw, pitch, roll, R_now=None):
    # 2개 후보: 원본, 동치(뒤집기)
    cand = [(yaw, pitch, roll),
            yaw_flip_zyx_deg(yaw, pitch, roll)]
    if R_now is None:
        # R_now가 없으면 |yaw| 최소인 후보 선택
        costs = [abs(wrap180(c[0])) for c in cand]
        return cand[int(np.argmin(costs))]
    # 각거리 최소 후보 선택
    best = None; best_cost = 1e9
    for y,p,r in cand:
        Rc = euler_zyx_to_R(y,p,r)
        cost = geodesic_angle(R_now, Rc)
        if cost < best_cost:
            best_cost, best = cost, (y,p,r)
    return best
