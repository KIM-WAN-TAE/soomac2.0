# humanoid_arm_ptp_anim_dh.py
import warnings
warnings.filterwarnings(
    "ignore",
    message="Link .* is of type 'fixed' but set as active in the active_links_mask",
    category=UserWarning
)

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from ikpy.chain import Chain
from ikpy.link import OriginLink, DHLink

# =========================
# 1) 체인 구성 (표준 DH, m)
# =========================
def build_chain():
    chain = Chain(name="humanoid_right_arm", links=[
        OriginLink(),  # base (fixed)
        # name,  theta_off,      d,       a,       alpha
        DHLink("j1", theta=-np.pi/2, d=0.0,    a=0.0,    alpha=+np.pi/2, bounds=(-np.pi, np.pi)),
        DHLink("j2", theta=+np.pi/2, d=0.0,    a=0.0,    alpha=-np.pi/2, bounds=(-np.pi, np.pi)),
        DHLink("j3", theta=+np.pi/2, d=-0.100, a=0.0,    alpha=+np.pi/2, bounds=(-np.pi, np.pi)),
        DHLink("j4", theta=0.0,      d=0.0,    a=0.0,    alpha=+np.pi/2, bounds=(-np.pi, np.pi)),
        DHLink("j5", theta=0.0,      d=+0.100, a=0.0,    alpha=+np.pi/2, bounds=(-np.pi, np.pi)),
        DHLink("j6", theta=+np.pi/2, d=0.0,    a=+0.100, alpha=0.0,      bounds=(-np.pi, np.pi)),
    ])
    # Base 비활성 마스크(경고 억제는 위 filter로 처리, 그래도 일관성 유지)
    try:
        chain.active_links_mask = [False] + [True] * (len(chain.links) - 1)
    except Exception:
        pass
    return chain

# =========================
# 2) RPY → 회전행렬
# =========================
def rpy_to_R(roll, pitch, yaw):
    cr, sr = np.cos(roll),  np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw),   np.sin(yaw)
    Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    return Rz @ Ry @ Rx  # Z-Y-X(RPY)

# =========================
# 3) IK (구/신 버전 호환: frame 또는 position만)
# =========================
def solve_ik(chain, pos, R=None, q_seed=None, use_orientation=True):
    """
    pos: (3,) [m], R: (3,3) or None, q_seed: (6,)
    반환: (6,)
    """
    pos = np.asarray(pos, dtype=float)
    seed_all = [0.0] + (list(q_seed) if q_seed is not None else [0.0]*6)

    # 1) 자세 포함: 4x4 프레임 이용 (구버전 미지원 시 except)
    if use_orientation and R is not None:
        T = np.eye(4)
        T[:3,:3] = R
        T[:3, 3] = pos
        try:
            q_all = chain.inverse_kinematics_frame(target=T, initial_position=seed_all)
            return np.asarray(q_all[1:])
        except Exception:
            pass  # 폴백

    # 2) 위치만
    try:
        q_all = chain.inverse_kinematics(target_position=pos, initial_position=seed_all)
        return np.asarray(q_all[1:])
    except Exception:
        q_all = chain.inverse_kinematics(target_position=pos)
        return np.asarray(q_all[1:])

# =========================
# 4) PTP (직선 보간 + 실패시 폴백)
# =========================
def plan_ptp(chain, start_xyz, end_xyz,
             start_rpy=(0,0,0), end_rpy=None,
             steps=60, q0=None, use_orientation=True):
    if end_rpy is None:
        end_rpy = start_rpy
    start_xyz = np.asarray(start_xyz, dtype=float)
    end_xyz   = np.asarray(end_xyz,   dtype=float)
    start_rpy = np.asarray(start_rpy, dtype=float)
    end_rpy   = np.asarray(end_rpy,   dtype=float)

    traj = []
    seed = q0
    for s in np.linspace(0.0, 1.0, steps):
        p   = (1-s)*start_xyz + s*end_xyz
        rpy = (1-s)*start_rpy + s*end_rpy
        R   = rpy_to_R(*rpy)

        q = solve_ik(chain, p, R, q_seed=seed, use_orientation=use_orientation)
        if not np.all(np.isfinite(q)):
            q = solve_ik(chain, p, None, q_seed=seed, use_orientation=False)
        traj.append(q)
        seed = q
    return np.vstack(traj)

# =========================
# 5) 표준 DH로 모든 관절 좌표 계산(ikpy 내부 API 불문)
# =========================
# 질문에 주신 DH(θ오프셋 포함, 단위 m) – build_chain과 동일
DH_TABLE = [
    (-np.pi/2,  0.0,    0.0,   +np.pi/2),  # j1
    (+np.pi/2,  0.0,    0.0,   -np.pi/2),  # j2
    (+np.pi/2, -0.100,  0.0,   +np.pi/2),  # j3
    (0.0,       0.0,    0.0,   +np.pi/2),  # j4
    (0.0,      +0.100,  0.0,   +np.pi/2),  # j5
    (+np.pi/2,  0.0,   +0.100,  0.0),      # j6 (EE)
]

def dh_T(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0 ,     sa,     ca,    d],
        [0 ,      0,      0,    1],
    ], dtype=float)

def joint_positions_DH(q):
    """
    표준 DH 누적 FK로 [base, j1, j2, ..., j6] 좌표 반환 (7x3)
    q: (6,) 라디안
    """
    T = np.eye(4)
    P = [T[:3,3].copy()]  # base at origin
    for i,(th0,d,a,al) in enumerate(DH_TABLE):
        T = T @ dh_T(th0 + q[i], d, a, al)
        P.append(T[:3,3].copy())
    return np.vstack(P)

# =========================
# 6) 3D 애니메이션
# =========================
def animate_robot_trajectory(chain, q_traj,
                             duration=5.0, repeat=True,
                             start_pos=None, end_pos=None):
    fig = plt.figure(figsize=(9, 8))
    ax  = fig.add_subplot(111, projection='3d')

    all_positions = np.array([joint_positions_DH(q) for q in q_traj])  # (N,7,3)

    # 축 범위/비율
    all_pts = all_positions.reshape(-1, 3)
    if start_pos is not None: all_pts = np.vstack([all_pts, np.asarray(start_pos)])
    if end_pos   is not None: all_pts = np.vstack([all_pts,   np.asarray(end_pos)])

    xyz_min = all_pts.min(axis=0); xyz_max = all_pts.max(axis=0)
    span = (xyz_max - xyz_min).max(); margin = 0.05 * max(span, 1e-6)
    center = 0.5 * (xyz_max + xyz_min)
    ax.set_xlim([center[0]-span/2-margin, center[0]+span/2+margin])
    ax.set_ylim([center[1]-span/2-margin, center[1]+span/2+margin])
    ax.set_zlim([center[2]-span/2-margin, center[2]+span/2+margin])
    try: ax.set_box_aspect([1,1,1])  # 동일 스케일
    except Exception: pass

    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_title('Humanoid Arm – PTP IK Animation')
    ax.view_init(elev=25, azim=-60)

    line,   = ax.plot([], [], [], 'b-', linewidth=3, label='Links')
    joints, = ax.plot([], [], [], 'ko', markersize=5, label='Joints')
    trail,  = ax.plot([], [], [], 'g--', alpha=0.6, label='EE trail')

    if start_pos is not None: ax.plot([start_pos[0]],[start_pos[1]],[start_pos[2]],'go',markersize=8,label='Start')
    if end_pos   is not None: ax.plot([end_pos[0]],[end_pos[1]],[end_pos[2]],'ro',markersize=8,label='End')
    ax.legend(loc='upper left')

    def set_3d(obj, X, Y, Z):
        try: obj.set_data_3d(X, Y, Z)
        except AttributeError:
            obj.set_data(X, Y); obj.set_3dproperties(Z)

    def animate(frame):
        P = all_positions[frame]  # (7,3)
        set_3d(line,   P[:,0], P[:,1], P[:,2])
        set_3d(joints, P[:,0], P[:,1], P[:,2])
        ee_trail = all_positions[:frame+1, -1, :]
        set_3d(trail, ee_trail[:,0], ee_trail[:,1], ee_trail[:,2])
        return line, joints, trail

    interval = max(1, int((duration * 1000) / len(q_traj)))
    anim = FuncAnimation(fig, animate, frames=len(q_traj),
                         interval=interval, blit=False, repeat=repeat)
    plt.tight_layout()
    return anim

# =========================
# 7) 데모
# =========================
if __name__ == "__main__":
    ANIMATION_DURATION = 5.0
    ANIMATION_REPEAT   = False
    TRAJECTORY_STEPS   = 100

    chain = build_chain()

    # 예시: 작업공간 내 좌표 (m)
    start = (0.05, 0.00, 0.00)
    end   = (0.05, 0.10, -0.1)
    start_rpy = (0.0, 0.0, 0.0)
    end_rpy   = (0.0, 0.0, 0.0)

    print("=== PTP planning ===")
    print("start:", start, "end:", end)

    q_traj = plan_ptp(chain, start, end,
                      start_rpy, end_rpy,
                      steps=TRAJECTORY_STEPS,
                      q0=None,
                      use_orientation=True)

    np.set_printoptions(precision=3, suppress=True)
    print("traj shape:", q_traj.shape)
    print("first:", q_traj[0])
    print("last :", q_traj[-1])

    print("\nStarting animation...")
    anim = animate_robot_trajectory(chain, q_traj,
                                    duration=ANIMATION_DURATION,
                                    repeat=ANIMATION_REPEAT,
                                    start_pos=start, end_pos=end)

    # 저장 옵션(원하면 주석 해제)
    # anim.save('humanoid_arm_ptp.gif', writer='pillow', fps=30)
    # anim.save('humanoid_arm_ptp.mp4', writer='ffmpeg', fps=30)

    plt.show()
