#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
4-DOF 로봇팔 IK 시뮬레이터 (Matplotlib 3D Animation)
- IK_Done 모듈을 사용하여 start/end IK 계산
- 관절공간 궤적 계획을 통한 부드러운 애니메이션
- 자세 모드: 'down' | 'straight'
- 단위: m (DH는 m 단위)
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from ikpy.link import DHLink
from dongsoo_py_pkg.Inverse_Kinematics import get_ik_result, CHAIN

# ---------------------------------------
# 유틸 함수들 (IK_Done에서 가져오지 못한 것들)
# ---------------------------------------
def chain_max_reach():
    """로봇 체인의 최대 도달 범위 계산"""
    total = 0.0
    for link in CHAIN.links[1:]:
        if isinstance(link, DHLink):
            total += abs(link.a) + abs(link.d)
    return max(total, 0.3)  # 최소 스케일 확보(0.3 m)

def forward_points(q_full):
    """각 링크의 위치 계산"""
    Ts = CHAIN.forward_kinematics(q_full, full_kinematics=True)
    pts = np.array([T[:3, 3] for T in Ts])
    return pts  # shape: (num_links, 3)

def end_axes(T, scale=0.08):
    """그리퍼 좌표축 벡터 (미터 단위, 보기 좋게 ~8cm)"""
    p = T[:3, 3]
    x_axis = T[:3, 0] * scale   # 빨강으로 표시
    z_axis = T[:3, 2] * scale   # 파랑으로 표시
    return p, x_axis, z_axis

# ---------------------------------------
# 궤적 계획 함수들
# ---------------------------------------
def plan_joint_trajectory(q_start, q_end, steps=80, traj_type='linear'):
    """
    관절공간에서 q_start → q_end 궤적 계획
    
    Args:
        q_start: 시작 관절각 [q1, q2, q3, q4]
        q_end: 끝 관절각 [q1, q2, q3, q4]
        steps: 궤적 단계 수
        traj_type: 'linear', 'smooth' (추후 확장 가능)
        
    Returns:
        q_traj: [steps, 4] 관절각 궤적
    """
    q_start = np.asarray(q_start, dtype=float)
    q_end = np.asarray(q_end, dtype=float)
    
    if traj_type == 'linear':
        # 선형 보간
        alphas = np.linspace(0.0, 1.0, steps)
        q_traj = (1 - alphas)[:, None] * q_start[None, :] + alphas[:, None] * q_end[None, :]
    elif traj_type == 'smooth':
        # S-커브 보간 (부드러운 가속/감속)
        t = np.linspace(0.0, 1.0, steps)
        # 3차 다항식: 3t^2 - 2t^3 (0에서 0, 1에서 1, 부드러운 전환)
        alphas = 3 * t**2 - 2 * t**3
        q_traj = (1 - alphas)[:, None] * q_start[None, :] + alphas[:, None] * q_end[None, :]
    else:
        raise ValueError("traj_type은 'linear' 또는 'smooth'")
    
    return q_traj

# ---------------------------------------
# 애니메이션 함수
# ---------------------------------------
def animate_trajectory(q_traj, start_pos, end_pos, mode='down', interval=40):
    """
    관절공간 궤적을 3D 애니메이션으로 시각화
    
    Args:
        q_traj: [steps, 4] 관절각 궤적
        start_pos: 시작 위치 [x, y, z]
        end_pos: 끝 위치 [x, y, z]
        mode: 'down' 또는 'straight'
        interval: 애니메이션 간격 (ms)
    """
    q_traj = np.asarray(q_traj)
    N = q_traj.shape[0]

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # 축 범위 설정
    reach = chain_max_reach()
    s = reach * 0.8
    ax.set_xlim(-s, s); ax.set_ylim(-s, s); ax.set_zlim(0, max(s, reach))
    ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]'); ax.set_zlabel('Z [m]')
    ax.set_title(f'4-DOF Arm Trajectory (mode: {mode})')

    # 시작/끝 위치 표시
    start_pos = np.asarray(start_pos).reshape(3)
    end_pos = np.asarray(end_pos).reshape(3)
    ax.scatter([start_pos[0]], [start_pos[1]], [start_pos[2]], 
              s=50, c='green', marker='o', label='start', alpha=0.7)
    ax.scatter([end_pos[0]], [end_pos[1]], [end_pos[2]], 
              s=50, c='red', marker='^', label='end', alpha=0.7)
    ax.legend(loc='upper left')

    # 초기 로봇 상태
    init_full_q = np.hstack(([0.0], q_traj[0]))
    pts0 = forward_points(init_full_q)
    link_line, = ax.plot(pts0[:, 0], pts0[:, 1], pts0[:, 2], 
                        'o-', lw=3, markersize=6, color='blue', alpha=0.8)

    # 그리퍼 좌표축
    T0 = CHAIN.forward_kinematics(init_full_q, full_kinematics=True)[-1]
    p0, x0, z0 = end_axes(T0)
    x_line, = ax.plot([p0[0], p0[0] + x0[0]], [p0[1], p0[1] + x0[1]], 
                     [p0[2], p0[2] + x0[2]], lw=3, color='red', alpha=0.8)
    z_line, = ax.plot([p0[0], p0[0] + z0[0]], [p0[1], p0[1] + z0[1]], 
                     [p0[2], p0[2] + z0[2]], lw=3, color='blue', alpha=0.8)

    # 궤적 추적선 (end-effector path)
    trajectory_x, trajectory_y, trajectory_z = [], [], []
    trajectory_line, = ax.plot([], [], [], '--', lw=2, color='orange', alpha=0.6)

    def update(frame):
        # 현재 프레임의 관절각
        q_full = np.hstack(([0.0], q_traj[frame]))
        pts = forward_points(q_full)
        
        # 로봇 링크 업데이트
        link_line.set_data(pts[:, 0], pts[:, 1])
        link_line.set_3d_properties(pts[:, 2])

        # 그리퍼 좌표축 업데이트
        T = CHAIN.forward_kinematics(q_full, full_kinematics=True)[-1]
        p, xv, zv = end_axes(T)
        x_line.set_data([p[0], p[0] + xv[0]], [p[1], p[1] + xv[1]])
        x_line.set_3d_properties([p[2], p[2] + xv[2]])
        z_line.set_data([p[0], p[0] + zv[0]], [p[1], p[1] + zv[1]])
        z_line.set_3d_properties([p[2], p[2] + zv[2]])
        
        # 궤적 추적 (end-effector 경로)
        trajectory_x.append(p[0])
        trajectory_y.append(p[1])
        trajectory_z.append(p[2])
        trajectory_line.set_data(trajectory_x, trajectory_y)
        trajectory_line.set_3d_properties(trajectory_z)
        
        return link_line, x_line, z_line, trajectory_line

    anim = FuncAnimation(fig, update, frames=N, interval=interval, 
                        blit=False, repeat=True)
    plt.show()
    return anim

# ---------------------------------------
# 메인 실행부
# ---------------------------------------
if __name__ == '__main__':
    # 시작/끝 위치 설정 (미터 단위)
    start_pos = np.array([0.2, 0.2, 0.3])
    end_pos = np.array([0.2, -0.2, 0.1])
    mode = 'down'  # 'down' 또는 'straight'
    w_ori = 0.2
    steps = 100  # 궤적 단계 수
    
    print(f"IK 계산 중... (start: {start_pos}, end: {end_pos}, mode: {mode})")
    
    # IK_Done 모듈로 시작/끝 관절각 계산
    try:
        ik_result = get_ik_result(start_pos, end_pos, mode=mode, w_ori=w_ori)
        q_start = ik_result['q_start']
        q_end = ik_result['q_end']
        
        print(f"시작 관절각 (rad): {q_start}")
        print(f"끝 관절각 (rad): {q_end}")
        print(f"관절각 변화량 (rad): {q_end - q_start}")
        
        # 관절공간 궤적 계획
        q_trajectory = plan_joint_trajectory(q_start, q_end, steps=steps, traj_type='smooth')
        print(f"궤적 생성 완료: {steps} 단계")
        
        # 애니메이션 실행
        print("애니메이션 시작...")
        anim = animate_trajectory(q_trajectory, start_pos, end_pos, mode=mode, interval=50)
        
    except Exception as e:
        print(f"오류 발생: {e}")
        import traceback
        traceback.print_exc()