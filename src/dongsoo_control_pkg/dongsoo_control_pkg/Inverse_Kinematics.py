#!/usr/bin/env python3

import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, DHLink
from scipy.optimize import least_squares
from dongsoo_control_pkg.read_json import GravityDH

MIN_LIMITS = np.array([0.0, np.deg2rad(-135), np.deg2rad(-135), np.deg2rad(-135), np.deg2rad(-135)])
MAX_LIMITS = np.array([0.0, np.deg2rad( 135), np.deg2rad( 135), np.deg2rad( 135), np.deg2rad( 135)])

POS_TOL = 2e-3
FALLBACK_TOL = 5e-3
ANG_TOL = np.deg2rad(5.0)
LOWER_BOUNDS = MIN_LIMITS[1:]
UPPER_BOUNDS = MAX_LIMITS[1:]

gravity_dh = GravityDH()

def create_robot_chain():
    links = [OriginLink()]
    for joint in gravity_dh.joints_data:
        dh_params = joint.get('dh_params', {})
        d = float(dh_params.get('d', 0.0))
        a = float(dh_params.get('a', 0.0))
        alpha = float(dh_params.get('alpha', 0.0))
        theta_offset = float(dh_params.get('theta_offset', 0.0))
        links.append(DHLink(d=d, a=a, alpha=alpha, theta=theta_offset))
    return Chain(name='4DOF_arm', links=links, active_links_mask=[False, True, True, True, True])

CHAIN = create_robot_chain()

def unit(v, eps=1e-12):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n < eps:
        return np.zeros_like(v)
    return v / n


def _error_single(q_active, target_pos, target_x, w_ori=0.2):
    full_q = np.hstack(([0.0], q_active))
    T = CHAIN.forward_kinematics(full_q, full_kinematics=True)[-1]
    pos_err = T[:3, 3] - target_pos
    xh = T[:3, 0]
    ori_err = (xh - target_x) * w_ori
    return np.hstack((pos_err, ori_err))

def _error_pos_only(q_active, target_pos):
    full_q = np.hstack(([0.0], q_active))
    T = CHAIN.forward_kinematics(full_q, full_kinematics=True)[-1]
    return T[:3, 3] - target_pos

def _error_straight(q_active, target_pos, w_ori=0.2):
    full_q = np.hstack(([0.0], q_active))
    T = CHAIN.forward_kinematics(full_q, full_kinematics=True)[-1]
    pos_err = T[:3, 3] - target_pos
    xh_z = T[2, 0]
    return np.hstack((pos_err, np.array([w_ori * xh_z])))


def _get_initial_guess(target_pos, initial_q_active=None):
    n_links = len(CHAIN.links)
    if initial_q_active is None:
        raw0 = CHAIN.inverse_kinematics(
            target_position=target_pos,
            initial_position=[0.0] * n_links
        )
        return raw0[1:]
    return np.asarray(initial_q_active, dtype=float)

def solve_ik_position_only(target_pos, initial_q_active=None,
                           xtol=1e-7, ftol=1e-7, retries=6, jitter=3e-3,
                           print_prefix="[start-pos]"):

    x0 = _get_initial_guess(target_pos, initial_q_active)

    x_init = x0.copy()
    for tr in range(retries):
        try:
            res = least_squares(
                _error_pos_only, x_init,
                bounds=(LOWER_BOUNDS, UPPER_BOUNDS),
                args=(target_pos,),
                xtol=xtol, ftol=ftol,
                loss='soft_l1'
            )
            q_try = res.x
            full_q_try = np.hstack(([0.0], q_try))
            T_try = CHAIN.forward_kinematics(full_q_try, full_kinematics=True)[-1]
            pos_err = np.linalg.norm(T_try[:3, 3] - target_pos)
            if res.success and pos_err < POS_TOL:
                return q_try
            else:
                print(f"{print_prefix}[warn] pos_err={pos_err:.4f} m, retry {tr+1}/{retries}")
                x_init = np.clip(q_try + np.random.randn(*q_try.shape)*jitter, LOWER_BOUNDS, UPPER_BOUNDS)
        except Exception as e:
            print(f"{print_prefix}[error] least_squares 예외: {e}, retry {tr+1}/{retries}")
            x_init = np.clip(x_init + np.random.randn(*x_init.shape)*jitter, LOWER_BOUNDS, UPPER_BOUNDS)

    raw_fb = CHAIN.inverse_kinematics(target_position=target_pos, initial_position=[0.0]*len(CHAIN.links))
    q_fb = raw_fb[1:]
    full_q_fb = np.hstack(([0.0], q_fb))
    T_fb = CHAIN.forward_kinematics(full_q_fb, full_kinematics=True)[-1]
    pos_err_fb = np.linalg.norm(T_fb[:3, 3] - target_pos)
    if pos_err_fb < FALLBACK_TOL:
        print(f"{print_prefix}[fallback] CHAIN IK 사용 (|e|={pos_err_fb:.4f} m)")
        return q_fb
    raise RuntimeError(f"{print_prefix}단일 위치 IK 실패")

def solve_ik_with_down(target_pos, initial_q_active=None,
                       w_ori=0.2, xtol=1e-7, ftol=1e-7,
                       retries=10, jitter=3e-3, print_prefix="[end-down]"):
    x0 = _get_initial_guess(target_pos, initial_q_active)
    tgt_x = np.array([0.0, 0.0, -1.0])

    w = float(w_ori)
    x_init = x0.copy()
    for tr in range(retries):
        try:
            res = least_squares(
                _error_single, x_init,
                bounds=(LOWER_BOUNDS, UPPER_BOUNDS),
                args=(target_pos, tgt_x, w),
                xtol=xtol, ftol=ftol,
                loss='soft_l1'
            )
            q_try = res.x
            full_q_try = np.hstack(([0.0], q_try))
            T_try = CHAIN.forward_kinematics(full_q_try, full_kinematics=True)[-1]
            pos_err = np.linalg.norm(T_try[:3, 3] - target_pos)
            ang_err = np.arccos(np.clip(np.dot(unit(T_try[:3, 0]), tgt_x), -1.0, 1.0))
            if res.success and pos_err < POS_TOL and ang_err < ANG_TOL:
                return q_try
            else:
                print(f"{print_prefix}[warn] pos_err={pos_err:.4f} m, "
                      f"ang_err={np.degrees(ang_err):.2f} deg, retry {tr+1}/{retries}, w={w:.3f}")
                x_init = np.clip(q_try + np.random.randn(*q_try.shape)*jitter, LOWER_BOUNDS, UPPER_BOUNDS)
                w *= 0.6
        except Exception as e:
            print(f"{print_prefix}[error] least_squares 예외: {e}, retry {tr+1}/{retries}")
            x_init = np.clip(x_init + np.random.randn(*x_init.shape)*jitter, LOWER_BOUNDS, UPPER_BOUNDS)
            w *= 0.6

    try:
        res = least_squares(
            _error_single, x0.copy(),
            bounds=(LOWER_BOUNDS, UPPER_BOUNDS),
            args=(target_pos, tgt_x, 0.0),
            xtol=xtol, ftol=ftol, loss='soft_l1'
        )
        q_try = res.x
        full_q_try = np.hstack(([0.0], q_try))
        T_try = CHAIN.forward_kinematics(full_q_try, full_kinematics=True)[-1]
        pos_err = np.linalg.norm(T_try[:3, 3] - target_pos)
        if res.success and pos_err < POS_TOL:
            print(f"{print_prefix}[fallback] orientation 포기, 위치만 만족 (|e|={pos_err:.4f} m)")
            return q_try
        else:
            raise RuntimeError(f"{print_prefix}폴백 실패(pos_err={pos_err:.4f} m)")
    except Exception as e2:
        raise RuntimeError(f"{print_prefix}끝점 IK 실패: {e2}")

def solve_ik_with_straight(target_pos, initial_q_active=None,
                           w_ori=0.2, xtol=1e-7, ftol=1e-7,
                           retries=10, jitter=3e-3, print_prefix="[end-straight]",
                           Z_TOL=0.10):
    x0 = _get_initial_guess(target_pos, initial_q_active)

    w = float(w_ori)
    x_init = x0.copy()
    for tr in range(retries):
        try:
            res = least_squares(
                _error_straight, x_init,
                bounds=(LOWER_BOUNDS, UPPER_BOUNDS),
                args=(target_pos, w),
                xtol=xtol, ftol=ftol,
                loss='soft_l1'
            )
            q_try = res.x
            full_q_try = np.hstack(([0.0], q_try))
            T_try = CHAIN.forward_kinematics(full_q_try, full_kinematics=True)[-1]
            pos_err = np.linalg.norm(T_try[:3, 3] - target_pos)
            xh_z = T_try[2, 0]
            if res.success and pos_err < POS_TOL and abs(xh_z) < Z_TOL:
                return q_try
            else:
                print(f"{print_prefix}[warn] pos_err={pos_err:.4f} m, |xh_z|={abs(xh_z):.3f}, "
                      f"retry {tr+1}/{retries}, w={w:.3f}")
                x_init = np.clip(q_try + np.random.randn(*q_try.shape)*jitter, LOWER_BOUNDS, UPPER_BOUNDS)
                w *= 0.6
        except Exception as e:
            print(f"{print_prefix}[error] least_squares 예외: {e}, retry {tr+1}/{retries}")
            x_init = np.clip(x_init + np.random.randn(*x_init.shape)*jitter, LOWER_BOUNDS, UPPER_BOUNDS)
            w *= 0.6

    try:
        res = least_squares(
            _error_straight, x0.copy(),
            bounds=(LOWER_BOUNDS, UPPER_BOUNDS),
            args=(target_pos, 0.0),
            xtol=xtol, ftol=ftol, loss='soft_l1'
        )
        q_try = res.x
        full_q_try = np.hstack(([0.0], q_try))
        T_try = CHAIN.forward_kinematics(full_q_try, full_kinematics=True)[-1]
        pos_err = np.linalg.norm(T_try[:3, 3] - target_pos)
        if res.success and pos_err < POS_TOL:
            print(f"{print_prefix}[fallback] orientation 포기, 위치만 만족 (|e|={pos_err:.4f} m)")
            return q_try
        else:
            raise RuntimeError(f"{print_prefix}폴백 실패(pos_err={pos_err:.4f} m)")
    except Exception as e2:
        raise RuntimeError(f"{print_prefix}끝점 IK 실패: {e2}")


def get_ik_result(start, end, mode='down', w_ori=0.2):
    start = np.asarray(start, dtype=float).reshape(3)
    end = np.asarray(end, dtype=float).reshape(3)

    q_start = solve_ik_position_only(start, print_prefix="[start-pos]")

    if mode == 'down':
        q_end = solve_ik_with_down(end, initial_q_active=q_start,
                                   w_ori=w_ori, print_prefix="[end-down]")
    elif mode == 'straight':
        q_end = solve_ik_with_straight(end, initial_q_active=q_start,
                                       w_ori=w_ori, print_prefix="[end-straight]")
    else:
        raise ValueError("mode는 'down' 또는 'straight'")

    return {
        'q_start': q_start,
        'q_end': q_end
    }