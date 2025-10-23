#!/usr/bin/env python3

import numpy as np
from scipy.optimize import fsolve
from dongsoo_control_pkg.read_json import GravityDH, GripperDH

grip_dh = GripperDH()

grip_d = grip_dh.get_parameter_list('d')
grip_a = grip_dh.get_parameter_list('a')
grip_alpha = grip_dh.get_parameter_list('alpha')
grip_th_off = grip_dh.get_parameter_list('theta_offset')

L1 =  grip_d[0]
L2 =  grip_a[1]
L3 =  grip_a[2]
L4 =  grip_d[4]

# if orientation_mode == 'straight':
#         length_n = np.sqrt(x**2 + y**2) - L4
#         length_z = z - L1
# elif orientation_mode == 'down':
#     length_n = np.sqrt(x**2 + y**2)
#     length_z = z - L1 + L4

MIN_LIMITS = np.array([np.deg2rad(-100), np.deg2rad(-110), np.deg2rad(-115), np.deg2rad(-100)])
MAX_LIMITS = np.array([np.deg2rad( 100), np.deg2rad( 110), np.deg2rad( 115), np.deg2rad( 100)])

def solve_ik_position_only(target_pos, initial_q_active=None,
                           xtol=1e-7, ftol=1e-7, retries=6, jitter=3e-3,
                           print_prefix="[start-pos]"):

    x, y, z = target_pos

    global length_n, length_z

    length_n = np.sqrt(x**2 + y**2) - L4  # How to define initial gripper pose
    length_z = z - L1
        
    def equations(vars):
        t1, t2 = vars
        eq1 = (L2*np.sin(t1) + L3*np.sin(t1+t2)) - length_n
        eq2 = (L2*np.cos(t1) + L3*np.cos(t1+t2)) - length_z
        return [eq1, eq2]

    sol = fsolve(equations, [0, 0])
    theta1_sol, theta2_sol = sol

    t1, t2 = theta1_sol, theta2_sol
    t1 = ((t1 + np.pi) % (2 * np.pi)) - np.pi
    t2 = ((t2 + np.pi) % (2 * np.pi)) - np.pi

    th_z = np.arctan2(y, x)
    if t2 < 0:
        t1 = 2*np.arctan2(length_n, length_z) - t1
        t2 = -t2

    angles = np.array([
        th_z,     # th0
        -t1,                    # th1
        -t2,                    # th2
        -((np.pi / 2) - (t1 + t2))      # th3
    ])

    angles = np.clip(angles, MIN_LIMITS, MAX_LIMITS)

    return angles

def solve_ik_with_down(target_pos, initial_q_active=None,
                       w_ori=0.2, xtol=1e-7, ftol=1e-7,
                       retries=10, jitter=3e-3, print_prefix="[end-down]"):
    x, y, z = target_pos

    global length_n, length_z

    length_n = np.sqrt(x**2 + y**2)
    length_z = z - L1 + L4
        
    def equations(vars):
        t1, t2 = vars
        eq1 = (L2*np.sin(t1) + L3*np.sin(t1+t2)) - length_n
        eq2 = (L2*np.cos(t1) + L3*np.cos(t1+t2)) - length_z
        return [eq1, eq2]

    sol = fsolve(equations, [0, 0])
    theta1_sol, theta2_sol = sol

    t1, t2 = theta1_sol, theta2_sol
    t1 = ((t1 + np.pi) % (2 * np.pi)) - np.pi
    t2 = ((t2 + np.pi) % (2 * np.pi)) - np.pi

    th_z = np.arctan2(y, x)
    if t2 < 0:
        t1 = 2*np.arctan2(length_n, length_z) - t1
        t2 = -t2

    angles = np.array([
        th_z,     # th0
        -t1,                    # th1
        -t2,                    # th2
        -((np.pi) - (t1 + t2))      # th3
    ])

    angles = np.clip(angles, MIN_LIMITS, MAX_LIMITS)

    return angles

def solve_ik_with_straight(target_pos, initial_q_active=None,
                           w_ori=0.2, xtol=1e-7, ftol=1e-7,
                           retries=10, jitter=3e-3, print_prefix="[end-straight]",
                           Z_TOL=0.10):
    x, y, z = target_pos

    global length_n, length_z

    length_n = np.sqrt(x**2 + y**2) - L4  # How to define initial gripper pose
    length_z = z - L1
        
    def equations(vars):
        t1, t2 = vars
        eq1 = (L2*np.sin(t1) + L3*np.sin(t1+t2)) - length_n
        eq2 = (L2*np.cos(t1) + L3*np.cos(t1+t2)) - length_z
        return [eq1, eq2]

    sol = fsolve(equations, [0, 0])
    theta1_sol, theta2_sol = sol

    t1, t2 = theta1_sol, theta2_sol
    t1 = ((t1 + np.pi) % (2 * np.pi)) - np.pi
    t2 = ((t2 + np.pi) % (2 * np.pi)) - np.pi

    th_z = np.arctan2(y, x)
    if t2 < 0:
        t1 = 2*np.arctan2(length_n, length_z) - t1
        t2 = -t2

    angles = np.array([
        th_z,     # th0
        -t1,                    # th1
        -t2,                    # th2
        -((np.pi / 2) - (t1 + t2))      # th3
    ])

    angles = np.clip(angles, MIN_LIMITS, MAX_LIMITS)

    return angles


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

course = get_ik_result([0.2, 0, 0.1], [0.0, 0.4, 0.1], mode='straight', w_ori=0.0)
print(L1, L2, L3, L4)
print(course)