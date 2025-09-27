#!/usr/bin/env python3

import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from dongsoo_interfaces.srv import DongSooExecutor
from std_msgs.msg import Float32MultiArray, Int32MultiArray, Float32, String
from rclpy.callback_groups import ReentrantCallbackGroup
from dongsoo_control_pkg.Inverse_Kinematics import get_ik_result
import numpy as np

def deg_to_pulse(degree):
    if isinstance(degree, (list, tuple, np.ndarray)):
        deg_arr = np.asarray(degree, dtype=np.float64)
        p = np.rint(deg_arr * (4096.0 / 360.0) + 2048).astype(int)
        return np.clip(p, 0, 4095).tolist()
    else:
        p = int(np.round(degree * (4096.0 / 360.0) + 2048))
        return max(0, min(4095, p))

def pulse_to_deg(pulse):
    if isinstance(pulse, (list, tuple, np.ndarray)):
        pulse_arr = np.asarray(pulse, dtype=int)
        pulse_arr = np.clip(pulse_arr, 0, 4095)
        return ((pulse_arr - 2048) * (360.0 / 4096.0)).tolist()
    else:
        pulse = max(0, min(4095, int(pulse)))
        return (pulse - 2048) * (360.0 / 4096.0)

def rad_to_pulse(rad):
    if isinstance(rad, (list, tuple, np.ndarray)):
        rad_arr = np.asarray(rad, dtype=np.float64)
        p = np.rint(rad_arr * (4096.0 / (2*np.pi)) + 2048).astype(int)
        return np.clip(p, 0, 4095).tolist()
    else:
        p = int(np.round(rad * (4096.0 / (2*np.pi)) + 2048))
        return max(0, min(4095, p))

def pulse_to_rad(pulse):
    if isinstance(pulse, (list, tuple, np.ndarray)):
        pulse_arr = np.asarray(pulse, dtype=int)
        pulse_arr = np.clip(pulse_arr, 0, 4095)
        return ((pulse_arr - 2048) * (2*np.pi / 4096.0)).tolist()
    else:
        pulse = max(0, min(4095, int(pulse)))
        return (pulse - 2048) * (2*np.pi / 4096.0)

def plan_wrist_trajectory(start_deg, end_deg, steps=80, traj_type='smooth'):
    if traj_type == 'linear':
        alphas = np.linspace(0.0, 1.0, steps)
        wrist_traj = (1 - alphas) * start_deg + alphas * end_deg
    elif traj_type == 'smooth':
        t = np.linspace(0.0, 1.0, steps)
        alphas = 3 * t**2 - 2 * t**3
        wrist_traj = (1 - alphas) * start_deg + alphas * end_deg
    else:
        raise ValueError("traj_type은 'linear' 또는 'smooth'")
    return wrist_traj

def plan_joint_trajectory(q_start, q_end, steps=80, traj_type='smooth'):
    q_start = np.asarray(q_start, dtype=float)
    q_end = np.asarray(q_end, dtype=float)

    if traj_type == 'linear':
        alphas = np.linspace(0.0, 1.0, steps)
        q_traj = (1 - alphas)[:, None] * q_start[None, :] + alphas[:, None] * q_end[None, :]
    elif traj_type == 'smooth':
        t = np.linspace(0.0, 1.0, steps)
        alphas = 3 * t**2 - 2 * t**3
        q_traj = (1 - alphas)[:, None] * q_start[None, :] + alphas[:, None] * q_end[None, :]
    else:
        raise ValueError("traj_type은 'linear' 또는 'smooth'")
    return q_traj

def calculate_joint_velocities(q_start_deg, q_end_deg, work_time):
    q_start = np.asarray(q_start_deg, dtype=float)
    q_end = np.asarray(q_end_deg, dtype=float)

    joint_displacements_deg = np.abs(q_end - q_start)
    max_displacement_deg = np.max(joint_displacements_deg)

    if max_displacement_deg < 0.1:
        return np.ones(4, dtype=float)

    velocity_ratios = joint_displacements_deg / max_displacement_deg
    velocity_ratios = np.clip(velocity_ratios, 0.1, 1.0)

    max_angular_velocity_deg_per_sec = max_displacement_deg / work_time
    max_rpm = max_angular_velocity_deg_per_sec * (60.0 / 360.0)
    max_goal_velocity = max_rpm / 0.229

    joint_velocities = velocity_ratios * max_goal_velocity
    joint_velocities = np.clip(joint_velocities, 5.0, 1023.0)

    return joint_velocities.astype(int)

class DongsooServer(Node):
    def __init__(self):
        super().__init__('dongsoo_server')
        self.get_logger().info('[AIOT] DongSoo Service Server On! ')
        
        self.lock = threading.Lock()
        self.srv_cb_gp = ReentrantCallbackGroup()
        self.sub_cb_gp = ReentrantCallbackGroup()
        
        # 팔 Pose 수신
        self.create_subscription(
            Float32MultiArray,
            '/aiot/matrix/gripper',
            self.gripper_mat_callback,
            10, callback_group=self.sub_cb_gp)
        
        # Present Joint Position 수신
        self.create_subscription(
            Int32MultiArray,
            '/aiot/array/present_motor_pulse',
            self.joint_deg_callback, 10,
            callback_group=self.sub_cb_gp)
        
        self.motor_command_pub = self.create_publisher(Float32MultiArray, '/aiot/array/target_motor_deg', 10)
        self.wrist_pub = self.create_publisher(Float32, '/aiot/float/target_wrist_deg', 10)
        self.ik_done_pub = self.create_publisher(String, '/info/string/movement_done', 10)
        self.motor_veloticy = self.create_publisher(Float32MultiArray, '/aiot/array/motor_speed', 10)
        
        self.create_service(DongSooExecutor, 'dongsoo_executor', self.service_callback, callback_group=self.srv_cb_gp)
        
        self.present_j = np.array([0.0, 0.0, 0.0, 0.0])
        self.present_wrist = 0.0
        
    def gripper_mat_callback(self, msg : Float32MultiArray):
        dims = msg.layout.dim

        if len(dims) < 2:
            self.get_logger().warn('[AIOT] 잘못된 행렬 수신 ')
            return

        rows = dims[0].size
        cols = dims[1].size

        if len(msg.data) != rows * cols:
            self.get_logger().warn(f'[AIOT] msg count error : msg_count : {rows*cols}')
            return
        
        grip_mat = np.asarray(msg.data, dtype=np.float32).reshape(rows, cols)
        
        with self.lock:
            self.present_position = np.array([grip_mat[:3,3]])
            self.present_orientation = np.array([grip_mat[:3,:3]])
            
    def joint_deg_callback(self, msg : Int32MultiArray):
        data = list(msg.data)

        if len(data) != 5:
            self.get_logger().warn('[AIOT] Wrong Data length')
            return

        with self.lock:
            present_deg = [pulse_to_deg(float(q)) for q in data]
            self.present_j = present_deg[:4]
            self.present_wrist = present_deg[4]
            
    def service_callback(self, req, response):
        try:
            with self.lock:
                wrist_start = self.present_wrist
                joint_start = self.present_j
                start_point = self.present_position
            
            end_point = req.position
            end_look  = req.look
            work_time = req.time
            wrist     = req.wrist
            
            if end_look == 'down':
                q_result = get_ik_result(start_point, end_point, mode='down', w_ori=0.2)
            elif end_look == 'straight':
                q_result = get_ik_result(start_point, end_point, mode='straight', w_ori=0.2)
            else:
                self.get_logger().warn('[AIOT] 잘못된 방향 입력 ')
                response.success = False
                return
            
            q_end_deg = [np.degrees(q) for q in q_result['q_end']]
            joint_start_deg = joint_start

            print(' ')
            for i, deg in enumerate(q_end_deg):
                self.get_logger().info(f'[AIOT] [Q_list_{i+1}] : {deg:7.2f}°')

            joint_velocities = calculate_joint_velocities(joint_start_deg, q_end_deg, work_time)
            self.get_logger().info(f'[AIOT] Joint Velocities: {joint_velocities}')

            if abs(wrist - wrist_start) > 1.0:
                wrist_target = wrist
                self.get_logger().info(f'[AIOT] Wrist : {wrist_start:7.2f}° -> {wrist_target:7.2f}° (절대 각도 목표)')
            else:
                wrist_target = wrist_start
                self.get_logger().info(f'[AIOT] Wrist : {wrist_start:7.2f}° (현재 위치 유지)')

            vel_msg = Float32MultiArray()
            vel_msg.data = [float(v) for v in joint_velocities]
            self.motor_veloticy.publish(vel_msg)

            time.sleep(0.1)

            q_msg = Float32MultiArray()
            q_msg.data = q_end_deg
            self.motor_command_pub.publish(q_msg)

            w_msg = Float32()
            w_msg.data = float(wrist_target)
            self.wrist_pub.publish(w_msg)

            time.sleep(work_time + 0.5)

            response.success = True

            ik_msg = String()
            ik_msg.data = 'done'
            self.get_logger().info(f'[AIOT] {ik_msg.data}')
            self.ik_done_pub.publish(ik_msg)
            
        except Exception as e:
            self.get_logger().error(f'[AIOT] Planning or Ik Fail : {e}')
            response.success = False

        return response

def main(args=None):
    rclpy.init(args=args)
    node = DongsooServer()

    exec = MultiThreadedExecutor(num_threads=4)
    exec.add_node(node)

    try:
        exec.spin()
    except KeyboardInterrupt:
        print("\n\nShutting down Dongsoo Service Server...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()