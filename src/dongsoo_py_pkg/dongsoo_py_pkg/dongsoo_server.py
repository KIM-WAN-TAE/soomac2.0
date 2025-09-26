#!/usr/bin/env python3

import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from dongsoo_interfaces.srv import DongSooExecutor
from std_msgs.msg import Float32MultiArray, Int32MultiArray, String
from rclpy.callback_groups import ReentrantCallbackGroup
from dongsoo_py_pkg.Inverse_Kinematics import get_ik_result
import numpy as np

def rad_to_pulse(rad_values):
    pulse_values = []
    for rad_val in rad_values:
        pulse_val = int((rad_val * 4096.0 / (2 * np.pi)) + 2048)
        pulse_values.append(pulse_val)
    return pulse_values

def deg_to_pulse(degree):
    p = int(np.round(-degree * (4096.0 / 360.0) + 2048))
    return max(0, min(4095, p))

def pulse_to_deg(pulse):
    degree = -(pulse - 2048) * (360.0 / 4096.0)
    return degree

def plan_wrist_trajectory(start_deg, end_deg, steps=80, traj_type='smooth'):
    """Plan wrist trajectory in degrees."""
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

class DongsooServer(Node):
    def __init__(self):
        super().__init__('dongsoo_server')
        self.get_logger().info(' DongSoo Service Server On! ')
        
        self.data_lock = threading.Lock()
        self.srv_cb_group = ReentrantCallbackGroup()
        self.sub_cb_group = ReentrantCallbackGroup()
        
        self.gripper_mat_sub = self.create_subscription(
            Float32MultiArray,
            '/info/matrix/gripper',
            self.gripper_mat_callback,
            10,
            callback_group = self.sub_cb_group
        )
        # Subscribe to current joint positions to ensure smooth start
        self.joint_pos_sub = self.create_subscription(
            Int32MultiArray,
            '/motor/position',
            self.joint_position_callback,
            10,
            callback_group=self.sub_cb_group
        )

        self.motor_control_pub = self.create_publisher(Int32MultiArray, '/motor/command_position', 10)
        self.wrist_pub         = self.create_publisher(Int32MultiArray, '/motor/command_dxl5_position', 10)
        self.ik_done_pub       = self.create_publisher(String, '/info/string/movement_done', 10)
        
        self.present_position = np.array([])
        self.present_orientation = np.array([])
        # Track latest joint pulses and active joint angles (J1..J4) in radians
        self.latest_pulses = np.array([2048, 2048, 2048, 2048, 2048], dtype=np.int32)
        self.q_current_active = np.zeros(4, dtype=float)
        self.wrist_current_deg = 0.0  # Current wrist angle in degrees
        self.last_joint_update = None
        self.last_wrist_target = None  # Track last wrist target to detect changes

        self.last_end_point = None
        
        self.create_service(DongSooExecutor, 'dongsoo_executor', self.service_callback, callback_group = self.srv_cb_group)
    
    def gripper_mat_callback(self, msg : Float32MultiArray):
        with self.data_lock:
            dims = msg.layout.dim
            
            if len(dims) < 2:
                self.get_logger().warn(' 잘못된 행렬 수신 ')
                return

            rows = dims[0].size
            cols = dims[1].size
            
            if len(msg.data) != rows * cols:
                self.get_logger().warn(f' msg count error : msg_count : {rows*cols}')
                return
            
            self.grip_mat = np.asarray(msg.data, dtype=np.float32).reshape(rows, cols)
            self.present_position = np.array([self.grip_mat[:3,3]])
            self.present_orientation = np.array([self.grip_mat[:3,:3]])
            
            # # monitoring
            # print(f'Position    : {self.present_position:8.4f}')
            # print(f'Orientation : {self.present_orientation:8.4f}')
            
    def joint_position_callback(self, msg: Int32MultiArray):
        """Update latest joint positions and compute current active joint angles (J1..J4) and wrist."""
        with self.data_lock:
            data = list(msg.data)
            if len(data) >= 4:
                # Store pulses
                for i in range(min(5, len(data))):
                    self.latest_pulses[i] = int(data[i])
                # Convert to radians for J1..J4 (assuming 2048 -> 0 rad)
                # 4096 cnt/rev, 2*pi rad/rev
                cnt2rad = 2.0 * np.pi / 4096.0
                self.q_current_active = ((self.latest_pulses[:4] - 2048).astype(float)) * cnt2rad

                # Update wrist current angle in degrees (Joint 5)
                if len(data) >= 5:
                    self.wrist_current_deg = pulse_to_deg(self.latest_pulses[4])

                self.last_joint_update = self.get_clock().now()

    def service_callback(self, req, response):
        try:
            # Desired end-effector pose from request
            if self.last_end_point is None:
                start_point = self.present_position
            else:
                # Ensure start_point is properly formatted as numpy array
                start_point = np.asarray(self.last_end_point, dtype=np.float32)
                # Make sure it has the same shape as present_position
                if start_point.ndim == 1:
                    start_point = start_point.reshape(1, -1)
            end_point = req.position
            end_look  = req.look
            work_time = req.time
            wrist     = req.wrist

            self.last_end_point = end_point

            # Use current joint angles (from /motor/position) as start state to avoid initial jerk
            with self.data_lock:
                q_start = self.q_current_active.copy()
                wrist_start = self.wrist_current_deg

            # Solve only for end configuration, warm-starting from current joints
            if end_look == 'down':
                q_result = get_ik_result(start_point, end_point, mode='down', w_ori=0.2)
            elif end_look == 'straight':
                q_result = get_ik_result(start_point, end_point, mode='straight', w_ori=0.2)
            else:
                self.get_logger().warn(' 잘못된 방향 입력 ')
                response.success = False
                return

            # Replace IK-computed q_start with actual current joints to ensure smooth start
            q_end = q_result['q_end']
            print(' ')
            for i, _ in enumerate(q_end):
                self.get_logger().info(f'[Q_list_{i+1}] : {np.degrees(q_end[i]):7.2f}')


            # 200Hz pub frequency: 1/200 = 0.005s per step
            pub_frequency = 5  # Hz
            sleep_time = 1.0 / pub_frequency  # 0.005s

            # Calculate trajectory steps based on work_time and publish frequency
            total_steps = int(work_time * pub_frequency)

            q_msg = Int32MultiArray()
            w_msg = Int32MultiArray()
            q_list = plan_joint_trajectory(q_start, q_end, steps=total_steps, traj_type='smooth')

            # Check if wrist target has changed (absolute angle comparison)
            if self.last_wrist_target is None or abs(wrist - self.last_wrist_target) > 0.01:
                # New absolute wrist target detected - plan trajectory to new absolute position
                wrist_target = wrist  # Use absolute target angle
                self.last_wrist_target = wrist
                self.get_logger().info(f'[Wrist] : {wrist_start:7.2f}° -> {wrist_target:7.2f}° (절대 각도 목표)')
            else:
                # No change in wrist target - maintain current position
                wrist_target = wrist_start
                self.get_logger().info(f'[Wrist] : {wrist_start:7.2f}° (현재 위치 유지)')

            # Plan wrist trajectory
            wrist_list = plan_wrist_trajectory(wrist_start, wrist_target, steps=total_steps, traj_type='smooth')


            # Combined trajectory execution with 5-joint control
            for i, (q_s, wrist_deg) in enumerate(zip(q_list, wrist_list)):
                # Convert Joint 1-4 from radians to pulse
                q_pulse = rad_to_pulse(q_s)
                # Convert Joint 5 (wrist) from degrees to pulse
                wrist_pulse = deg_to_pulse(wrist_deg)
                # Combine all 5 joints
                q_msg.data = q_pulse
                w_msg.data = [int(wrist_pulse)]
                self.motor_control_pub.publish(q_msg)
                self.wrist_pub.publish(w_msg)
                time.sleep(sleep_time)
            
            response.success = True
            
            ik_msg = String()
            ik_msg.data = 'done'
            self.get_logger().info(f'{ik_msg.data}')
            self.ik_done_pub.publish(ik_msg)

        except Exception as e:
            self.get_logger().error(f' Planning or Ik Fail : {e}')
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
