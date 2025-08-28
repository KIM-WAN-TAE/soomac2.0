#!/usr/bin/env python3

import rclpy
import numpy as np
import threading
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from dynamixel_sdk import *
from dongsoo_py_pkg.read_json import CameraDH, GripperDH

# === DH 정의 ===
cam_dh = CameraDH()
grip_dh = GripperDH()

grip_d = grip_dh.get_parameter_list('d')
grip_a = grip_dh.get_parameter_list('a')
grip_alpha = grip_dh.get_parameter_list('alpha')
grip_th_off = grip_dh.get_parameter_list('theta_offset')

def dh_transform(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    
    T = np.array([[ ct, -st*ca,  st*sa, a*ct],
                  [ st,  ct*ca, -ct*sa, a*st],
                  [  0,      sa,     ca,    d],
                  [  0,       0,      0,    1]])
    return T

def rotation_matrix_to_rpy(R):
    """회전 행렬을 Roll-Pitch-Yaw로 변환"""
    sy = np.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
    
    singular = sy < 1e-6
    
    if not singular:
        roll = np.arctan2(R[2,1], R[2,2])
        pitch = np.arctan2(-R[2,0], sy)
        yaw = np.arctan2(R[1,0], R[0,0])
    else:
        roll = np.arctan2(-R[1,2], R[1,1])
        pitch = np.arctan2(-R[2,0], sy)
        yaw = 0
    
    return roll, pitch, yaw

class MonitoringHub(Node):
    def __init__(self):
        super().__init__('monitoring_hub')
        self.get_logger().info(' Monitoring Node On! ')
        
        # Thread lock for data synchronization
        self.data_lock = threading.Lock()
        
        # Data storage variables
        self.joint_pulses = [0, 0, 0, 0, 0]  # pulse values
        self.joint_degrees = [0.0, 0.0, 0.0, 0.0, 0.0]  # degree values
        self.q_rad = [0.0, 0.0, 0.0, 0.0, 0.0]  # radian values
        self.currents = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.velocities = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Pose data (4x4 transformation matrices)
        self.gripper_pose = np.eye(4)
        self.camera_pose = np.eye(4)
        
        # Subscribers
        self.sub_position = self.create_subscription(
            Int32MultiArray,
            '/motor/position',
            self.present_position_callback,
            10
        )
        
        self.sub_current = self.create_subscription(
            Float32MultiArray,
            '/motor/current',
            self.present_current_callback,
            10
        )
        
        self.sub_velocity = self.create_subscription(
            Float32MultiArray,
            '/motor/velocity',
            self.present_velocity_callback,
            10
        )
        
        # DH parameter objects
        self.cam_dh = CameraDH()
        self.grip_dh = GripperDH()

        timer_period = 1/10 # 100 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f"Camera DH joints: {self.cam_dh.get_joint_count()}")
        self.get_logger().info(f"Gripper DH joints: {self.grip_dh.get_joint_count()}")
        self.get_logger().info("Timer set to 0.1Hz (10 second intervals)")
    
    def fk(self, dh_params):
        """Forward kinematics calculation"""
        T = np.eye(4)
        for params in dh_params:
            T = T @ dh_transform(params['theta'], params['d'], params['a'], params['alpha'])
        return T
    
    def calculate_poses(self, q_rad):
        """Calculate gripper and camera poses from joint angles"""
        # Get DH parameters
        cam_params = self.cam_dh.get_all_dh_params(q_rad)
        grip_params = self.grip_dh.get_all_dh_params(q_rad)
        
        # Forward kinematics
        T_cam = self.fk(cam_params)
        T_grip = self.fk(grip_params)
        
        return {
            'camera': T_cam,
            'gripper': T_grip
        }
    
    def present_position_callback(self, msg: Int32MultiArray):
        with self.data_lock:
            pulses = list(msg.data)
            
            # Store pulse values
            for i in range(min(5, len(pulses))):
                self.joint_pulses[i] = pulses[i]
                # Convert pulse to degrees (assuming 4096 pulses = 360 degrees for Dynamixel)
                self.joint_degrees[i] = (pulses[i] / 4096.0) * 360.0
                # Convert to radians
                self.q_rad[i] = np.deg2rad(self.joint_degrees[i])
            
            # Calculate poses
            poses = self.calculate_poses(self.q_rad)
            self.camera_pose = poses['camera']
            self.gripper_pose = poses['gripper']
    
    def present_current_callback(self, msg: Float32MultiArray):
        with self.data_lock:
            currents = list(msg.data)
            for i in range(min(5, len(currents))):
                self.currents[i] = currents[i]
    
    def present_velocity_callback(self, msg: Float32MultiArray):
        with self.data_lock:
            velocities = list(msg.data)
            for i in range(min(5, len(velocities))):
                self.velocities[i] = velocities[i]
    
    def timer_callback(self):
        """Timer callback for periodic data display (0.1Hz)"""
        with self.data_lock:
            # Clear terminal and print data
            print("\033[2J\033[H", end="")  # Clear terminal
            
            print(" ==================== MONITORING HUB ==================== ")
            print(" ")
            
            # Joint information
            print(" ====== JOINT STATUS ======")
            for i in range(5):
                print(f" Joint #{i+1:1d} : Pulse: {self.joint_pulses[i]:6d} | "
                      f"DEG: {self.joint_degrees[i]:7.2f} | "
                      f"Current: {self.currents[i]:6.3f}mA | "
                      f"Velocity: {self.velocities[i]:7.2f}rpm")
            print(" ")
            
            # Gripper pose
            print(" ====== GRIPPER POSE ======")
            print()
            for i in range(4):
                row = " ".join(f"{self.gripper_pose[i,j]:8.4f}" for j in range(4))
                print(f" {row}")
            print()
            
            # Camera pose
            print(" ====== CAMERA POSE ======")
            print()
            for i in range(4):
                row = " ".join(f"{self.camera_pose[i,j]:8.4f}" for j in range(4))
                print(f" {row}")
            print()
            
            print(" ==================== MONITORING HUB ==================== ")


def main(args=None):
    rclpy.init(args=args)
    node = MonitoringHub()
    
    # Use MultiThreadedExecutor for thread separation
    exec = MultiThreadedExecutor(num_threads=4)
    exec.add_node(node)
    
    try:
        exec.spin()
    except KeyboardInterrupt:
        print("\n\nShutting down Monitoring Hub...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()