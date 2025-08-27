#!/usr/bin/env python3

import rclpy
import numpy as np
import threading
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray, Int32MultiArray, MultiArrayDimension
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

class DataHub(Node):
    def __init__(self):
        super().__init__('data_hub')
        self.get_logger().info(' DataHub Node On! ')
        
        self.data_lock = threading.Lock()
        
        self.joint_pulses = [0, 0, 0, 0, 0]
        self.joint_degrees = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.q_rad = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.currents = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.velocities = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.gripper_pose = np.eye(4)
        self.camera_pose = np.eye(4)
        
        self.sub_pose = self.create_subscription(
            Int32MultiArray,
            '/motor/position',
            self.present_position_callback,
            10
        )
        
        self.camera_mat_pub = self.create_publisher(Float32MultiArray, '/info/matrix/camera', 10)
        self.gripper_mat_pub = self.create_publisher(Float32MultiArray, '/info/matrix/gripper', 10)
        
        timer_period = 1/10  # 10Hz 
        self.create_timer(timer_period, self.timer_callback)
    
    def fk(self, dh_params):
        T = np.eye(4)
        for params in dh_params:
            T = T @ dh_transform(params['theta'], params['d'], params['a'], params['alpha'])
        return T
    
    def calculate_poses(self, q_rad):
        cam_params = cam_dh.get_all_dh_params(q_rad)
        grip_params = grip_dh.get_all_dh_params(q_rad)
        
        T_cam = self.fk(cam_params)
        T_grip = self.fk(grip_params)
        
        return {
            'camera': T_cam,
            'gripper': T_grip
        }
    
    def present_position_callback(self, msg: Int32MultiArray):
        with self.data_lock:
            pulses = list(msg.data)
            
            for i in range(min(5, len(pulses))):
                self.joint_pulses[i] = pulses[i]
                self.joint_degrees[i] = (pulses[i] / 4096.0) * 360.0
                self.q_rad[i] = np.deg2rad(self.joint_degrees[i])
            
            poses = self.calculate_poses(self.q_rad)
            self.camera_pose = poses['camera']
            self.gripper_pose = poses['gripper']
    
    def timer_callback(self):
        with self.data_lock:
            cam_mat = np.array(self.camera_pose, dtype=np.float32)
            grip_mat = np.array(self.gripper_pose, dtype=np.float32)
            rows, cols = cam_mat.shape
            
            c_msg = Float32MultiArray()
            c_msg.layout.dim.append(MultiArrayDimension(label='rows', size=rows, stride=cols))
            c_msg.layout.dim.append(MultiArrayDimension(label='cols', size=cols, stride=1))
            c_msg.layout.data_offset = 0
            c_msg.data = cam_mat.flatten().tolist()
            self.camera_mat_pub.publish(c_msg)
            
            g_msg = Float32MultiArray()
            g_msg.layout.dim.append(MultiArrayDimension(label='rows', size=rows, stride=cols))
            g_msg.layout.dim.append(MultiArrayDimension(label='cols', size=cols, stride=1))
            g_msg.layout.data_offset = 0
            g_msg.data = grip_mat.flatten().tolist()
            self.gripper_mat_pub.publish(g_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DataHub()
    
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