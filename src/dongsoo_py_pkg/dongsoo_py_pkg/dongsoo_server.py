#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from dongsoo_interfaces.srv import DongSooExecutor
from std_msgs.msg import Float32MultiArray, Int32MultiArray 
from rclpy.callback_groups import ReentrantCallbackGroup
from dongsoo_py_pkg.Inverse_Kinematics import get_ik_result
import numpy as np

def rad_to_pulse(rad_values):
    pulse_values = []
    for rad_val in rad_values:
        pulse_val = int((rad_val * 4096.0 / (2 * np.pi)) + 2048)
        pulse_values.append(pulse_val)
    return pulse_values

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
        
        self.motor_control_pub = self.create_publisher(Int32MultiArray, '/motor/command_position', 10)
        
        self.present_position = np.array([])
        self.present_orientation = np.array([])
        
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
            
    def service_callback(self, req, response):
        try:
            start_point = self.present_position
            end_point = req.position
            end_look  = req.look
            
            if end_look == 'down':
                q_result = get_ik_result(start_point, end_point, mode='down', w_ori=0.2)
            
            elif end_look == 'straight':
                q_result = get_ik_result(start_point, end_point, mode='straight', w_ori=0.2)
                
            q_start = q_result['q_start']
            q_end   = q_result['q_end']
            self.get_logger().info(f'[Q_list] : {q_end}')
            
            sleep_time = 0.01
            
            q_msg = Int32MultiArray()
            q_list = plan_joint_trajectory(q_start, q_end, steps=10000, traj_type='smooth')
            
            import time
            # rad -> pulse 변환 함수 사용
            for i, q_s in enumerate(q_list):
                q_pulse = rad_to_pulse(q_s)
                q_msg.data = q_pulse
                self.motor_control_pub.publish(q_msg)
                time.sleep(sleep_time)
            
            response.success = True

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