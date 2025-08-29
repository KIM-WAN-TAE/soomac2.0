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

def plan_joint_trajectory(q_start, q_end, steps=80, traj_type='linear'):
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
        
        self.create_service(DongSooExecutor, 'dongsoo_excutor', self.service_callback, callback_group = self.srv_cb_group)
    
    # Topic으로 Gripper Pose 받아오는 코드
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
            q_list = plan_joint_trajectory(q_start, q_end, steps=10000, traj_type='linear')
            
            import time
            # 이거 쏘기 전에 rad -> pulse 로 변환 전부 해야함
            for i, q_s in enumerate(q_list):
                q_msg.data = q_s.tolist()
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