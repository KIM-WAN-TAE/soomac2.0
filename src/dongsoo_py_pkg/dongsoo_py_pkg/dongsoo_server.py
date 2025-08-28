#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from dongsoo_interfaces.srv import DongSooExecutor
from std_msgs.msg import Float32MultiArray
from rclpy.callback_groups import ReentrantCallbackGroup
from dongsoo_py_pkg.Inverse_Kinematics import get_ik_result
import numpy as np

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
        
        self.motor_control_pub = self.create_publisher(Float32MultiArray, '/motor/command_position', 10)
        
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
                q_list = get_ik_result(start_point, end_point, mode='down', w_ori=0.2)
            
            elif end_look == 'straight':
                q_list = get_ik_result(start_point, end_point, mode='down', w_ori=0.2)
                
            q_msg = Float32MultiArray()
            
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