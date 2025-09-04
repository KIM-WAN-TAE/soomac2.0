#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from dongsoo_interfaces.msg import DongSooCommand
import numpy as np

def rad_to_pulse(radian):
    return int(radian * (4096 / (2 * np.pi)) + 2048)

def pos_as_T(P):
    T = np.eye(4)
    T[:3, 3] = np.asarray(P, float).reshape(3)
    return T
    

class CommandCentorNode(Node):
    def __init__(self):
        super().__init__('command_centor_node')
        self.get_logger().info('Command Centor is Ready!')
        
        self.block_coor_sub = self.create_subscription(
            Float32MultiArray,
            '/info/array/target_obj_array',
            self.block_coordinate_callback,
            10)
        
        self.camera_mat_sub = self.create_subscription(
            Float32MultiArray,
            '/info/matrix/camera',
            self.camera_mat_callback,
            10)
        self.cam_mat = np.eye(4)
        
        self.target_pose_pub = self.create_publisher(
            DongSooCommand,
            '/command/array/pose',
            10)

        self.yaw_pose_pub    = self.create_publisher(
            Int32MultiArray,
            '/motor/command_dxl5_position',
            10)
        
    def camera_mat_callback(self, msg : Float32MultiArray):
        dims = msg.layout.dim
        
        if len(dims) < 2:
            self.get_logger().warn(' 잘못된 행렬 수신 ')
            return

        rows = dims[0].size
        cols = dims[1].size
        
        if len(msg.data) != rows * cols:
            self.get_logger().warn(f' msg count error : msg_count : {rows*cols}')
            return
        
        self.cam_mat = np.asarray(msg.data, dtype=np.float32).reshape(rows, cols)
    
    def block_coordinate_callback(self, msg : Float32MultiArray):
        if len(msg.data) < 4:
            self.get_logger().warn('Wrong Data length')
            return
        
        position, yaw = msg.data[:3], msg.data[3]
        self.get_logger().info(f'Position: [{position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}], Yaw: {yaw:.3f}')
        
        obj_mat = pos_as_T(position)
        
        base_2_obj = self.cam_mat @ obj_mat
        
        obj_pos = base_2_obj[:3,3]

        while True:
            ans = input('y를 누르면 전송합니다 : ')
            if ans.lower() == 'y':
                tar_msg = DongSooCommand()
                tar_msg.position = obj_pos.astype(np.float32).tolist()
                tar_msg.look = 'down'
                
                yaw_msg = Int32MultiArray()
                yaw_msg.data = [rad_to_pulse(yaw)]
                
                self.target_pose_pub.publish(tar_msg)
                self.yaw_pose_pub.publish(yaw_msg)
                break

def main(args=None):
    rclpy.init(args=args)
    node = CommandCentorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt로 종료')
    finally:
        # 안전한 종료
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()