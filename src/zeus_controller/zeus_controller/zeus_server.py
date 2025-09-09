#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from zeus_interfaces.srv import ZeusExecutor

import numpy as np
import threading

def dh_transform(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    
    T = np.array([[ ct, -st*ca,  st*sa, a*ct],
                  [ st,  ct*ca, -ct*sa, a*st],
                  [  0,      sa,     ca,    d],
                  [  0,       0,      0,    1]])
    return T

def command_string(frame, arr):
    if len(arr) != 6:
        raise ValueError("입력 리스트의 길이는 6이어야 합니다.")
    
    if frame == 'j' or frame == 'J':
        values_str = ",".join([f"{v:.4f}" for v in arr])
        cmd = f"move_j_abs+{values_str}"
        return cmd
    
    elif frame == 'l' or frame == 'L':
        values_str = ",".join([f"{v:.4f}" for v in arr])
        cmd = f"move_l_abs+{values_str},6.0"
        return cmd
    
    else:
        raise ValueError("Wrong Frame")

class ZeusServerNode(Node):
    def __init__(self):
        super().__init__('zeus_server_node')
        
        self.lock = threading.Lock()
        
        self.srv = self.create_service(ZeusExecutor, '/zeus_exec', self.handler) 
        
        self.create_subscription(Float32MultiArray, '/zeus/array/xy_state', self.xy_state_callback, 10)
        self.create_subscription(Float32MultiArray, '/zeus/array/joint_state', self.joint_state_callback, 10)
        
        self.command_pub = self.create_publisher(String, '/zeus/string/binary_command', 10)
        
    def calculate_base_to_camera_transform(self, joint_angles):
        all_dh_params = self.dh_reader.get_all_dh_params(joint_angles)
        T_total = np.eye(4)
        
        for params in all_dh_params:
            T_joint = dh_transform(
                params['theta'], params['d'], params['a'], params['alpha'])
            T_total = np.dot(T_total, T_joint)
        
        return T_total
        
    def xy_state_callback(self, msg : Float32MultiArray):
        if len(msg.data) != 6:
            self.get_logger().warn('[ZEUS] Wrong Data length : XYZ State')
            return
        with self.lock:
            self.xy_coor = msg.data
        
    def joint_state_callback(self, msg : Float32MultiArray):
        if len(msg.data) != 6:
            self.get_logger().warn('[ZEUS] Wrong Data length : Joint State')
            return
        with self.lock:
            self.joint_coor = msg.data
            
    def handler(self, req, res):
        try:
            # 여기서 이제 목표를 전송하고 좌표가 도달했는지 feedback 하는 과정
            frame = req.frame
            goal_coor = req.coordinate

            com_str = command_string(frame, goal_coor)
            
            com_msg = String()
            com_msg.data = com_str
            self.command_pub.publish(com_msg)
            
            # 여기서부터 좌표 비교해서 return 하는 로직 추가해야함

        except Exception as e:
            self.get_logger().error(f'[ZEUS SERVER] Send Coordinate Fail : {e}')
            
        return res
        
def main(args=None):
    rclpy.init(args=args)
    node = ZeusServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()