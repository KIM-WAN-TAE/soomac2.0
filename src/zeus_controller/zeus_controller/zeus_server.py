#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from zeus_interfaces.srv import ZeusExecutor

import numpy as np
import threading, time 

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
    
    elif frame == 't' or frame == 'T':
        values_str = ",".join([f"{v:.4f}" for v in arr])
        cmd = f"tool_move+{values_str}"
        return cmd
    
    else:
        raise ValueError("Wrong Frame")

class ZeusServerNode(Node):
    def __init__(self):
        super().__init__('zeus_server_node')
        self.get_logger().info('[ZEUS] Server Node On!')
        
        self.service_cb_group = ReentrantCallbackGroup()
        self.sub_cb_group = ReentrantCallbackGroup()
        
        self.lock = threading.Lock()
        
        self.srv = self.create_service(ZeusExecutor, '/zeus_exec', self.handler, callback_group=self.service_cb_group)
        self.create_subscription(Float32MultiArray, '/zeus/array/xy_state', self.xy_state_callback, 10, callback_group=self.sub_cb_group)
        self.create_subscription(Float32MultiArray, '/zeus/array/joint_state', self.joint_state_callback, 10, callback_group=self.sub_cb_group)
        self.command_pub = self.create_publisher(String, '/zeus/string/binary_command', 10)
        
        self.xy_coor = np.zeros(6, dtype=np.float32)
        self.joint_coor = np.zeros(6, dtype=np.float32)   
        
        self.tol_ang = 0.5
        self.tol_pos = 1.0
        
    def xy_state_callback(self, msg):
        with self.lock:
            if len(msg.data) == 6:
                self.xy_coor = msg.data
         # self.get_logger().info(f'[ZEUS] xyz_coor: {self.xy_coor}')

    def joint_state_callback(self, msg):
        with self.lock:
            if len(msg.data) == 6:
                self.joint_coor = msg.data
        # self.get_logger().info(f'[ZEUS] joint_coor: {self.joint_coor}')

            
    def handler(self, req, res):
        res.success = False
        try:
            frame = req.frame
            goal_coor = np.array(req.coordinate)

            com_str = command_string(frame, goal_coor)
            
            com_msg = String()
            com_msg.data = com_str
            self.command_pub.publish(com_msg)
            
            target = None
            if frame.lower() == 't':
                with self.lock:
                    current = np.array(self.xy_coor)
                    target = current - goal_coor  # 상대좌표
            
            while True:
                with self.lock:
                    if frame.lower() == 'l':
                        current = np.array(self.xy_coor)
                        error = np.linalg.norm(goal_coor - current)
                        self.get_logger().info(f'Linear error : {error}')
                        
                        if error < self.tol_pos:
                            res.success = True
                            break
                        
                    elif frame.lower() == 'j':
                        current = np.array(self.joint_coor)
                        error = np.linalg.norm(goal_coor - current)
                        self.get_logger().info(f'Joint error : {error}')
                        
                        if error < self.tol_ang:
                            res.success = True
                            break
                        
                    elif frame.lower() == 't':
                        current = np.array(self.xy_coor)
                        error = np.linalg.norm(target - current)
                        self.get_logger().info(f'target : {target}, Toolmove error : {error}')
                        if error < self.tol_pos:
                            res.success = True
                            break
                        
                time.sleep(0.05)

        except Exception as e:
            self.get_logger().error(f'[ZEUS SERVER] Send Coordinate Fail : {e}')
            
        return res
        
def main(args=None):
    rclpy.init(args=args)
    node = ZeusServerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()