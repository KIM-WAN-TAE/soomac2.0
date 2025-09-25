#!/usr/bin/env python3

import os, sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from dongsoo_interfaces.srv import DongSooExecutor
from dongsoo_interfaces.msg import DongSooCommand
from std_msgs.msg import Float32MultiArray
import numpy as np

class DongsooClient(Node):
    def __init__(self):
        super().__init__('dongsoo_client')
        self.get_logger().info(' DongSoo Service Client On! ')
        
        self.dongsoo_client = self.create_client(DongSooExecutor, 'dongsoo_executor')
        
        self.target_co_sub = self.create_subscription(
            DongSooCommand,
            '/command/array/pose',
            self.target_pose_callback,
            10)
        
        self.target_Position = np.array([])
        self.target_look = None
        self.target_time = None
        self.target_flag = False
        
        timer_period = 1/10
        self.client_timer = self.create_timer(timer_period, self.client_timer)
        
    def target_pose_callback(self, msg: DongSooCommand):
        P = np.array(msg.position, dtype=np.float32)
        LOOK = msg.look
        TIME = msg.time
        
        self.target_Position= P
        self.target_look = LOOK
        self.target_time = TIME
        self.target_flag = True
        
    def client_timer(self):
        print(f' Flag State : {self.target_flag}')
        if not self.target_flag:
            self.get_logger().info(' Waiting Target Pose... ')
            return
        
        # os.system('clear')
        while self.target_flag:
            if self.target_flag and self.target_Position.size != 0:
                copy_target = self.target_Position
                copy_look   = self.target_look
                copy_time   = self.target_time
                self.target_flag = False
                self.send_next_pose(copy_target, copy_look, copy_time)
        
    def send_next_pose(self, position, look, time):
        if position.size < 3:
            self.get_logger().warn(' Wrong Array Size Target Pose')
            return
        
        req = DongSooExecutor.Request()
        req.position = position
        req.look     = look
        req.time     = time
                
        future = self.dongsoo_client.call_async(req)
        future.add_done_callback(self.response_callback)
        
    def response_callback(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f'서비스 호출 실패: {e}')
            return

        if res.success:
            self.get_logger().info(f' 동작 성공 ')
        else:
            self.get_logger().warn(f' 동작 실패 ')
            
        self.target_flag = False

def main(args=None):
    rclpy.init(args=args)
    node = DongsooClient()
    
    exec = MultiThreadedExecutor(num_threads=2)
    exec.add_node(node)
    
    try:
        exec.spin()
    except KeyboardInterrupt:
        print("\n\nShutting down Dongsoo Service Client...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()