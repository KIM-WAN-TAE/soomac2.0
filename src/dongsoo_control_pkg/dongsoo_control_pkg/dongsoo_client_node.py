#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dongsoo_interfaces.srv import DongSooExecutor
from dongsoo_interfaces.msg import DongSooCommand
import numpy as np

RATE = 10

class DongsooClient(Node):
    def __init__(self):
        super().__init__('dongsoo_client')
        self.get_logger().info('[AIOT] DongSoo Service Client On! ')
        
        self.dongsoo_client = self.create_client(DongSooExecutor, 'dongsoo_executor')
        
        self.create_subscription(DongSooCommand, '/aiot/array/command_pose',
                                 self.target_pose_callback, 10)
        
        self.target_position = np.array([])
        self.target_look  = None
        self.target_time  = None
        self.target_wrist = None
        self.target_flag  = False
        
        timer_period = 1/RATE 
        self.client_timer = self.create_timer(timer_period, self.client_timer_callback)
        
    def target_pose_callback(self, msg : DongSooCommand):
        P     = np.array(msg.position, dtype=np.float32)
        LOOK  = msg.look
        TIME  = msg.time
        WRIST = msg.wrist

        self.target_position = P
        self.target_look     = LOOK
        self.target_time     = TIME
        self.target_wrist    = WRIST
        self.target_flag     = True
        
    def client_timer_callback(self):
        self.get_logger().info(f'[AIOT] Flag State : {self.target_flag}')

        if not self.target_flag:
            self.get_logger().info(' Waiting Target Pose... ')
            return

        if self.target_flag and self.target_position.size != 0:
            copy_target = self.target_position
            copy_look   = self.target_look
            copy_time   = self.target_time
            copy_wrist  = self.target_wrist
            self.target_flag = False
            self.send_next_pose(copy_target, copy_look, copy_time, copy_wrist)
                
    def send_next_pose(self, position, look, time, wrist):
        if position.size < 3:
            self.get_logger().warn(' Wrong Array Size Target Pose')
            return
        
        req = DongSooExecutor.Request()
        req.position = position
        req.look     = look
        req.time     = time
        req.wrist    = wrist
                
        future = self.dongsoo_client.call_async(req)
        future.add_done_callback(self.response_callback)
        
    def response_callback(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f'[AIOT] 서비스 호출 실패: {e}')
            return

        if res.success:
            self.get_logger().info(f'[AIOT] 동작 성공 ')
        else:
            self.get_logger().warn(f'[AIOT] 동작 실패 ')
            
        self.target_flag = False
        
def main(args=None):
    rclpy.init(args=args)
    node = DongsooClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nShutting down Dongsoo Service Client...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()