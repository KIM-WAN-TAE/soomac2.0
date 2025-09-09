#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from zeus_interfaces.srv import ZeusExecutor

class ZeusClientNode(Node):
    def __init__(self):
        super().__init__('zeus_client_ndoe')
        self.cli = self.create_client(ZeusExecutor, '/zeus_exec')
        self.create_subscription(Float32MultiArray, '/zeus', self.target_callback, 10)
        
    def target_callback(self):
        while not self.cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('[ZEUS] Waiting For Service Node ... ')
            
        req = ZeusExecutor.Request()
        req.frame = 'L'
        req.coordinate = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        future = self.cli.call_async(req)
        future.add_done_callback(self.resp)
        
    def response_callback(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().warn(f'[ZEUS] Service Fail : {e}')
            return
        
        if res.success:
            self.get_logger().info('[ZEUS] Service Success')
        
        else:
            self.get_logger().warn('[ZEUS] Service Fail ')
            
def main(args=None):
    rclpy.init(args=args)
    node = ZeusClientNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()