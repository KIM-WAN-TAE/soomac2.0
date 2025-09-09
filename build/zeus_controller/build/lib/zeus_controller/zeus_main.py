#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ZeusMainNode(Node):
    def __init__(self):
        super().__init__('zeus_main_node')
        
        self.state_sub = self.create_subscription(
            String,
            '/zeus/string/binary_responed',
            self.respond_callback,
            10
        )
    
    def respond_callback(self, msg):
        self.get_logger().info(f'Received response: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = ZeusMainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()