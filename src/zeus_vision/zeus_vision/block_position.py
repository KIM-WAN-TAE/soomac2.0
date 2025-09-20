#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading

BLUE_DROP_LIST = [
    [1.2, -44.19, -105.30, 0.19, -30.46, -91.38],
    [9.44, -43.74, -106.58, 0.19, -29.60, -83.06],
    [15.83, -41.82, -112.86, 0.19, -25.51, -165.04],
    [1.27, -40.0, -117.84, 0.17, -22.04, -89.60],
]

GREEN_DROP_LIST = [
    [-7.57, -42.03, -105.41, 0.18, -32.51, -7.31],
    [-8.81, -37.41, -117.40, 0.17, -24.75, -8.49],
    [-13.01, -35.64, -123.64, 0.38, -20.67, -102.28],
]

class MainDropLogic(Node):
    def __init__(self):
        super().__init__('main_drop_logic')

        self.lock = threading.Lock()
        self.cb_group = ReentrantCallbackGroup()

        # qos_profile 인자 추가
        self.create_subscription(
            String,
            '/zeus/string/block_color',
            self.block_color_callback,
            10,
            callback_group=self.cb_group
        )
        self.create_subscription(
            String,
            '/zeus/string/drop_done',
            self.drop_flag_callback,
            10,
            callback_group=self.cb_group
        )

        self.block_coor_pub = self.create_publisher(
            Float32MultiArray,
            '/zeus/array/drop_point',
            10
        )

        self.block_color = None
        self.blue_count = 0
        self.green_count = 0

    def block_color_callback(self, msg: String):
        with self.lock:
            self.block_color = msg.data.strip().lower()

    def drop_flag_callback(self, msg: String):
        if msg.data.strip().lower() != 'done':
            return

        with self.lock:
            if self.block_color == 'blue':
                if self.blue_count < len(BLUE_DROP_LIST):
                    out = Float32MultiArray()
                    out.data = BLUE_DROP_LIST[self.blue_count]
                    self.block_coor_pub.publish(out)
                    self.get_logger().info(f'Published Drop Point (blue #{self.blue_count}): {out.data}')
                    self.blue_count += 1
                else:
                    self.get_logger().warn('No more BLUE_DROP_LIST points left')
                self.block_color = None

            elif self.block_color == 'green':
                if self.green_count < len(GREEN_DROP_LIST):
                    out = Float32MultiArray()
                    out.data = GREEN_DROP_LIST[self.green_count]
                    self.block_coor_pub.publish(out)
                    self.get_logger().info(f'Published Drop Point (green #{self.green_count}): {out.data}')
                    self.green_count += 1
                else:
                    self.get_logger().warn('No more GREEN_DROP_LIST points left')
                self.block_color = None

def main(args=None):
    rclpy.init(args=args)
    node = MainDropLogic()
    try:
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()