#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading

# 9개
BLUE_DROP_LIST = [
    [62.26, 535.79, 30.95, 176.06, -0.09, -179.92],
    [169.89,  535.68,  30.84,  175.95,   -0.11, -179.89],
    [280.85,  535.63,  30.68,  175.84,   -0.15, -179.90],
    [409.03,  535.63,  30.50,  175.87,   -0.19, -179.92],
    [524.50,  535.71,  30.29,  175.73,   -0.20,  180.00],
    [56.22,  398.28,  30.15,  176.10,   -0.08, -179.90],
    [166.53,  398.29,  30.15,  176.10,   -0.08, -179.90],
    [283.06,  398.18,  30.14,  176.02,   -0.12, -179.90],
    [396.32,  397.95,  30.09,  175.99,   -0.21, -179.85]
]

# 7개
GREEN_DROP_LIST = [
    [62.26, 535.79, 30.95, 176.06, -0.09, -179.92],
    [169.89,  535.68,  30.84,  175.95,   -0.11, -179.89],
    [280.85,  535.63,  30.68,  175.84,   -0.15, -179.90],
    [409.03,  535.63,  50.50,  175.87,   -0.19, -179.92],
    [524.50,  535.71,  30.29,  175.73,   -0.20,  180.00],
    [56.22,  398.28,  30.15,  176.10,   -0.08, -179.90],
    [166.53,  398.29,  30.15,  176.10,   -0.08, -179.90]
]

# 8개
PINK_DROP_LIST = [
    [62.26, 535.79, 30.95, 176.06, -0.09, -179.92],
    [169.89,  535.68,  30.84,  175.95,   -0.11, -179.89],
    [280.85,  535.63,  30.68,  175.84,   -0.15, -179.90],
    [409.03,  535.63,  30.30,  175.87,   -0.19, -179.92],
    [524.30,  535.71,  30.29,  175.73,   -0.20,  180.00],
    [56.22,  398.28,  30.15,  176.10,   -0.08, -179.90],
    [166.53,  398.29,  30.15,  176.10,   -0.08, -179.90],
    [283.06,  398.18,  30.14,  176.02,   -0.12, -179.90],
]

#7개
PURPLE_DROP_LIST = [
    [62.26, 535.79, 30.95, 176.06, -0.09, -179.92],
    [169.89,  535.68,  30.84,  175.95,   -0.11, -179.89],
    [280.85,  535.63,  30.68,  175.84,   -0.15, -179.90],
    [409.03,  535.63,  30.30,  175.87,   -0.19, -179.92],
    [524.30,  535.71,  30.29,  175.73,   -0.20,  180.00],
    [56.22,  398.28,  30.15,  176.10,   -0.08, -179.90],
    [166.53,  398.29,  30.15,  176.10,   -0.08, -179.90],
]

#4개
RED_DROP_LIST = [
    [62.26, 535.79, 30.95, 176.06, -0.09, -179.92],
    [169.89,  535.68,  30.84,  175.95,   -0.11, -179.89],
    [280.85,  535.63,  30.68,  175.84,   -0.15, -179.90],
    [409.03,  535.63,  30.30,  175.87,   -0.19, -179.92],
]

#6개
YELLOW_DROP_LIST = [
    [62.26, 535.79, 30.95, 176.06, -0.09, -179.92],
    [169.89,  535.68,  30.84,  175.95,   -0.11, -179.89],
    [280.85,  535.63,  30.68,  175.84,   -0.15, -179.90],
    [409.03,  535.63,  30.30,  175.87,   -0.19, -179.92],
    [524.30,  535.71,  30.29,  175.73,   -0.20,  180.00],
    [56.22,  398.28,  30.15,  176.10,   -0.08, -179.90],
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
        self.pink_count = 0
        self.purple_count = 0
        self.red_count = 0
        self.yellow_count = 0

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


            elif self.block_color == 'pink':
                if self.pink_count < len(PINK_DROP_LIST):
                    out = Float32MultiArray()
                    out.data = PINK_DROP_LIST[self.pink_count]
                    self.block_coor_pub.publish(out)
                    self.get_logger().info(f'Published Drop Point (pink #{self.pink_count}): {out.data}')
                    self.pink_count += 1
                else:
                    self.get_logger().warn('No more PINK_DROP_LIST points left')
                self.block_color = None

            elif self.block_color == 'purple':
                if self.purple_count < len(PURPLE_DROP_LIST):
                    out = Float32MultiArray()
                    out.data = PURPLE_DROP_LIST[self.purple_count]
                    self.block_coor_pub.publish(out)
                    self.get_logger().info(f'Published Drop Point (purple #{self.purple_count}): {out.data}')
                    self.purple_count += 1
                else:
                    self.get_logger().warn('No more PURPLE_DROP_LIST points left')
                self.block_color = None

            elif self.block_color == 'red':
                if self.red_count < len(RED_DROP_LIST):
                    out = Float32MultiArray()
                    out.data = RED_DROP_LIST[self.red_count]
                    self.block_coor_pub.publish(out)
                    self.get_logger().info(f'Published Drop Point (red #{self.red_count}): {out.data}')
                    self.red_count += 1
                else:
                    self.get_logger().warn('No more RED_DROP_LIST points left')
                self.block_color = None

            elif self.block_color == 'yellow':
                if self.yellow_count < len(YELLOW_DROP_LIST):
                    out = Float32MultiArray()
                    out.data = YELLOW_DROP_LIST[self.yellow_count]
                    self.block_coor_pub.publish(out)
                    self.get_logger().info(f'Published Drop Point (yellow #{self.yellow_count}): {out.data}')
                    self.yellow_count += 1
                else:
                    self.get_logger().warn('No more YELLOW_DROP_LIST points left')
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