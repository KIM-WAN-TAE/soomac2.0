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
    [23.03,  533.22,  -18.54, -179.67,   -0.12,  179.86],
    [98.19,  533.33,  -18.54, -179.58,   -0.10,  179.79],
    [-29.80,  494.88,  -18.54,  -90.36,    0.34,  179.95],
    [22.66,  455.47,  -18.54, -179.64,   -0.09,  179.80],
    [98.07,  455.56,  -18.54, -179.46,   -0.07,  179.78],
    [-29.76,  417.90,  -18.54,  -90.20,    0.28, -179.98],
    [22.28,  381.60,  -18.54, -179.38,   -0.05,  179.84],
    [99.02,  381.90,  -18.54, -179.22,   -0.03,  179.72],
    [204.38,  619.66,  -18.54,  -91.79,    0.33, -179.69]
]

# 7개
GREEN_DROP_LIST = [
    [172.82,  505.78,  -18.54,  -90.34,    0.22,  179.93],
    [172.82,  431.70,  -18.54,  -90.18,    0.10,  179.95],
    [198.06,  382.33,  -18.54,  178.87,    0.03,  179.69],
    [274.28,  382.50,  -18.54,  178.91,    0.09,  179.61],
    [297.35,  431.82,  -18.54,  -90.08,    0.20,  179.99],
    [297.31,  507.42,  -18.54,  -90.06,    0.25,  179.97],
    [232.63,  619.67,  -18.54,  -91.86,    0.31, -179.67]
]

# 8개
PINK_DROP_LIST = [
    [397.84,  532.26,  -18.54,  178.95,    0.05,  179.60],
    [472.69,  532.14,  -18.54,  178.90,   -0.02,  179.74],
    [344.76,  505.80,  -18.54,  -90.20,    0.36, -179.97],
    [370.09,  457.63,  -18.54,  178.77,    0.01,  179.73],
    [445.85,  457.56,  -18.54,  178.65,   -0.04,  179.83],
    [494.29,  417.22,  -18.54,  -90.20,    0.42, -179.82],
    [370.08,  383.11,  -18.54,  178.61,    0.03,  179.77],
    [443.74,  383.17,  -18.54,  178.67,    0.06,  179.74],
]

#7개
PURPLE_DROP_LIST = [
    [-172.97,  529.87,  -18.54, -179.79,   -0.22,  179.81],
    [-99.24,  529.89,  -18.54, -179.77,   -0.16,  179.84],
    [-113.42,  481.34,  -18.54, -136.05,   -0.05,  179.90],
    [-166.42,  429.10,  -18.54, -136.11,   -0.03,  179.84],
    [-172.16,  381.06,  -18.54, -179.80,   -0.08,  179.96],
    [-95.98,  381.23,  -18.54, -179.59,   -0.10,  179.88],
    [180.67,  619.65,  -18.54,  -91.77,    0.34, -179.71],
]

#4개
RED_DROP_LIST = [
    [286.73,  755.01,  -18.54, -136.26,    0.35, -179.79],
    [321.11,  722.37,  -18.54,  -91.74,    0.06, -179.62],
    [321.02,  646.64,  -18.54,  -91.92,    0.08, -179.65],
    [321.34,  598.55,  -18.54,  179.75,    0.31,  179.98],
]

#6개
YELLOW_DROP_LIST = [
    [-7.77,  615.44,  -18.54,  -90.08,    0.33, -179.81],
    [-7.76,  687.87,  -18.54,  -90.00,    0.31, -179.76],
    [33.84,  686.88,  -18.54,  -59.91,    0.08, -179.65],
    [71.36,  619.51,  -18.54,  -60.07,    0.18, -179.63],
    [115.85,  619.62,  -18.54,  -90.04,    0.30, -179.74],
    [115.91,  694.83,  -18.54,  -89.94,    0.16, -179.70],
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
        
        if msg.data.strip().lower() == 'done':
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
                    
        elif msg.data.strip().lower() == 'again':
            with self.lock:
                if self.block_color == 'blue':
                    if self.blue_count < len(BLUE_DROP_LIST):
                        out = Float32MultiArray()
                        out.data = BLUE_DROP_LIST[self.blue_count]
                        self.block_coor_pub.publish(out)
                        self.get_logger().info(f'Published Drop Point (blue #{self.blue_count}): {out.data}')
                    else:
                        self.get_logger().warn('No more BLUE_DROP_LIST points left')
                    self.block_color = None

                elif self.block_color == 'green':
                    if self.green_count < len(GREEN_DROP_LIST):
                        out = Float32MultiArray()
                        out.data = GREEN_DROP_LIST[self.green_count]
                        self.block_coor_pub.publish(out)
                        self.get_logger().info(f'Published Drop Point (green #{self.green_count}): {out.data}')
                    else:
                        self.get_logger().warn('No more GREEN_DROP_LIST points left')
                    self.block_color = None


                elif self.block_color == 'pink':
                    if self.pink_count < len(PINK_DROP_LIST):
                        out = Float32MultiArray()
                        out.data = PINK_DROP_LIST[self.pink_count]
                        self.block_coor_pub.publish(out)
                        self.get_logger().info(f'Published Drop Point (pink #{self.pink_count}): {out.data}')
                    else:
                        self.get_logger().warn('No more PINK_DROP_LIST points left')
                    self.block_color = None

                elif self.block_color == 'purple':
                    if self.purple_count < len(PURPLE_DROP_LIST):
                        out = Float32MultiArray()
                        out.data = PURPLE_DROP_LIST[self.purple_count]
                        self.block_coor_pub.publish(out)
                        self.get_logger().info(f'Published Drop Point (purple #{self.purple_count}): {out.data}')
                    else:
                        self.get_logger().warn('No more PURPLE_DROP_LIST points left')
                    self.block_color = None

                elif self.block_color == 'red':
                    if self.red_count < len(RED_DROP_LIST):
                        out = Float32MultiArray()
                        out.data = RED_DROP_LIST[self.red_count]
                        self.block_coor_pub.publish(out)
                        self.get_logger().info(f'Published Drop Point (red #{self.red_count}): {out.data}')
                    else:
                        self.get_logger().warn('No more RED_DROP_LIST points left')
                    self.block_color = None

                elif self.block_color == 'yellow':
                    if self.yellow_count < len(YELLOW_DROP_LIST):
                        out = Float32MultiArray()
                        out.data = YELLOW_DROP_LIST[self.yellow_count]
                        self.block_coor_pub.publish(out)
                        self.get_logger().info(f'Published Drop Point (yellow #{self.yellow_count}): {out.data}')
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