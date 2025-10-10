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
    [-2.84,  -39.56, -107.46,    0.40,  -33.17,  -93.11],
    [-11.82,  -41.38, -102.98,    0.32,  -35.88, -102.00],
    [-3.32,  -34.88, -119.76,    0.50,  -25.56,  -93.71],
    [-13.71,  -36.72, -114.90,    0.37,  -28.61, -103.97],
    [-4.00,  -30.98, -131.21,    0.69,  -18.00,  -94.59],
    [-16.26,  -32.83, -125.72,    0.45,  -21.70, -106.61],
]

# 7개
GREEN_DROP_LIST = [
    [-19.89,  -42.17, -101.08,    0.25,  -37.00,  -20.03],
    [-23.28,  -37.38, -113.23,    0.28,  -29.65,  -23.45],
    [-30.94,  -47.79,  -87.51,    0.15,  -44.97,  -30.97],
    [-35.32,  -42.98,  -99.14,    0.13,  -38.16,  -35.35],
    [-32.63,  -37.76, -112.30,    0.18,  -30.23, -122.72],
    [-6.36,  -51.12,  -79.48,    0.27,  -49.62,   -6.46]
]

# 8개
PINK_DROP_LIST = [
    [-36.43,  -54.77,  -71.11,    0.09,  -54.40, -126.41],
    [-41.57,  -61.78,  -54.99,    0.05,  -63.52, -131.53],
    [-38.67,  -48.33,  -86.25,    0.08,  -45.71, -128.67],
    [-44.07,  -54.15,  -72.56,    0.05,  -53.58, -134.04],
    [-43.32,  -44.46,  -95.54,    0.06,  -40.28, -133.30],
    [-48.76,  -50.16,  -81.92,    0.02,  -48.21, -138.70],
]

#7개
PURPLE_DROP_LIST = [
    [19.14,  -38.72, -109.35,    0.52,  -32.03,  -71.23],
    [10.03,  -38.44, -110.18,    0.49,  -31.52,  -80.32],
    [14.18,  -34.91, -119.48,    0.61,  -25.73, -121.31],
    [27.40,  -30.25, -133.13,    1.00,  -16.68,  -63.50],
    [14.43,  -29.96, -134.31,    0.98,  -15.85,  -76.43],
    [-10.50,  -52.45,  -76.38,    0.24,  -51.39,  -10.59]
]

#4개
RED_DROP_LIST = [
    [-18.42,  -53.64,  -73.66,    0.20,  -52.95, -108.47],
    [-16.97,  -59.43,  -60.21,    0.19,  -60.61,  -16.99],
]

#6개
YELLOW_DROP_LIST = [
    [-0.62,  -57.45,  -64.68,    0.27,  -58.07,   -0.69],
    [-0.69,  -49.53,  -83.18,    0.30,  -47.49,   -0.82],
    [4.98,  -54.49,  -71.47,    0.30,  -54.20,  -25.13],
    [9.17,  -55.83,  -68.32,    0.30,  -56.00,    9.06],
    [10.35,  -48.23,  -86.14,    0.36,  -45.77,   10.17],
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