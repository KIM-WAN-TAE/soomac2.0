#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading

# z offset 15cm인 joint 좌표(10.27)
# 6개
BLUE_DROP_LIST = [
    [-2.75,  -30.09, -104.36,    0.13,  -45.61,  -92.95],
    [-11.67,  -32.37,  -99.96,    0.11,  -47.74, -101.85],
    [-3.11,  -23.80, -116.20,    0.15,  -40.06,  -93.33],
    [-13.56,  -26.37, -111.46,    0.12,  -42.24, -103.75],
    [-3.62,  -17.59, -127.16,    0.16,  -35.31,  -93.86],
    [-15.98,  -20.63, -121.93,    0.13,  -37.52, -106.19],
]

# 6개
GREEN_DROP_LIST = [
    [-19.66,  -33.38,  -98.00,    0.10,  -48.71,  -19.84],
    [-23.02,  -27.29, -109.75,    0.10,  -43.04,  -23.20],
    [-30.77,  -39.86,  -84.28,    0.07,  -55.95,  -30.91],
    [-35.11,  -34.15,  -95.70,    0.06,  -50.25,  -35.26],
    [-32.39,  -27.53, -108.43,    0.08,  -44.14, -122.55],
    [-10.38,  -45.60,  -72.83,    0.10,  -61.64,  -10.53]
]

# 6개
PINK_DROP_LIST = [
    [-36.34,  -47.99,  -67.56,    0.05,  -64.55, -126.47],
    [-41.41,  -55.74,  -51.17,    0.04,  -73.20, -131.53],
    [-38.64,  -40.48,  -83.05,    0.05,  -56.57, -128.77],
    [-44.05,  -47.20,  -69.22,    0.04,  -63.68, -134.17],
    [-43.32,  -35.89,  -92.27,    0.04,  -51.94, -133.46],
    [-48.64,  -42.48,  -78.98,    0.03,  -58.65, -138.77],
]

#6개
PURPLE_DROP_LIST = [
    [19.46,  -29.30, -105.80,    0.15,  -44.92,  -70.76],
    [10.22,  -28.89, -106.63,    0.15,  -44.52,  -79.99],
    [14.58,  -24.00, -115.78,    0.17,  -40.25, -120.65],
    [27.60,  -16.54, -128.82,    0.19,  -34.64,  -62.66],
    [14.91,  -15.92, -129.90,    0.19,  -34.21,  -75.36],
    [-6.13,  -44.10,  -76.09,    0.10,  -59.87,   -6.29]
]

#2개
RED_DROP_LIST = [
    [-18.18,  -46.75,  -70.45,    0.08,  -62.88, -108.32],
    [-16.84,  -53.11,  -57.06,    0.08,  -69.91,  -16.98],
    [-12.46,  -66.71,  -27.48,    0.08,  -85.89,   32.43],
]

#5개
YELLOW_DROP_LIST = [
    [-0.39,  -51.56,  -60.45,    0.10,  -68.04,   -0.53],
    [-0.45,  -42.38,  -79.62,    0.11,  -58.05,   -0.62],
    [5.71,  -47.92,  -68.12,    0.11,  -64.01,  -24.44],
    [9.34,  -49.72,  -64.32,    0.11,  -66.00,    9.19],
    [10.64,  -40.90,  -82.99,    0.13,  -56.15,   10.46],
]

# # z offset 15cm인 joint 좌표
# BLUE_DROP_LIST = [
#     [-2.86,  -34.65, -105.91,    0.34,  -39.63,  -93.06],
#     [-11.84,  -36.73, -101.46,    0.28,  -42.05, -101.97],
#     [-3.34,  -29.05, -118.08,    0.40,  -33.07,  -93.61],
#     [-13.73,  -31.29, -113.28,    0.30,  -35.66, -103.91],
#     [-4.03,  -23.89, -129.31,    0.47,  -27.00,  -94.38],
#     [-16.28,  -26.41, -123.93,    0.33,  -29.91, -106.50],
# ]

# # 7개
# GREEN_DROP_LIST = [
#     [-19.90,  -37.62,  -99.57,    0.22,  -43.06,  -20.00],
#     [-23.29,  -32.08, -111.62,    0.23,  -36.55,  -23.41],
#     [-30.95,  -43.85,  -86.02,    0.14,  -50.40,  -30.96],
#     [-35.33,  -38.53,  -97.63,    0.12,  -44.12,  -35.34],
#     [-32.64,  -32.53, -110.70,    0.15,  -37.06, -122.69],
#     [-6.37,  -47.47,  -77.96,    0.25,  -54.79,   -6.45]
# ]

# # 8개
# PINK_DROP_LIST = [
#     [-36.43,  -51.38,  -69.52,    0.08,  -59.37, -126.41],
#     [-41.57,  -58.84,  -53.15,    0.05,  -68.30, -131.53],
#     [-38.67,  -44.44,  -84.75,    0.07,  -51.10, -128.66],
#     [-44.07,  -50.72,  -70.99,    0.05,  -58.58, -134.04],
#     [-43.32,  -40.18,  -94.04,    0.05,  -46.05, -133.29],
#     [-48.76,  -46.43,  -80.41,    0.02,  -53.46, -138.70],
# ]

# #7개
# PURPLE_DROP_LIST = [
#     [19.11,  -33.68, -107.79,    0.44,  -38.62,  -71.16],
#     [10.00,  -33.35, -108.61,    0.41,  -38.18,  -80.25],
#     [14.15,  -29.10, -117.81,    0.48,  -33.21, -121.19],
#     [27.36,  -22.87, -131.20,    0.65,  -25.99,  -63.17],
#     [14.39,  -22.41, -132.34,    0.62,  -25.37,  -76.09],
#     [-10.51,  -48.90,  -74.84,    0.22,  -56.48,  -10.58]
# ]

# #4개
# RED_DROP_LIST = [
#     [-18.43,  -50.17,  -72.10,    0.19,  -57.98, -108.46],
#     [-16.98,  -56.35,  -58.47,    0.18,  -65.43,  -16.98],
# ]

# #6개
# YELLOW_DROP_LIST = [
#     [-0.64,  -54.25,  -63.02,    0.26,  -62.94,   -0.68],
#     [-0.71,  -45.75,  -81.68,    0.28,  -52.77,   -0.80],
#     [4.96,  -51.09,  -69.89,    0.28,  -59.18,  -25.12],
#     [9.15,  -52.52,  -66.71,    0.28,  -60.92,    9.07],
#     [10.33,  -44.34,  -84.65,    0.33,  -51.15,   10.19],
# ]

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