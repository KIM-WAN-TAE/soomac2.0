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
    [-3.45,  -29.47, -103.24,    0.06,  -47.40,  -93.49],
    [-12.40,  -31.89,  -98.65,    0.04,  -49.57, -102.42],
    [-3.91,  -23.13, -115.31,    0.07,  -41.66,  -93.97],
    [-14.28,  -25.83, -110.46,    0.04,  -43.83, -104.31],
    [-4.71,  -17.00, -126.07,    0.07,  -37.04,  -94.77],
    [-16.89,  -20.20, -120.71,    0.03,  -39.21, -106.92],
]

# 6개
GREEN_DROP_LIST = [
    [-20.33,  -32.98,  -96.56,    0.02,  -50.58,  -20.34],
    [-23.78,  -26.96, -108.60,    0.01,  -44.56,  -23.79],
    [-31.38,  -39.87,  -83.16,   -0.01,  -57.08,  -31.38],
    [-35.70,  -34.32,  -94.56,   -0.02,  -51.24,  -35.68],
    [-33.59,  -27.74, -107.15,   -0.02,  -45.23, -123.58],
    [-6.89,  -43.50,  -75.26,    0.05,  -61.35,   -6.91]
]

# 6개
PINK_DROP_LIST = [
    [-36.92,  -48.10,  -66.31,   -0.02,  -65.70, -126.91],
    [-41.98,  -55.98,  -49.66,   -0.03,  -74.47, -131.98],
    [-39.20,  -40.55,  -81.79,   -0.03,  -57.78, -129.19],
    [-44.54,  -47.43,  -67.68,   -0.04,  -64.99, -134.52],
    [-43.83,  -36.19,  -91.07,   -0.04,  -52.86, -133.80],
    [-49.06,  -42.88,  -77.58,   -0.05,  -59.65, -139.04],
]

#6개
PURPLE_DROP_LIST = [
    [18.53,  -28.25, -105.48,    0.12,  -46.35,  -71.55],
    [9.35,  -27.92, -106.11,    0.10,  -46.06,  -80.71],
    [13.81,  -22.88, -115.26,    0.11,  -41.95, -121.27],
    [26.57,  -15.46, -128.49,    0.16,  -36.11,  -63.56],
    [13.61,  -14.95, -129.34,    0.13,  -35.79,  -76.50],
    [-11.03,  -45.10,  -71.99,    0.04,  -63.02,  -11.05]
]

#2개
RED_DROP_LIST = [
    [-18.87,  -46.52,  -69.08,    0.02,  -64.52, -108.88],
    [-17.54,  -52.72,  -55.96,    0.02,  -71.43,  -17.55],
]

#5개
YELLOW_DROP_LIST = [
    [-1.08,  -50.77,  -59.87,    0.06,  -69.46,   -1.10],
    [-1.22,  -41.67,  -78.96,    0.06,  -59.48,   -1.25],
    [4.97,  -47.26,  -67.17,    0.07,  -65.67,  -25.06],
    [8.70,  -48.92,  -63.72,    0.07,  -67.46,    8.68],
    [9.83,  -39.97,  -82.18,    0.08,  -57.94,    9.79],
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