#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading

# z offset 15cm인 joint 좌표(10.27)
# 7개
# BLUE_DROP_LIST = [
#     [-2.75,  -30.09, -104.36,    0.13,  -45.61,  -92.95],
#     [-11.67,  -32.37,  -99.96,    0.11,  -47.74, -101.85],
#     [-3.11,  -23.80, -116.20,    0.15,  -40.06,  -93.33],
#     [-13.56,  -26.37, -111.46,    0.12,  -42.24, -103.75],
#     [-3.62,  -17.59, -127.16,    0.16,  -35.31,  -93.86],
#     [-15.98,  -20.63, -121.93,    0.13,  -37.52, -106.19],
#     [-8.06,  -44.69,  -74.70,    0.10,  -60.67,   -8.22]
# ]

# # 6개
# GREEN_DROP_LIST = [
#     [-19.66,  -33.38,  -98.00,    0.10,  -48.71,  -19.84],
#     [-23.02,  -27.29, -109.75,    0.10,  -43.04,  -23.20],
#     [-30.77,  -39.86,  -84.28,    0.07,  -55.95,  -30.91],
#     [-35.11,  -34.15,  -95.70,    0.06,  -50.25,  -35.26],
#     [-32.39,  -27.53, -108.43,    0.08,  -44.14, -122.55],
#     [-10.59,  -45.69,  -72.65,    0.10,  -61.73,  -10.74]
# ]

# # 6개
# PINK_DROP_LIST = [
#     [-36.34,  -47.99,  -67.56,    0.05,  -64.55, -126.47],
#     [-41.41,  -55.74,  -51.17,    0.04,  -73.20, -131.53],
#     [-38.64,  -40.48,  -83.05,    0.05,  -56.57, -128.77],
#     [-44.05,  -47.20,  -69.22,    0.04,  -63.68, -134.17],
#     [-43.32,  -35.89,  -92.27,    0.04,  -51.94, -133.46],
#     [-48.64,  -42.48,  -78.98,    0.03,  -58.65, -138.77],
# ]

# #6개
# PURPLE_DROP_LIST = [
#     [19.46,  -29.30, -105.80,    0.15,  -44.92,  -70.76],
#     [10.22,  -28.89, -106.63,    0.15,  -44.52,  -79.99],
#     [14.58,  -24.00, -115.78,    0.17,  -40.25, -120.65],
#     [27.60,  -16.54, -128.82,    0.19,  -34.64,  -62.66],
#     [14.91,  -15.92, -129.90,    0.19,  -34.21,  -75.36],
#     [-5.58,  -43.93,  -76.45,    0.11,  -59.68,   -5.73]
# ]

# #2개
# RED_DROP_LIST = [
#     [-18.18,  -46.75,  -70.45,    0.08,  -62.88, -108.32],
#     [-16.84,  -53.11,  -57.06,    0.08,  -69.91,  -16.98],
#     [-12.46,  -66.71,  -27.48,    0.08,  -85.89,   32.43],
# ]

# #5개
# YELLOW_DROP_LIST = [
#     [-0.39,  -51.56,  -60.45,    0.10,  -68.04,   -0.53],
#     [-0.45,  -42.38,  -79.62,    0.11,  -58.05,   -0.62],
#     [5.71,  -47.92,  -68.12,    0.11,  -64.01,  -24.44],
#     [9.34,  -49.72,  -64.32,    0.11,  -66.00,    9.19],
#     [10.64,  -40.90,  -82.99,    0.13,  -56.15,   10.46],
# ]

# z offset 10cm인 joint 좌표
BLUE_DROP_LIST = [
    [-2.74,  -34.58, -106.67,    0.15,  -38.81,  -92.97],
    [-11.66,  -36.62, -102.22,    0.12,  -41.23, -101.86],
    [-3.10,  -29.12, -118.70,    0.18,  -32.24,  -93.36],
    [-13.55,  -31.32, -113.87,    0.14,  -34.88, -103.77],
    [-3.61,  -24.02, -130.00,    0.21,  -26.04,  -93.91],
    [-15.97,  -26.47, -124.59,    0.16,  -29.02, -106.22],
    [-8.05,  -47.89,  -77.03,    0.11,  -55.14,   -8.23]
]

# 6개
GREEN_DROP_LIST = [
    [-19.65,  -37.53, -100.25,    0.11,  -42.31,  -19.85],
    [-23.01,  -32.12, -112.13,    0.12,  -35.83,  -23.21],
    [-30.77,  -43.38,  -86.58,    0.08,  -50.13,  -30.91],
    [-35.11,  -38.15,  -98.00,    0.07,  -43.94,  -35.27],
    [-32.38,  -32.24, -110.86,    0.09,  -37.00, -122.56],
    [-10.58,  -48.82,  -75.00,    0.11,  -56.25,  -10.75]
]

# 8개
PINK_DROP_LIST = [
    [-33.53,  -45.93,  -81.02,    0.06,  -53.15,  -33.67],
    [-36.34,  -50.95,  -70.02,    0.05,  -59.14, -126.47],
    [-41.41,  -58.19,  -54.05,    0.04,  -67.87, -131.53],
    [-38.64,  -43.96,  -85.35,    0.05,  -50.79, -128.77],
    [-44.05,  -50.21,  -71.65,    0.04,  -58.24, -134.17],
    [-43.32,  -39.74,  -94.56,    0.04,  -45.80, -133.46],
    [-48.64,  -45.81,  -81.30,    0.03,  -53.00, -138.77],
]

#7개
PURPLE_DROP_LIST = [
    [19.47,  -33.88, -108.12,    0.17,  -38.02,  -70.78],
    [10.23,  -33.52, -108.96,    0.17,  -37.56,  -80.01],
    [14.59,  -29.28, -118.27,    0.20,  -32.47, -120.68],
    [27.62,  -23.19, -131.72,    0.25,  -25.09,  -62.72],
    [14.93,  -22.71, -132.85,    0.26,  -24.46,  -75.42],
    [-5.57,  -47.20,  -78.74,    0.12,  -54.12,   -5.74]
]

#4개
RED_DROP_LIST = [
    [-18.17,  -49.81,  -72.83,    0.08,  -57.44, -108.32],
    [-16.83,  -55.76,  -59.71,    0.08,  -64.61,  -16.98],
    [-12.46,  -68.17,  -32.03,    0.08,  -79.88,   32.43]
]

#6개
YELLOW_DROP_LIST = [
    [-0.38,  -54.32,  -63.00,    0.10,  -62.73,   -0.53],
    [-0.44,  -45.76,  -81.89,    0.12,  -52.41,   -0.63],
    [5.72,  -50.92,  -70.52,    0.12,  -58.62,  -24.45],
    [9.35,  -52.60,  -66.78,    0.12,  -60.66,    9.19],
    [10.65,  -44.41,  -85.21,    0.14,  -50.42,   10.45],
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