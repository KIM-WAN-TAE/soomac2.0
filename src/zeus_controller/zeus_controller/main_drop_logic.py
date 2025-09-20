#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import threading, time
import numpy as np

# '1 blue', '2 green', '3 purple', '4 red' Int32multiArray [color, idx]

BLUE_DROP_LIST = [[1.2, -44.19, -105.30, 0.19, -30.46, -91.38],
                  [9.44, -43.74, -106.58, 0.19, -29.60, -83.06],
                  [15.83, -41.82, -112.86, 0.19, -25.51, -165.04],
                  [1.27, -40.0, -117.84, 0.17, -22.04, -89.60]]

GREEN_DROP_LIST = [[-7.57, -42.03, -105.41, 0.18, -32.51, -7.31],
                   [-8.81, -37.41, -117.40, 0.17, -24.75, -8.49],
                   [-13.01, -35.64, -123.64, 0.38, -20.67, -102.28]]

class MainDropLogic(Node):
    def __init__(self):
        super().__init__('main_drop_logic')
        
        self.lock = threading.Lock()
        
        self.sub_cb_gp = ReentrantCallbackGroup()

        self.create_subscription(String, '/zeus/string/block_color', self.block_color_callback, callback_group=self.sub_cb_gp)
        self.create_subscription(String, '/zeus/string/drop_done', self.drop_flag_callback, callback_group=self.sub_cb_gp)
        
        self.block_coor_pub = self.create_publisher(Float32MultiArray, '/zeus/array/drop_point', 10)
        
        self.block_color = None
        self.drop_done_flag = False
        
        self.blue_count = 0
        self.green_count = 0
        
    def block_color_callback(self, msg : String):
        with self.lock:
            self.block_color = msg.data
            
            self.count_block_color(self.block_color)
    
    def drop_flag_callback(self, msg : String):
        if msg.data == 'done':
            self.drop_done_flag = True
    
    def count_block_color(self, block_color):
        with self.lock:
            if block_color == 'blue':
                BLUE_DROP_LIST[self.blue_count]
                
        
            elif block_color == 'green':
                GREEN_DROP_LIST[self.green_count]
        
def main(args=None):
    rclpy.init(args=args)
    node = MainDropLogic()
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()