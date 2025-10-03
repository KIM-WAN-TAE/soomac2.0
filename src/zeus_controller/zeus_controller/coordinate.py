#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float32MultiArray

import threading
import numpy as np
import copy

class CoordinateNode(Node):
    def __init__(self):
        super().__init__('Coordinate_Node')
        
        self.lock = threading.Lock()
        self.lst = []
        self.trigger = False
        
        self.cb_group = ReentrantCallbackGroup()
        self.timer_group = ReentrantCallbackGroup()
        
        self.create_subscription(Float32MultiArray, '/zeus/array/xy_state', self.callback, 10, callback_group=self.cb_group)
        self.create_timer(1/10, self.timer, callback_group=self.timer_group)
        
        threading.Thread(target=self.input_thread, daemon=True).start()
        
    def input_thread(self):
        while True:
            val = input()
            if val.strip().lower() == 's':
                with self.lock:
                    self.trigger = True
        
    def callback(self, msg : Float32MultiArray):
        with self.lock:
            self.lst = msg.data
    
    def timer(self):
        with self.lock:
            lst = copy.deepcopy(self.lst)
            do_log = self.trigger
            if self.trigger:
                self.trigger = False

        if do_log:
            formatted = [f"{v:.2f}" for v in lst]
            self.get_logger().info(f"\n{formatted}")

def main(args=None):
    rclpy.init(args=args)
    node = CoordinateNode()
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
        