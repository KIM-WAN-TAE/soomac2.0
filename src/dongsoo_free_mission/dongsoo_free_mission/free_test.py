#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray

class TestModule(Node):
    def __init__(self):
        super().__init__('test_module_node')
        
        self.tool_pub = self.create_publisher(String, '/info/string/obj_name', 10)
        self.tool_coor_sub = self.create_subscription(Float32MultiArray, '/info/array/target_obj_array', self.coor_callback, 10)
        
    def coor_callback(self, msg : Float32MultiArray):
        coor = msg.data[:3]
        yaw  = msg.data[3]
        