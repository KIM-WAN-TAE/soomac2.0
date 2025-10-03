#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Bool
from zeus_interfaces.msg import ZeusMainCommand

from zeus_controller.module import *

import numpy as np
import threading
import json

RATE = 10
TIMER_PERIOD = 1/RATE

class MainControlNode(Node):
    def __init__(self):
        super().__init__('main_control')
        self.lock = threading.Lock()
        
        self.cmd_pub = self.create_publisher(ZeusMainCommand, '/zeus/custom/client_command', 10)
        
        self.srv_done = self.create_subscription(Bool, '/zeus/bool/service_done', self.client_state_callback, 10)
        self.sub_llm = self.create_subscription(String, '/zeus/string/llm_cmd', self.llm_callback, 10)

        self.create_timer(TIMER_PERIOD, self.loop)
        self.reset_param()
   
    def reset_param(self):
        self.param = None
        self.client_flag = False
        
    def client_state_callback(self, msg : Bool):
        self.client_flag = msg.data

    def llm_callback(self, msg : String):
        self.get_logger().info(f"[RAW] {msg.data}")
        try:
            obj = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"JSON 파싱 실패: {e}")
            return
        
        with self.lock:
            self.tool      = obj.get('tool')
            self.mode      = obj.get('mode')
            self.direction = obj.get('direction')
            self.target    = obj.get('target') # 공구
            
        self.get_logger().info(
            f"[PARSED] mode={self.mode}, tool={self.tool}, "
            f"direction={self.direction}, target={self.target}, control={self.control}"
        )
            
    def loop(self):
        with self.lock:
            tool      = self.tool
            mode      = self.mode
            direction = self.direction
            target    = self.target
        
        if mode == 'START':
            handler = Start()
            