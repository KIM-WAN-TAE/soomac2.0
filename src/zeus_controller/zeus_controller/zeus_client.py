#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from zeus_interfaces.srv import ZeusExecutor
from zeus_interfaces.msg import ZeusMainCommand

import threading, time
import numpy as np

RATE = 10

def mparam_command_string(param_type, value):
    if isinstance(value, (list, tuple)):
        value_str = ",".join([str(v) for v in value])
    else:
        value_str = str(value)
    cmd = f"mparam+{param_type},{value_str}"
    return cmd

class ZeusClientNode(Node):
    def __init__(self):
        super().__init__('zeus_client_node')
        self.get_logger().info('[ZEUS] Client Node On!')

        self.lock = threading.Lock()
        
        self.zeus_client = self.create_client(ZeusExecutor, '/zeus_exec')
        while not self.zeus_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting again...')
        
        self.service_done = self.create_publisher(String, '/zeus/string/service_done', 10)
        self.binary_cmd = self.create_publisher(String, '/zeus/string/binary_command', 10)
        
        self.make_pose_speed_fast()
        
        self.create_subscription(ZeusMainCommand, '/zeus/custom/client_command', self.target_pose_callback, 10)
        
        self.target_flag = False
        self.target_frame = None
        self.target_point = np.array([])
        self.target_speed = None
        
        timer_period = 1/RATE
        self.client_timer = self.create_timer(timer_period, self.client_timer_callback)
    
    def send_speed(self, frame, value):
        if frame.lower() == 'j':
            raw_frame = 'jntspd'
        elif frame.lower() == 'l1':
            raw_frame = 'linspd'
        elif frame.lower() == 'l2':
            raw_frame = 'linspd'
        elif frame.lower() == 'l3':
            raw_frame = 'linspd'
        elif frame.lower() == 'l4':
            raw_frame = 'linspd'
        elif frame.lower() == 'l5':
            raw_frame = 'linspd'
        elif frame.lower() == 'l6':
            raw_frame = 'linspd'
        elif frame.lower() == 'l7':
            raw_frame = 'linspd'
        elif frame.lower() == 'l8':
            raw_frame = 'linspd'
        elif frame.lower() == 't':
            raw_frame = 'linspd'
        
        cmd = mparam_command_string(raw_frame, value)
        msg = String()
        msg.data = str(cmd)
        self.binary_cmd.publish(msg)
    
    def target_pose_callback(self, msg : ZeusMainCommand):
        with self.lock:
            self.target_frame = msg.frame
            self.target_point = np.array(msg.position)
            self.target_speed = msg.speed
            self.target_flag = True
            
            self.get_logger().info(f'[AIOT] Target Point Received : {self.target_point}, Frame : {self.target_frame}, Speed : {self.target_speed}')
    
    def make_pose_speed_fast(self):
        raw_frame = 'posspd'
        value = 150.0
        cmd = mparam_command_string(raw_frame, value)
        msg = String()
        msg.data = str(cmd)
        self.binary_cmd.publish(msg)
    
    def client_timer_callback(self):
        self.get_logger().info(f'[AIOT] Flag State : {self.target_flag}')
        
        if not self.target_flag:
            self.get_logger().info('[AIOT] Waiting For Target Point ...')
            return

        if self.target_flag and self.target_point.size != 0:
            copy_target_point = self.target_point
            copy_target_frame = self.target_frame
            copy_target_speed = self.target_speed
            self.target_flag = False
            
            self.send_speed(copy_target_frame, copy_target_speed)
            time.sleep(0.5)
            self.send_next_command(copy_target_point, copy_target_frame)
            
    def send_next_command(self, point, frame):
        if point.size != 6:
            self.get_logger().warn('[ZEUS] Invalid Target Point Size')
            return
        
        req = ZeusExecutor.Request()
        req.frame = frame
        req.coordinate = point
        
        future = self.zeus_client.call_async(req)
        future.add_done_callback(self.response_callback)
        
    def response_callback(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().warn(f'[ZEUS] 서비스 호출 실패: {e}')
            srv_msg = String()
            srv_msg.data = 'fail'
            self.service_done.publish(srv_msg)
            return
        if res.success:
            self.get_logger().info(f'[ZEUS] 명령 성공')
            srv_msg = String()
            srv_msg.data = 'done'
            self.service_done.publish(srv_msg)
        else:
            self.get_logger().warn(f'[ZEUS] 명령 실패')
            srv_msg = String()
            srv_msg.data = 'fail'
            self.service_done.publish(srv_msg)
            
def main(args=None):
    rclpy.init(args=args)
    node = ZeusClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nShutting down ZEUS Service Client...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
