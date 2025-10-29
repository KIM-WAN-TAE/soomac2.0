#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from zeus_interfaces.msg import ZeusMainCommand

import threading
import logging

class Trestle_Calibration(Node):
    def __init__(self):
        super().__init__('trestle_calibration')
        
        self.lock = threading.Lock()
        
        self.cmd_pub = self.create_publisher(ZeusMainCommand, '/zeus/custom/client_command', 10)
        self.grip_cmd_pub = self.create_publisher(String, '/zeus/string/gripper_command', 10)
        
        self.input_thread = threading.Thread(target=self.input_listener, daemon=True)
        
        self.move_flag = 'idle'
        
        self.create_timer(1/10, self.timer_callback)
        
        self.input_thread.start()
        
    def input_listener(self):
        while rclpy.ok():
            if self.move_flag == 'idle':
                logging.info('[ 캘리브레이션 메뉴 ]')
                logging.info('1번 위치로 이동 : 1')
                logging.info('2번 위치로 이동 : 2')
                logging.info('3번 위치로 이동 : 3')
                logging.info('정밀 진입       : down')
                logging.info('위치 초기화     : init')
                val = input('원하는 명령을 입력하시오 : ')
                
                if val == '1':
                    self.move_flag = '1'
                    
                elif val == '2':
                    self.move_flag = '2'
                    
                elif val == '3':
                    self.move_flag = '3'
                    
                elif val == 'down':
                    self.move_flag = 'down'
                    
                elif val == 'init':
                    self.move_flag = 'init'

    
    def make_movement(self, frame, coor, speed):
        cmd_msg = ZeusMainCommand()
        
        cmd_msg.frame = frame
        cmd_msg.position = coor
        cmd_msg.speed = speed
        
        self.cmd_pub.publish(cmd_msg)
        
        with self.lock:
            self.move_flag = 'idle'
        
    def move_init(self):  
        cmd_msg = ZeusMainCommand()
        
        cmd_msg.frame = 't'
        cmd_msg.position = [0.0, 0.0, -50.0, 0.0, 0.0, 0.0]
        cmd_msg.speed = 10.0
        
        self.cmd_pub.publish(cmd_msg)
        
        self.get_logger().info(' 초기화 과정 # 1 :: 상승')
        
        import time
        start_time = time.time()
        
        while True:
            print(f'남은 시간 : {(6.0 - (time.time() - start_time)):.2f}')
            if time.time() - start_time > 6.0:
                break
            
        cmd_msg.frame = 'j'
        cmd_msg.position =  [-170.55,  -19.30,  147.09,   -0.30,   54.12,   98.71]
        cmd_msg.speed = 5.0
        
        self.get_logger().info(' 초기화 과정 # 2 :: 복귀')
        
        self.cmd_pub.publish(cmd_msg)
        
        with self.lock:
            self.move_flag = 'idle'
        
    def timer_callback(self):
        with self.lock:
            flag = self.move_flag
            
        if flag == 'idle':
            # self.get_logger().info('Waiting For Command')
            return
        
        if flag == 'init':
            self.move_init()
        
        else :
            if flag == '1':
                frame = 'j'
                speed = 5.0
                coor = [-192.82,   40.54,  128.98,   -1.11,   10.82,   77.46]
            
            elif flag == '2':
                frame = 'j'
                speed = 5.0
                coor = [-162.92,   40.86,  127.61,   -0.05,   11.93,  106.32]
            
            elif flag == '3':
                frame = 'j'
                speed = 5.0
                coor = [-155.83,   47.26,  104.30,    0.08,   28.84,  113.29]
            
            elif flag == 'down':
                frame = 't'
                speed = 5.0
                coor = [0.0, 0.0, 30.0, 0.0, 0.0, 0.0]
            
            self.make_movement(frame, coor, speed)

def main(args=None):
    rclpy.init(args=args)
    node = Trestle_Calibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nShutting down ZEUS Service Client...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
