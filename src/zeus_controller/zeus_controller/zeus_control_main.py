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
        
        self.srv_done = self.create_subscription(String, '/zeus/string/service_done', self.client_state_callback, 10)
        self.sub_llm = self.create_subscription(String, '/zeus/string/llm_cmd', self.llm_callback, 10)

        self.create_timer(TIMER_PERIOD, self.loop)
        self.reset_param()
   
    def reset_param(self):
        with self.lock:
            self.handler = None
            self.current_step = None
            self.next_step = None
            
            self.current_flag = 'idle'
            '''
            idle : LLM 명령 대기 상태
            order : Client에 명령 전송 가능 상태
            waiting : client에 명령 전송 완료, 응답 대기 중
            done : client 동작 상태
            '''
        
    def client_state_callback(self, msg : String):
        data = msg.data.strip().lower()
        with self.lock:
            if data in ('done', 'success'):
                if self.current_flag == 'waiting':
                    self.current_flag = 'done'
                    
            elif data in ('fail', 'error'):
                self.current_flag = 'fail'
                # 추후 안전 자세로 되돌아가는 로직 추가
        
    def create_handler(self, mode):
        if mode == 'START':
            return Start()

        elif mode == 'FINISH':
            return Finish()
        
        return None

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
            
            self.handler = self.create_handler(self.mode)
            self.current_step = 'step_1'
            
            self.current_flag = 'order'
            
        self.get_logger().info(
            f"[PARSED] mode={self.mode}, tool={self.tool}, "
            f"direction={self.direction}, target={self.target}"
        )
        
    def advance_step(self, next_step):
        if next_step in (None, 'None'):
            self.get_logger().info('[ZEUS] All Step Finished')
            self.reset_param()
            
        else:
            with self.lock:
                self.current_step = next_step
                self.current_flag = 'order'
                self.next_step = None
            
    def loop(self):
        with self.lock:
            handler = self.handler
            current_step = self.current_step
            current_flag = self.current_flag
            next_step = self.next_step

        # 모니터링: 현재 상태 출력
        self.get_logger().info(f'[MONITOR] Flag: {current_flag}, Step: {current_step}, Next: {next_step}')

        if handler is None or current_step is None:
            return
        
        if current_flag == 'order':
            ans = handler.step(current_step)
            
            if ans is None:
                self.get_logger().warn(f'[ZEUS] Wrong Step : {current_step}')
                
                with self.lock:
                    self.current_step = None
                    self.current_flag  = 'order'
                return
            
            requires_ack = ans.get('requires_ack')
            # 여기서 모듈에 대한 명령을 요청함
            self.module_translator(ans)
            
            # 이후 flag 처리를 어떻게 할지에 대해, next_step을 다음 step으로 넘길지에 대한 논의
            
            # 동작에 대한 대기가 있어야 할 경우 next step에 다음 스텝을 저장 이후 done이 나오면 반환
            if requires_ack:
                with self.lock:
                    self.next_step = ans.get('next_step')
                    self.current_flag = 'waiting'

            # 동작에 대한 대기가 필요 없는 경우 next step을 바로 현재 스텝에 넣어 다음 스텝으로 넘어갈 수 있게
            else:
                with self.lock:
                    self.current_step = ans.get('next_step')
                    self.current_flag = 'order'
                    
        elif current_flag == 'waiting':
            self.get_logger().info('[ZEUS] Waiting Movement')
            
            return
        
        elif current_flag == 'done':
            self.get_logger().info('[ZEUS] 단일 동작 완료')
            self.advance_step(next_step)
                
        elif current_flag == 'fail':
            self.get_logger().warn('[ZEUS] 클라이언트에서 명령 실패 응답. 플래그를 idle로 복구합니다.')
            
            # 모든 변수 초기화 및 flag : idle 상태로 전환해 대기 상태로 전환
            self.reset_param()
            
            
    def module_translator(self, ans):
        if ans.get('position'):
            cmd_msg = ZeusMainCommand()
            
            cmd_msg.frame    = ans['frame']
            cmd_msg.position = ans['position']
            cmd_msg.speed    = ans['speed']
            self.cmd_pub.publish(cmd_msg)
            
            self.get_logger().info(f"[ZEUS] Move to position")
        
        if ans.get('gripper'):
            self.get_logger().info(f"[ZEUS] gripper")
        
        if ans.get('wait'):
            self.get_logger().info(f"[ZEUS] wait")
            
        if ans.get('camera_trigger'):
            self.get_logger().info(f"[ZEUS] camera_trigger")
            
def main(args=None):
    rclpy.init(args=args)
    node = MainControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nShutting down ZEUS Service Client...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
