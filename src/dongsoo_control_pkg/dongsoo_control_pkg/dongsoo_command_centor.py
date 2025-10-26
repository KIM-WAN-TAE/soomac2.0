#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Float32MultiArray
from dongsoo_interfaces.msg import DongSooCommand

from .dh_module import *
from .module import *

import numpy as np
import threading
import json

class MainControlNode(Node):
    def __init__(self):
        super().__init__('main_control_node')
        
        self.data_lock = threading.Lock()
        self.lock = threading.Lock()
        
        self.create_subscription(String, '/aiot/string/client_done', self.client_done_callback, 10)
        self.create_subscription(String, '/aiot/string/gripper_done', self.grip_done_callback, 10)
        self.create_subscription(Float32MultiArray, '/aiot/array/tool_pose', self.cam_callback, 10)
        self.create_subscription(Float32MultiArray, '/aiot/matrix/camera', self.cam_coor_callback, 10)
        # self.create_subscription(Float32MultiArray, '/aiot/matrix/gripper', self.grip_coor_callback, 10)
        self.create_subscription(String, '/aiot/string/llm_cmd', self.llm_callback, 10)
        
        self.cmd_pub = self.create_publisher(DongSooCommand, '/aiot/custom/command', 10)
        self.grip_pub = self.create_publisher(String, '/aiot/string/gripper_command', 10)
        self.cam_pub = self.create_publisher(String, '/aiot/string/tool_info', 10)
        
        self.cam_mat = None
        # self.grip_mat = np.array([])
        
        self.reset_param()
        self.reset_tool_param()
        
        self.create_timer(1/10, self.loop)
            
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
            
            self.tool = None
            self.mode = None
            self.direction = None
            self.target = None
                        
    def reset_tool_param(self):
        with self.lock:
            
            self.tool_p = []
            self.tool_yaw = None
    
    # self.cam_mat
    def cam_coor_callback(self, msg : Float32MultiArray):
        with self.data_lock:
            dims = msg.layout.dim
            
            if len(dims) < 2:
                self.get_logger().warn(' 잘못된 행렬 수신 ')
                return

            rows = dims[0].size
            cols = dims[1].size
            
            if len(msg.data) != rows * cols:
                self.get_logger().warn(f' msg count error : msg_count : {rows*cols}')
                return

            self.cam_mat = np.asarray(msg.data, dtype=np.float32).reshape(rows, cols)
    
    # 혹시 몰라 만들어 둔 Gripper Pose 값 읽어오는 callback
    # self.grip_mat    
    def grip_coor_callback(self, msg : Float32MultiArray):
        with self.data_lock:
            dims = msg.layout.dim
            
            if len(dims) < 2:
                self.get_logger().warn(' 잘못된 행렬 수신 ')
                return

            rows = dims[0].size
            cols = dims[1].size
            
            if len(msg.data) != rows * cols:
                self.get_logger().warn(f' msg count error : msg_count : {rows*cols}')
                return
            
            self.grip_mat = np.asarray(msg.data, dtype=np.float32).reshape(rows, cols)
            self.grip_coor = self.grip_mat[:3, 3]
    
    def client_done_callback(self, msg : String):
        data = msg.data.strip().lower()
        
        with self.lock:
            if data in ('done', 'success'):
                if self.current_flag == 'waiting':
                    self.current_flag = 'done'
                    
            elif data in ('fail', 'error'):
                self.current_flag = 'fail'
    
    def grip_done_callback(self, msg : String):
        data = msg.data.strip().lower()
        
        print(1111111111111111111)
        
        with self.lock:
            if data in ('done', 'success'):
                if self.current_flag == 'waiting':
                    self.current_flag = 'done'

        
    def cam_callback(self, msg : Float32MultiArray):
        if len(msg.data) != 4:
            return
        
        P = np.asarray(msg.data[:3], dtype=np.float32)
        T_CO = pos_as_T(P)
        
        with self.data_lock:
            T_BO = self.cam_mat @ T_CO
            
        obj_pos = T_BO[:3, 3].astype(np.float32)
        yaw = float(msg.data[3])
        
        print(f'\n########{obj_pos}########\n')
        print(f'\n########{yaw}########\n')

        with self.lock:
            self.tool_p   = obj_pos
            self.tool_yaw = yaw

            if self.current_flag == 'waiting':
                self.current_flag = 'done'
    
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
            
            self.handler = self.create_handler(self.mode, self.tool)
            self.current_step = 'step_1'
            
            self.current_flag = 'order'
            
        self.get_logger().info(
            f"[PARSED] mode={self.mode}, tool={self.tool}, "
            f"direction={self.direction}, target={self.target}"
        )
        
    def advance_step(self, next_step):
        if next_step in (None, 'None'):
            self.get_logger().info('[AIOT] All Step Finished')
            self.reset_param()
            
        else:
            with self.lock:
                self.current_step = next_step
                self.current_flag = 'order'
                self.next_step = None
                    
    def create_handler(self, mode, tool):
        if mode == 'TEST':
            return Test()
        
        return None
    
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
                self.get_logger().warn(f'[AIOT] Wrong Step : {current_step}')
                
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
            self.get_logger().info('[AIOT] Waiting Movement')
            return
        
        elif current_flag == 'done':
            self.get_logger().info('[AIOT] 단일 동작 완료')
            self.advance_step(next_step)
                
        elif current_flag == 'fail':
            self.get_logger().warn('[AIOT] 클라이언트에서 명령 실패 응답. 플래그를 idle로 복구합니다.')
            
            # 모든 변수 초기화 및 flag : idle 상태로 전환해 대기 상태로 전환
            self.reset_param()
            
    def module_translator(self, ans):
        with self.lock:
            xy_coor = self.grip_coor
            
        if ans.get('position'):
            cmd_msg = DongSooCommand()
            
            cmd_msg.position = ans['position']
            cmd_msg.look     = ans['look']
            cmd_msg.time     = ans['time']
            cmd_msg.wrist    = ans['wrist']
            
            self.cmd_pub.publish(cmd_msg)
            
            self.get_logger().info(f"[ZEUS] Move to position")
        
        if ans.get('gripper'):
            grip_msg = String()
            
            grip_msg.data = ans['gripper']
            self.grip_pub.publish(grip_msg)
            
            self.get_logger().info(f"[ZEUS] gripper")
            
        if ans.get('clear'):
            self.reset_tool_param()
            self.get_logger().info(f"[ZEUS] clear")
            
        if ans.get('camera_trigger'):
            cam_msg = String()
            
            with self.lock:
                tool = self.tool
                
            cam_msg.data = tool
            self.cam_pub.publish(cam_msg)
            self.get_logger().info(f"[ZEUS] camera_trigger")
            
        if ans.get('camera_move'):
            with self.data_lock:
                if self.cam_mat is None:
                    self.get_logger().warn(f"[ZEUS] Waiting For Camera Matrix")
                    return
                
                P = self.tool_p
                yaw = self.tool_yaw
                
            cmd_msg = DongSooCommand()
            
            cmd_msg.position = [float(P[0] + 0.03), float(P[1]), float(P[2] - 0.03)]
            cmd_msg.look     = ans['look']
            cmd_msg.time     = ans['time']
            cmd_msg.wrist    = yaw
            
            self.cmd_pub.publish(cmd_msg)
        
        if ans.get('move_only_one_axis'):
            pose = np.array(xy_coor) + np.array(ans['move_only_one_axis'])
            pose = pose.tolist()
            
            cmd_msg = DongSooCommand()
            
            cmd_msg.position = pose
            cmd_msg.look     = ans['look']
            cmd_msg.time     = ans['time']
            cmd_msg.wrist    = ans['wrist']
            
            self.cmd_pub.publish(cmd_msg)
            
            self.get_logger().info(f"[ZEUS] Move to Relative position")
            
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