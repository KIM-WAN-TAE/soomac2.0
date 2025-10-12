#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Float32MultiArray
from zeus_interfaces.msg import ZeusMainCommand

from zeus_controller.module import *
from zeus_controller.dh_module import *
from zeus_controller.read_json import CameraDHParameters

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
        self.grip_cmd_pub = self.create_publisher(String, '/zeus/string/gripper_command', 10)
        self.cam_pub = self.create_publisher(String, '/zeus/string/tool_info', 10)
        
        self.srv_done = self.create_subscription(String, '/zeus/string/service_done', self.client_state_callback, 10)
        self.gripper_done = self.create_subscription(String, '/zeus/string/gripper_done', self.gripper_state_callback, 10)
        self.sub_llm = self.create_subscription(String, '/zeus/string/llm_cmd', self.llm_callback, 10)
        self.cam_done = self.create_subscription(Float32MultiArray, '/zeus/array/tool_pos', self.tool_callback, 10)
        
        self.create_subscription(Float32MultiArray, '/zeus/array/xy_state', self.xy_state_callback, 10)
        self.create_subscription(Float32MultiArray, '/zeus/array/joint_state', self.joint_state_callback, 10)

        self.dh_params = CameraDHParameters()
        self.base_to_camera_matrix = None

        self.create_timer(TIMER_PERIOD, self.loop)
        self.reset_param()
        self.reset_tool_param()
   
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
            
    def cal_base_to_cam(self, joint_angles):
        all_dh_params = self.dh_params.get_all_dh_params(joint_angles[:6])
    
        T_BC = fk(all_dh_params)
        return T_BC
    
    # ==============================================================
    # ======================== Callback 모음 ========================
    
    def joint_state_callback(self, msg):
        if len(msg.data) == 6:
            with self.lock:
                self.joint_coor = msg.data
                self.base_to_camera_matrix = self.cal_base_to_cam(np.deg2rad(list(self.joint_coor)))
                
    def xy_state_callback(self, msg):
        if len(msg.data) == 6:
            with self.lock:
                self.xy_coor = msg.data
    
    def reset_tool_param(self):
        with self.lock:
            
            self.tool_p   = []
            self.tool_yaw = None
    
    def client_state_callback(self, msg : String):
        data = msg.data.strip().lower()
        with self.lock:
            if data in ('done', 'success'):
                if self.current_flag == 'waiting':
                    self.current_flag = 'done'
                    
            elif data in ('fail', 'error'):
                self.current_flag = 'fail'
                # 추후 안전 자세로 되돌아가는 로직 추가
                
    def gripper_state_callback(self, msg : String):
        data = msg.data.strip().lower()
        with self.lock:
            if data in ('done', 'success'):
                if self.current_flag == 'waiting':
                    self.current_flag = 'done'
                    
    def tool_callback(self, msg : Float32MultiArray):
        if len(msg.data) != 4:
            return
        
        P = np.asarray(msg.data[:3], dtype=np.float32)
        T_CO = pos_as_T(P)
        
        with self.lock:
            T_BO = self.base_to_camera_matrix @ T_CO
            
        obj_pos = T_BO[:3, 3].astype(np.float32)
        yaw = float(msg.data[3])
        
        print(f'\n########{obj_pos}########\n')
        print(f'\n########{yaw}########\n')
        
        with self.lock:
            self.tool_p   = obj_pos
            self.tool_yaw = yaw
            
            if msg.data:
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
                    
    # ======================== Callback 모음 ========================
    # ==============================================================
        
    def create_handler(self, mode, tool):
        if mode == 'START':
            return Start()

        elif mode == 'FINISH':
            return Finish()
        
        elif mode == 'DOWN':
            return Down()
        
        elif mode == 'DELIVER':
            if tool == 'M3':
                return Deliver_Box()
            
            else:
                return Deliver_Normal()
            
        elif mode == 'RETURN':
            return Return_Normal()
            
        elif mode == 'TEST':
            return Test()
        
        return None

    def advance_step(self, next_step):
        if next_step in (None, 'None'):
            self.get_logger().info('[ZEUS] All Step Finished')
            self.reset_param()
            self.reset_tool_param()
            
        else:
            with self.lock:
                self.current_step = next_step
                self.current_flag = 'order'
                self.next_step = None
                
    def waiting(self, t):
        import time
        
        start_time = time.time()
        
        while True:
            print(f"Time left : {(t - (time.time() - start_time)):.2}")
            if time.time() - start_time >= t:
                break
            
        with self.lock:
            if self.current_flag == 'waiting':
                self.current_flag = 'done'
    
    # 메인 루프 함수       
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
         
    # 요청 받은 동작에 대한 움직임을 관장하는 함수   
    def module_translator(self, ans):
        if ans.get('position'):
            cmd_msg = ZeusMainCommand()
            
            cmd_msg.frame    = ans['frame']
            cmd_msg.position = ans['position']
            cmd_msg.speed    = ans['speed']
            self.cmd_pub.publish(cmd_msg)
            
            self.get_logger().info(f"[ZEUS] Move to position")
        
        if ans.get('gripper'):
            grip_msg = String()
            
            grip_msg.data = ans['gripper']
            self.grip_cmd_pub.publish(grip_msg)
            
            self.get_logger().info(f"[ZEUS] gripper")
        
        if ans.get('wait'):
            self.get_logger().info(f"[ZEUS] wait")
            
        if ans.get('clear'):
            self.reset_tool_param()
            self.get_logger().info(f"[ZEUS] clear")
        
        # Deliver Normal 에 사용하는 기능 =======================================
        if ans.get('camera_trigger'):
            cam_msg = String()
            
            with self.lock:
                tool = self.tool
                
            cam_msg.data = tool
            self.cam_pub.publish(cam_msg)
            self.get_logger().info(f"[ZEUS] camera_trigger")
            
        if ans.get('camera_move'):
            with self.lock:
                if self.base_to_camera_matrix is None:
                    self.get_logger().warn(f"[ZEUS] Waiting For Camera Matrix")
                    return
                
                x,y,z = self.tool_p
                current_p = self.xy_coor
                yaw = self.tool_yaw
                
            cmd_msg = ZeusMainCommand()
            
            # 340mm
            cmd_msg.frame    = 'l7' # 커터는 대회장에서 해야할 듯
            if self.tool == 'wire_cutter': # 커터만 예외처리
                cmd_msg.position = [current_p[0] - 250.0, float(y) - 10.0, float(z) + 20.0, -90.0, float(yaw), 90.0]
            
            # 나머지 툴은 다른 Offset -30 : 타공판 진입 Offset임, 아마 tool Offset이 안들어가 있어서 그런 듯
            elif self.tool == 'wire_stripper' or self.tool == 'nipper': 
                cmd_msg.position = [current_p[0] - 250.0, float(y), float(z), -90.0, float(yaw), 90.0]
            
            elif self.tool == 'M3': # 집는 Z 값 Offset 들어가있음
                cmd_msg.position = [float(x), float(y), 60.0, -90.0 + float(yaw), 0.0, 179.0]
                
            else:
                self.get_logger().warn('[ZEUS] Wrong Tool')
                return
                
            cmd_msg.speed    = ans['speed']
            self.cmd_pub.publish(cmd_msg)
            
        if ans.get('deliver_offset_move'):
            cmd_msg = ZeusMainCommand()
            with self.lock:
                current_p = self.xy_coor
                yaw = self.tool_yaw
            
            cmd_msg.frame = 'l7'
            # 많이 기울어져 있으면 동작 추가
            if abs(yaw) > 10.0:
                cmd_msg.position = current_p
                cmd_msg.position[2] += 18.0
                
            # 아니면 현 위치 고수
            else:
                cmd_msg.position = current_p
            
            cmd_msg.speed = ans['speed']
            self.cmd_pub.publish(cmd_msg)
        # ==========================================================================
        
        if ans.get('target_trigger'):
            target_msg = String()
            
            with self.lock:
                target = self.target
                
            target_msg.data = target
            self.cam_pub.publish(target_msg)
            self.get_logger().info(f"[ZEUS] target_trigger")
    
        if ans.get('target_move'):
            with self.lock:
                if self.base_to_camera_matrix is None:
                    self.get_logger().warn(f"[ZEUS] Waiting For Camera Matrix")
                    return
                
                x,y,z = self.tool_p
                yaw = self.tool_yaw
            
            cmd_msg = ZeusMainCommand()
            cmd_msg.frame = 'l7'
            cmd_msg.position = [float(x), float(y), 60.0, -90.0, 0.0, 179.0]
            
            cmd_msg.speed    = ans['speed']
            self.cmd_pub.publish(cmd_msg)
            
        if ans.get('boxbox'):
            with self.lock:
                direction = self.direction
            
            cmd_msg = ZeusMainCommand()
            cmd_msg.frame = 't'
            cmd_msg.speed    = ans['speed']
            
            if direction == 'right':
                cmd_msg.position = [-110.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                
            elif direction == 'left':
                cmd_msg.position = [110.0, 0.0, 0.0, 0.0, 0.0, 0.0]

            elif direction == 'front':
                cmd_msg.position = [0.0, 110.0, 0.0, 0.0, 0.0, 0.0]
                
            elif direction == 'back':
                cmd_msg.position = [0.0, -110.0, 0.0, 0.0, 0.0, 0.0]
                
            self.cmd_pub.publish(cmd_msg)
            
        if ans.get('return_camera_trigger'):
            cam_msg = String()
            
            with self.lock:
                tool = self.tool
                
            cam_msg.data = tool
            self.cam_pub.publish(cam_msg)
            self.get_logger().info(f"[ZEUS] return_camera_trigger")
            
        if ans.get('return_camera_move'):
            with self.lock:
                if self.base_to_camera_matrix is None:
                    self.get_logger().warn(f"[ZEUS] Waiting For Camera Matrix")
                    return
                
                x,y,z = self.tool_p
                yaw = self.tool_yaw
            
            cmd_msg = ZeusMainCommand()
            cmd_msg.frame = 't'
            cmd_msg.position = [0.0, 0.0, 67.0, 0.0, 0.0, 0.0]
            
            cmd_msg.speed    = ans['speed']
            self.cmd_pub.publish(cmd_msg)
            
        if ans.get('return_camera_center'):
            with self.lock:
                if self.base_to_camera_matrix is None:
                    self.get_logger().warn(f"[ZEUS] Waiting For Camera Matrix")
                    return
                
                x,y,z = self.tool_p
                yaw = self.tool_yaw
                xy_coor = self.xy_coor
                
            # xy 값만 도구 중심으로 이동할 수 있게 삽입
            P = xy_coor
            P[0] = float(x)
            P[1] = float(y)
            P[2] = P[2] - 240.0 
            P[3] = P[3] + yaw
            cmd_msg = ZeusMainCommand()
            cmd_msg.frame = 'l7'
            cmd_msg.position = P
            
            cmd_msg.speed    = ans['speed']
            self.cmd_pub.publish(cmd_msg)
        
        # 여기서 자리 기억 및 tool 정보에 따른 반환 자리 다 작성해야 함    
        if ans.get('return_tool_offset'):
            with self.lock:
                tool = self.tool
            print('')
            print(tool)
            print('')
            cmd_msg = ZeusMainCommand()
            cmd_msg.frame = 'j'
            
            if tool == 'wire_stripper':
                cmd_msg.position = [-147.02,   45.30,  118.83,  121.14,   81.51,  -76.08]
            
            cmd_msg.speed = ans['speed']
            self.cmd_pub.publish(cmd_msg)
        
        if ans.get('wait_a_sec'):
            t = ans['wait_a_sec']
            self.waiting(t)
            
        
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
