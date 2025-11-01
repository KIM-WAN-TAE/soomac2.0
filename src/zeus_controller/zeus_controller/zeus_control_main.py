#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Float32
from zeus_interfaces.msg import ZeusMainCommand

from zeus_controller.module import *
from zeus_controller.dh_module import *
from zeus_controller.read_json import CameraDHParameters

import numpy as np
import threading
import json, time

RATE = 20
TIMER_PERIOD = 1/RATE
NUM_OF_TOOLS = 4

class MainControlNode(Node):
    def __init__(self):
        super().__init__('main_control')
        self.lock = threading.Lock()
        
        self.cmd_pub = self.create_publisher(ZeusMainCommand, '/zeus/custom/client_command', 10)
        self.grip_cmd_pub = self.create_publisher(String, '/zeus/string/gripper_command', 10)
        self.cam_pub = self.create_publisher(String, '/zeus/string/tool_info', 10)
        self.llm_pub = self.create_publisher(String, '/tool_chat/out', 10)
        self.work_done_pub = self.create_publisher(String, '/task/done', 10)
        
        self.srv_done = self.create_subscription(String, '/zeus/string/service_done', self.client_state_callback, 10)
        self.gripper_done = self.create_subscription(String, '/zeus/string/gripper_done', self.gripper_state_callback, 10)
        self.sub_llm = self.create_subscription(String, '/zeus/string/llm_cmd', self.llm_callback, 10)
        self.cam_done = self.create_subscription(Float32MultiArray, '/zeus/array/tool_pos', self.tool_callback, 10)
        
        self.create_subscription(Float32MultiArray, '/zeus/array/xy_state', self.xy_state_callback, 10)
        self.create_subscription(Float32MultiArray, '/zeus/array/joint_state', self.joint_state_callback, 10)
        self.create_subscription(Float32, '/zeus/float/length_val', self.length_callback, 10)

        self.dh_params = CameraDHParameters()
        self.base_to_camera_matrix = None

        self.create_timer(TIMER_PERIOD, self.loop)
        self.reset_param()
        self.reset_tool_pose()
        self.reset_tool_param()
        self.reset_tool_dict()
   
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
            self.length = None
            
    def reset_tool_dict(self):
        with self.lock:
            # Return 디버깅용 초기화 값들
            # self.tool_dict = {'wire_stripper': 'middle', 'nipper': 'left', 'M3':'middle'}
            # self.tool_dict = {'wire_stripper': 'middle', 'nipper': 'left'}
            # self.tool_dict = {'nipper': 'right'}
            # self.tool_dict = {'wire_stripper': 'right'}
            # self.tool_dict = {'wire_stripper': 'middle'}
            self.tool_dict = {}
            # self.tool_dict = {'wire_stripper': 'middle', 'wire_cutter' : 'right', 'nipper': 'left'}
            
    def reset_tool_pose(self):
        with self.lock:
            self.last_tool_pose = None
    
    def reset_tool_param(self):
        with self.lock:
            
            self.tool_p   = []
            self.tool_yaw = None
            
    def cal_base_to_cam(self, joint_angles):
        all_dh_params = self.dh_params.get_all_dh_params(joint_angles[:6])
    
        T_BC = fk(all_dh_params)
        return T_BC
    
    def return_goal_memory(self, pick_pose, tool):
        p_y = pick_pose[1]
        
        candidates = [
            (471.43, 'right'),
            (588.44, 'middle'),
            (705.46, 'left'),
        ]
        
        _, closest_name = min(candidates, key=lambda x: abs(p_y - x[0]))
        
        # 현재 사용중인 도구 갯수가 4개
        if len(self.tool_dict) >= NUM_OF_TOOLS:
            return
        
        if tool not in self.tool_dict:
            self.tool_dict[tool] = closest_name
        else:
            pass
        
        # 사용 예시
        # tool_dict = {'wire_stripper': 'middle', 'nipper': 'right'}
        
    def box_return_goal_memory(self, pick_pose):
        p_y = pick_pose[1]
        
        candidates = [
            (339.21, 'left'),
            (419.21, 'middle'),
            (496.21, 'right'),
        ]
        
        _, closest_name = min(candidates, key=lambda x: abs(p_y - x[0]))
        
       
        if 'M3' not in self.tool_dict:
            self.tool_dict['M3'] = closest_name
        # M3 가 이미 존재한다면 pass
        else:
            pass
    
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
        print(P)
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
        
    def length_callback(self, msg : Float32):
        if msg.data:
            with self.lock:
                self.length = msg.data
                if self.current_flag == 'waiting':
                    self.current_flag = 'done'
                    
    # ======================== Callback 모음 ========================
    # ==============================================================
        
    def create_handler(self, mode, tool):
        if mode == 'START':
            return Start()

        elif mode == 'FINISH':
            return Finish()
        
        elif mode == 'DELIVER':
            if tool == 'M3':
                return Deliver_Box()
            
            else:
                return Deliver_Normal()
            
        elif mode == 'RETURN':
            return Return_All()
        
        elif mode == 'DOWN':
            return Down()
        
        elif mode == 'MEASURE':
            return Measure()
        
        elif mode == 'TEST':
            return Test()
        
        return None

    def advance_step(self, next_step):
        if next_step in (None, 'None'):
            self.get_logger().info('[ZEUS] All Step Finished')
            self.reset_param()
            self.reset_tool_param()
            
            msg = String()
            msg.data = '작업 완료'
            self.work_done_pub.publish(msg)
            
        # elif next_step == 'check_offset':
        #     with self.lock:
        #         # 와이어 커터는 항상 개떡같이 잡음
        #         if self.tool == 'wire_cutter':
        #             self.current_step = 'step_offset'
        #             self.current_flag = 'order'
        #             self.next_step = None
                    
        #         # 얘는 Yaw에 따라 다름
        #         elif self.tool == 'wire_stripper':
        #                 self.current_step = 'step_offset'
        #                 self.current_flag = 'order'
        #                 self.next_step = None
                    
        #         # 니퍼는 개떡같이 잡아도 문제가 없음
        #         elif self.tool == 'nipper':
        #             self.current_step = 'step_5'
        #             self.current_flag = 'order'
        #             self.next_step = None
            
        
        elif next_step == 'check':
            with self.lock:
                # 모든 공구 이동이 끝났다면 final 초기화 단계로 이동할 수 있게 변경
                if len(self.tool_dict) <= 0:
                    self.current_step = 'final'
                    self.current_flag = 'order'
                    self.next_step = None
                
                # 아직 남아있는 공구 딕셔너리가 있다면 디텍하러 위치로 이동
                # check -> step_1 으로 이동
                else:
                    # 딕셔너리에 M3 만 남아있는 경우
                    if len(self.tool_dict) == 1 and 'M3' in self.tool_dict:
                        self.current_step = 'M3_step_1'
                        self.current_flag = 'order'
                        self.next_step = None
                    
                    # 딕셔너리에 M3 가 없거나 2개 이상 남아있는 경우  
                    else:
                        self.current_step = 'step_1'
                        self.current_flag = 'order'
                        self.next_step = None
            
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
        self.get_logger().info(f'TOOL LIST = {self.tool_dict}')
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
        
        # 범용 동작에 사용하는 기능 =======================================
        # 범용 동작에 사용하는 기능 =======================================
        # 범용 동작에 사용하는 기능 =======================================
        if ans.get('position'):
            cmd_msg = ZeusMainCommand()
            
            cmd_msg.frame    = ans['frame']
            cmd_msg.position = ans['position']
            cmd_msg.speed    = ans['speed']
            self.cmd_pub.publish(cmd_msg)
            
            self.get_logger().info(f"[ZEUS] Move to position")
        
        if ans.get('gripper'):
            grip_msg = String()
            
            if ans['requires_ack'] == False:    
                if ans['gripper'] == 'open':
                    grip_msg.data = 'none_return_open'
                elif ans['gripper'] == 'close':
                    grip_msg.data = 'none_return_close'
            else:
                grip_msg.data = ans['gripper']
                
            self.grip_cmd_pub.publish(grip_msg)
            
            self.get_logger().info(f"[ZEUS] gripper")
            
        if ans.get('clear'):
            self.reset_tool_param()
            self.get_logger().info(f"[ZEUS] clear")
            
        if ans.get('wait_a_sec'):
            t = ans['wait_a_sec']
            self.waiting(t)
            
        if ans.get('wait_for_callback'):
            self.get_logger().info(f"[ZEUS] Wait for callback")
        # 범용 동작에 사용하는 기능 =======================================
        # 범용 동작에 사용하는 기능 =======================================
        # 범용 동작에 사용하는 기능 =======================================
        
        # Deliver Normal 에 사용하는 기능 =======================================
        if ans.get('front_offset_move'):
            with self.lock:
                tool = self.tool
                yaw  = self.tool_yaw
                tool_dict = self.tool_dict
                
            cmd_msg = ZeusMainCommand()
            
            STRIP_X_OFFSET = 12.0
            CUTTER_X_OFFSET = 16.0
            NIPPER_X_OFFSET = 8.0
            
            if tool == 'wire_stripper':
                if yaw < -10.0:
                    pose = [3.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                
                elif yaw > 10.0:
                    pose = [-STRIP_X_OFFSET, 0.0, 0.0, 0.0, 0.0, 0.0]
                
                else:
                    pose = [-(STRIP_X_OFFSET - 5), 0.0, 0.0, 0.0, 0.0, 0.0]
                
            elif tool == 'wire_cutter':
                # if tool_dict[tool] == 'middle':
                #     pose = [-4.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                # else:
                #     pose = [-CUTTER_X_OFFSET, 0.0, 0.0, 0.0, 0.0, 0.0]
                
                pose = [-CUTTER_X_OFFSET, 0.0, 0.0, 0.0, 0.0, 0.0]
                
            elif tool == 'nipper':
                if yaw < -10.0:
                    pose = [3.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                
                elif yaw > 10.0:
                    pose = [-NIPPER_X_OFFSET, 0.0, 0.0, 0.0, 0.0, 0.0]
                
                else:
                    pose = [-(NIPPER_X_OFFSET - 5), 0.0, 0.0, 0.0, 0.0, 0.0]
            
            cmd_msg.position = pose
            cmd_msg.frame = 't'
            cmd_msg.speed = 30.0
            self.cmd_pub.publish(cmd_msg)
                
                
        if ans.get('camera_trigger'):
            cam_msg = String()
            
            with self.lock:
                tool = self.tool
                
            cam_msg.data = tool
            self.cam_pub.publish(cam_msg)
            self.get_logger().info(f"[ZEUS] camera_trigger")
            
        if ans.get('camera_move'):
            with self.lock:
                tool = self.tool
                if self.base_to_camera_matrix is None:
                    self.get_logger().warn(f"[ZEUS] Waiting For Camera Matrix")
                    return
                
                x,y,z = self.tool_p
                current_p = self.xy_coor
                yaw = self.tool_yaw
                self.return_goal_memory([x, y, z], tool)

            cmd_msg = ZeusMainCommand()
            
            # 340mm
            cmd_msg.frame    = 'l7' # 커터는 대회장에서 해야할 듯
            if self.tool == 'wire_cutter': # 커터만 예외처리
                cmd_msg.position = [current_p[0] - 250.0, float(y) - 10.0, float(z) + 20.0, -90.0, float(yaw), 90.0]
            
            # 나머지 툴은 다른 Offset -30 : 타공판 진입 Offset임, 아마 tool Offset이 안들어가 있어서 그런 듯
            elif self.tool == 'wire_stripper' or self.tool == 'nipper': 
                cmd_msg.position = [current_p[0] - 250.0, float(y), float(z), -90.0, float(yaw), 90.0]
                
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
        
        # BOXBOX 동작에 사용하는 기능 =======================================
        # BOXBOX 동작에 사용하는 기능 =======================================   
        # BOXBOX 동작에 사용하는 기능 =======================================
        if ans.get('box_camera_move'):
            with self.lock:
                if self.base_to_camera_matrix is None:
                    self.get_logger().warn(f"[ZEUS] Waiting For Camera Matrix")
                    return
                
                x,y,z = self.tool_p
                current_p = self.xy_coor
                yaw = self.tool_yaw
                self.box_return_goal_memory([x, y, z])

            cmd_msg = ZeusMainCommand()

            cmd_msg.frame    = 'l7' # 커터는 대회장에서 해야할 듯
            if self.tool == 'M3': # 집는 Z 값 Offset 들어가있음
                cmd_msg.position = [float(x), float(y), 0.0, -90.0 - float(yaw), 0.0, 179.0]
                
            else:
                self.get_logger().warn('[ZEUS] Wrong Tool')
                return
                
            cmd_msg.speed    = ans['speed']
            self.cmd_pub.publish(cmd_msg)
        
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
                if self.length:
                    length = self.length * 10 # cm 단위로 들어오기에 mm로 변환 
                    direction = self.direction
                    
            msg = String()
            text = '네, 이동하겠습니다.'
            msg.data = text
            self.llm_pub.publish(msg)
            
            cmd_msg = ZeusMainCommand()
            cmd_msg.frame = 't'
            cmd_msg.speed    = ans['speed']
            
            if direction == 'right':
                cmd_msg.position = [-length, 0.0, 0.0, 0.0, 0.0, 0.0]
                
            elif direction == 'left':
                cmd_msg.position = [length, 0.0, 0.0, 0.0, 0.0, 0.0]

            elif direction == 'front':
                cmd_msg.position = [0.0, length, 0.0, 0.0, 0.0, 0.0]
                
            elif direction == 'back':
                cmd_msg.position = [0.0, -length, 0.0, 0.0, 0.0, 0.0]
                
            self.cmd_pub.publish(cmd_msg)
            
        if ans.get('chat_trigger'):
            msg = String()
            text = '몇 센치 만큼 이동할까요?'
            msg.data = text
            self.llm_pub.publish(msg)
            
        # BOXBOX 동작에 사용하는 기능 =======================================
        # BOXBOX 동작에 사용하는 기능 =======================================   
        # BOXBOX 동작에 사용하는 기능 =======================================  
        
        if ans.get('talking_measure_result'):
            if ans['result'] == 'sound':
                msg = String()
                text = '검사 결과 문제 없음!'
                msg.data = text
                self.llm_pub.publish(msg)
                
            elif ans['result'] == 'no_sound':
                msg = String()
                text = '이 부분이 문제네요!'
                msg.data = text
                self.llm_pub.publish(msg)
        
        # return 동작에 사용하는 기능 =======================================
        # return 동작에 사용하는 기능 =======================================   
        # return 동작에 사용하는 기능 =======================================   
        # if ans.get('return_camera_trigger'):
        #     cam_msg = String()
            
        #     with self.lock:
        #         tool = self.tool
                
        #     cam_msg.data = tool
        #     self.cam_pub.publish(cam_msg)
        #     self.get_logger().info(f"[ZEUS] return_camera_trigger")
            
        # if ans.get('return_camera_move'):
        #     with self.lock:
        #         if self.base_to_camera_matrix is None:
        #             self.get_logger().warn(f"[ZEUS] Waiting For Camera Matrix")
        #             return
                
        #         x,y,z = self.tool_p
        #         yaw = self.tool_yaw
            
        #     cmd_msg = ZeusMainCommand()
        #     cmd_msg.frame = 't'
        #     cmd_msg.position = [0.0, 0.0, 40.0, 0.0, 0.0, 0.0]
            
        #     cmd_msg.speed    = ans['speed']
        #     self.cmd_pub.publish(cmd_msg)
            
        # if ans.get('return_camera_center'):
        #     with self.lock:
        #         if self.base_to_camera_matrix is None:
        #             self.get_logger().warn(f"[ZEUS] Waiting For Camera Matrix")
        #             return
                
        #         x,y,z = self.tool_p
        #         yaw = self.tool_yaw
        #         xy_coor = self.xy_coor
                
        #     # xy 값만 도구 중심으로 이동할 수 있게 삽입
        #     P = xy_coor
        #     P[0] = float(x)
        #     P[1] = float(y)
        #     P[2] = -41.8
        #     P[3] = P[3] + yaw
        #     cmd_msg = ZeusMainCommand()
        #     cmd_msg.frame = 'l7'
        #     cmd_msg.position = P
            
        #     cmd_msg.speed    = ans['speed']
        #     self.cmd_pub.publish(cmd_msg)
        
        # # 여기서 자리 기억 및 tool 정보에 따른 반환 자리 다 작성해야 함    
        # if ans.get('return_tool_offset'):
        #     with self.lock:
        #         tool = self.tool
        #         return_direction = self.last_tool_pose
        #     print('')
        #     print(tool)
        #     print(f' Return Direction : {return_direction}')
        #     print('')
        #     cmd_msg = ZeusMainCommand()
        #     cmd_msg.frame = 'j'
            
        #     if tool == 'wire_stripper':
        #         if return_direction == 'right':
        #             cmd_msg.position = [-147.02,   45.30,  118.83,  121.14,   81.51,  -76.08]
                    
        #         elif return_direction == 'middle':
        #             cmd_msg.position = [-153.79,   52.48,  100.30,  112.84,   78.58,  -64.70]
                    
        #         elif return_direction == 'left':
        #             cmd_msg.position = [-158.52,   62.48,   76.42,  105.79,   76.49,  -50.39]
                    
        #         else:
        #             cmd_msg.position = [-153.79,   52.48,  100.30,  112.84,   78.58,  -64.70]
                    
        #     elif tool == 'wire_cutter':
        #         if return_direction == 'right':
        #             cmd_msg.position = [-147.02,   45.30,  118.83,  121.14,   81.51,  -76.08]
                    
        #         elif return_direction == 'middle':
        #             cmd_msg.position = [-153.79,   52.48,  100.30,  112.84,   78.58,  -64.70]
                    
        #         elif return_direction == 'left':
        #             cmd_msg.position = [-158.52,   62.48,   76.42,  105.79,   76.49,  -50.39]
                    
        #         else:
        #             cmd_msg.position = [-153.79,   52.48,  100.30,  112.84,   78.58,  -64.70]
                    
        #     elif tool == 'nipper':
        #         if return_direction == 'right':
        #             cmd_msg.position = [-148.20,   41.87,  119.46,  119.60,   80.40,  -73.46]
                    
        #         elif return_direction == 'middle':
        #             cmd_msg.position = [-154.99,   49.91,  100.15,  111.22,   78.09,  -61.89]
                    
        #         elif return_direction == 'left':
        #             cmd_msg.position = [-159.51,   60.40,   76.02,  104.44,   76.49,  -47.73]
                    
        #         else:
        #             cmd_msg.position = [-154.99,   49.91,  100.15,  111.22,   78.09,  -61.89]        
            
        #     cmd_msg.speed = ans['speed']
        #     self.cmd_pub.publish(cmd_msg)
            
        #     self.reset_tool_pose()
        # return 동작에 사용하는 기능 =======================================
        # return 동작에 사용하는 기능 =======================================   
        # return 동작에 사용하는 기능 ======================================= 
        
        
        # 통합 Return 에 사용할 기능
        if ans.get('all_return_pop_dict'):
            with self.lock:
                idx_0 = list(self.tool_dict.keys())[0]
                del self.tool_dict[idx_0]
        
        if ans.get('all_return_camera_trigger'):
            cam_msg = String()
            
            with self.lock:
                tool = list(self.tool_dict.keys())[0]
                
            cam_msg.data = tool
            self.cam_pub.publish(cam_msg)
            self.get_logger().info(f"[ZEUS] return_camera_trigger")
            
        if ans.get('all_return_camera_move'):
            with self.lock:
                if self.base_to_camera_matrix is None:
                    self.get_logger().warn(f"[ZEUS] Waiting For Camera Matrix")
                    return
                
                x,y,z = self.tool_p
                yaw = self.tool_yaw
            
            cmd_msg = ZeusMainCommand()
            cmd_msg.frame = 't'
            cmd_msg.position = [0.0, 0.0, 61.4, 0.0, 0.0, 0.0]
            
            cmd_msg.speed    = ans['speed']
            self.cmd_pub.publish(cmd_msg)
            
        if ans.get('all_return_offset_move'):
            with self.lock:
                yaw  = self.tool_yaw
                tool = list(self.tool_dict.keys())[0]
            print(f'내가 지금 반환할 툴 : {tool}')
                
            cmd_msg = ZeusMainCommand()
            
            STRIP_X_OFFSET = 8.0
            CUTTER_X_OFFSET = 10.0
            NIPPER_X_OFFSET = 6.0
            
            if tool == 'wire_stripper':
                if yaw < -10.0:
                    pose = [STRIP_X_OFFSET, 7.0, 0.0, 0.0, 0.0, 0.0]
                
                elif yaw > 10.0:
                    pose = [-STRIP_X_OFFSET, 7.0, 0.0, 0.0, 0.0, 0.0]
                
                else:
                    pose = [0.0, 7.0, 0.0, 0.0, 0.0, 0.0]
                
            elif tool == 'wire_cutter':
                # if tool_dict[tool] == 'middle':
                #     pose = [-4.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                # else:
                #     pose = [-CUTTER_X_OFFSET, 0.0, 0.0, 0.0, 0.0, 0.0]
                
                if yaw < -10.0:
                    pose = [6.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                
                elif yaw > 10.0:
                    pose = [-CUTTER_X_OFFSET, 0.0, 0.0, 0.0, 0.0, 0.0]
                
                else:
                    pose = [2.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                

                
            elif tool == 'nipper':
                if yaw < -10.0:
                    pose = [3.0, 3.0, 0.0, 0.0, 0.0, 0.0]
                
                elif yaw > 10.0:
                    pose = [-NIPPER_X_OFFSET, 3.0, 0.0, 0.0, 0.0, 0.0]
                
                else:
                    pose = [-(NIPPER_X_OFFSET - 5), 3.0, 0.0, 0.0, 0.0, 0.0]
            
            else:
                pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
            cmd_msg.position = pose
            cmd_msg.frame = 't'
            cmd_msg.speed = 30.0
            self.cmd_pub.publish(cmd_msg)
            
            
        if ans.get('all_return_camera_center'):
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
            P[2] = -41.8
            P[3] = P[3] + yaw
            cmd_msg = ZeusMainCommand()
            cmd_msg.frame = 'l7'
            cmd_msg.position = P
            
            cmd_msg.speed    = ans['speed']
            self.cmd_pub.publish(cmd_msg)
        
        # dict 구조로 항상 0번째 살리기   
        if ans.get('all_return_tool_offset'):
            with self.lock:
                tool = list(self.tool_dict.keys())[0]
                return_direction = list(self.tool_dict.values())[0]
                
            print('')
            print(tool)
            print(f' Return Direction : {return_direction}')
            print('')
            cmd_msg = ZeusMainCommand()
            cmd_msg.frame = 'j'
            
            if tool == 'wire_stripper':
                if return_direction == 'right':
                    cmd_msg.position = [-147.02,   45.30,  118.83,  121.14,   81.51,  -76.08]
                    
                elif return_direction == 'middle':
                    cmd_msg.position = [-153.79,   52.48,  100.30,  112.84,   78.58,  -64.70]
                    
                elif return_direction == 'left':
                    cmd_msg.position = [-158.52,   62.48,   76.42,  105.79,   76.49,  -50.39]
                    
                else:
                    cmd_msg.position = [-153.79,   52.48,  100.30,  112.84,   78.58,  -64.70]
                    
            elif tool == 'wire_cutter':
                if return_direction == 'right':
                    cmd_msg.position = [-147.05,   44.24,  118.78,  120.96,   80.95,  -75.12]
                    
                elif return_direction == 'middle':
                    cmd_msg.position = [-153.82,   51.61,  100.25,  112.64,   78.24,  -63.83]
                    
                elif return_direction == 'left':
                    cmd_msg.position = [-158.55,   61.75,   76.36,  105.58,   76.29,  -49.60]
                    
                else:
                    cmd_msg.position = [-153.82,   51.61,  100.25,  112.64,   78.24,  -63.83]
                    
            elif tool == 'nipper':
                if return_direction == 'right':
                    cmd_msg.position = [-148.20,   41.87,  119.46,  119.60,   80.40,  -73.46]
                    
                elif return_direction == 'middle':
                    cmd_msg.position = [-154.99,   49.91,  100.15,  111.22,   78.09,  -61.89]
                    
                elif return_direction == 'left':
                    cmd_msg.position = [-159.51,   60.40,   76.02,  104.44,   76.49,  -47.73]
                    
                else:
                    cmd_msg.position = [-154.99,   49.91,  100.15,  111.22,   78.09,  -61.89]        
            
            cmd_msg.speed = ans['speed']
            self.cmd_pub.publish(cmd_msg)
            
            self.reset_tool_pose()
            
        if ans.get('all_return_m3_camera_trigger'):
            cam_msg = String()
            cam_msg.data = 'M3'
            self.cam_pub.publish(cam_msg)
            self.get_logger().info(f"[ZEUS] camera_trigger")
            
        if ans.get('all_return_m3_box_camera_move'):
            with self.lock:
                if self.base_to_camera_matrix is None:
                    self.get_logger().warn(f"[ZEUS] Waiting For Camera Matrix")
                    return
                
                x,y,z = self.tool_p
                current_p = self.xy_coor
                yaw = self.tool_yaw

            cmd_msg = ZeusMainCommand()

            cmd_msg.frame    = 'l7' # 커터는 대회장에서 해야할 듯
            cmd_msg.position = [float(x), float(y), 0.0, -90.0 - float(yaw), 0.0, 179.0]
            cmd_msg.speed    = ans['speed']
            self.cmd_pub.publish(cmd_msg)
            
        if ans.get('all_return_m3_offset'):
            with self.lock:
                tool = list(self.tool_dict.keys())[0]
                return_direction = list(self.tool_dict.values())[0]
                
            print('')
            print(tool)
            print(f' Return Direction : {return_direction}')
            print('')
            cmd_msg = ZeusMainCommand()
            cmd_msg.frame = 'j'
        
            if return_direction == 'left':
                cmd_msg.position = [-143.10,   33.37,  109.11,    0.78,   36.64,   36.28]
                
            elif return_direction == 'middle':
                cmd_msg.position = [-148.04,   37.80,   99.28,    0.81,   42.09,   31.37]
                
            elif return_direction == 'right':
                cmd_msg.position = [-151.83,   42.82,   88.15,    0.80,   48.23,   27.65]  
            
            else:
                cmd_msg.position = [-148.04,   37.80,   99.28,    0.81,   42.09,   31.37]
                
            cmd_msg.speed = ans['speed']
            self.cmd_pub.publish(cmd_msg)
            
            self.reset_tool_pose()
        
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