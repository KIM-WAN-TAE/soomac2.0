#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Float32MultiArray, Int32MultiArray, Bool
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
        self.sub_position = self.create_subscription(Int32MultiArray, '/aiot/array/present_motor_pulse', self.present_position_callback, 10)
        
        self.cmd_pub = self.create_publisher(DongSooCommand, '/aiot/custom/command', 10)
        self.grip_pub = self.create_publisher(String, '/aiot/string/gripper_command', 10)
        self.cam_pub = self.create_publisher(String, '/aiot/string/tool_info', 10)
        self.led_pub = self.create_publisher(Bool, '/aiot/bool/led_command', 10)
        self.task_done = self.create_publisher(String, '/task/done', 10)
        
        self.cam_mat = None
        # self.grip_mat = np.array([])
        
        self.joint_degrees = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.M3_where = None
        
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
            self.tool_lst = []
            # self.tool_lst = ['wire_cutter', 'nipper']
            
    def box_return_goal_memory(self, pick_pose):
        p_x = pick_pose[0]
        print(1112312415124213412)
        
        candidates = [
            (0.08, 'left'),
            (0.0, 'middle'),
            (-0.08, 'right'),
        ]
        
        _, closest_name = min(candidates, key=lambda x: abs(p_x - x[0]))
        
        return closest_name
    
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
    # # self.grip_mat    
    # def grip_coor_callback(self, msg : Float32MultiArray):
    #     with self.data_lock:
    #         dims = msg.layout.dim
            
    #         if len(dims) < 2:
    #             self.get_logger().warn(' 잘못된 행렬 수신 ')
    #             return

    #         rows = dims[0].size
    #         cols = dims[1].size
            
    #         if len(msg.data) != rows * cols:
    #             self.get_logger().warn(f' msg count error : msg_count : {rows*cols}')
    #             return
            
    #         self.grip_mat = np.asarray(msg.data, dtype=np.float32).reshape(rows, cols)
    
    def present_position_callback(self, msg: Int32MultiArray):
        with self.data_lock:
            pulses = list(msg.data)
            
            for i in range(min(5, len(pulses))):
                self.joint_degrees[i] = ((pulses[i] - 2048) / 4096.0) * 360.0
    
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
            self.section   = obj.get('section')
            
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
            done = String()
            done.data = 'done'
            self.task_done.publish(done)
            
        elif next_step == 'check':
            with self.lock:
                if len(self.tool_lst) <= 0:
                    self.current_step = 'final'
                    self.current_flag = 'order'
                    self.next_step = 'None'
                
                else:
                    if len(self.tool_lst) == 1 and 'M3' in self.tool_lst:
                        self.current_step = 'M3_step_1'
                        self.current_flag = 'order'
                        self.next_step = None
                    else:
                        self.current_step = 'step_2'
                        self.current_flag = 'order'
                        self.next_step = None
            
        else:
            with self.lock:
                
                
                self.current_step = next_step
                self.current_flag = 'order'
                self.next_step = None
                    
    def create_handler(self, mode, tool):
        if mode == 'DELIVER':
            if tool == 'M3':
                return Box()
            
            else:
                return Deliver()
        
        elif mode == 'RETURN':
            return Return()
        
        elif mode == 'START':
            return Start()
        
        elif mode == 'FINISH':
            return Finish()
        
        elif mode == 'LIGHT_ON':
            return Fuck()
        
        elif mode == 'LIGHT_OFF':
            return Shit()
        
        return None
    
    def loop(self):
        with self.lock:
            handler = self.handler
            current_step = self.current_step
            current_flag = self.current_flag
            next_step = self.next_step
            tool_list = self.tool_lst
            M3_where   = self.M3_where
            jnt_deg   = self.joint_degrees

        # 모니터링: 현재 상태 출력
        self.get_logger().info(f'[MONITOR] Flag: {current_flag}, Step: {current_step}, Next: {next_step}')
        self.get_logger().info(f'M3   List = {M3_where}')
        self.get_logger().info(f'Tool List = {tool_list}')
        # self.get_logger().info(f'Joint Deg = {jnt_deg}')
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
        if ans.get('position'):
            cmd_msg = DongSooCommand()
            
            cmd_msg.frame    = ans['frame']
            
            if cmd_msg.frame == 'l':
                # 원본 리스트를 수정하지 않도록 복사본 생성
                lst = list(ans['position'])
                lst.append(0.0)
                cmd_msg.position = lst
            else:
                cmd_msg.position = ans['position']
                
            cmd_msg.look     = ans['look']
            cmd_msg.time     = ans['time']
            cmd_msg.wrist    = ans['wrist']
            
            self.cmd_pub.publish(cmd_msg)
            
            self.get_logger().info(f"[AIOT] Move to position")
        
        if ans.get('gripper'):
            grip_msg = String()
            
            grip_msg.data = ans['gripper']
            self.grip_pub.publish(grip_msg)
            
            self.get_logger().info(f"[AIOT] gripper")
            
        if ans.get('clear'):
            self.reset_tool_param()
            self.get_logger().info(f"[AIOT] clear")
            
        if ans.get('camera_trigger'):
            cam_msg = String()
            
            with self.lock:
                tool = self.tool
                self.tool_lst.append(tool)
                
            cam_msg.data = tool
            self.cam_pub.publish(cam_msg)
            self.get_logger().info(f"[AIOT] camera_trigger")
        
        ### =======================================
        ### Tool Deliver Module ===================
        if ans.get('camera_tool_move'):
            with self.lock:
                tool = self.tool
                tool_p = self.tool_p
                tool_yaw = self.tool_yaw
                
                
            cmd_msg = DongSooCommand()
            cmd_msg.frame = 'l'
            cmd_msg.position = [float(tool_p[0]), float(tool_p[1]) - 0.028, float(tool_p[2]) + 0.075, 0.0]
            cmd_msg.look     = 'straight'
            cmd_msg.time     = 7.0
            cmd_msg.wrist    = tool_yaw
            
            self.cmd_pub.publish(cmd_msg)
            
        if ans.get('camera_tool_up_1_move'):
            with self.lock:
                tool = self.tool
                tool_p = self.tool_p
                tool_yaw = self.tool_yaw
                
            cmd_msg = DongSooCommand()
            cmd_msg.frame = 'l'
            cmd_msg.position = [float(tool_p[0]), float(tool_p[1]) - 0.026, float(tool_p[2]) + 0.10, 0.0]
            cmd_msg.look     = 'straight'
            cmd_msg.time     = 0.8
            cmd_msg.wrist    = tool_yaw
            
            self.cmd_pub.publish(cmd_msg)
            
        if ans.get('camera_tool_up_2_move'):
            with self.lock:
                tool = self.tool
                tool_p = self.tool_p
                tool_yaw = self.tool_yaw
                
            cmd_msg = DongSooCommand()
            cmd_msg.frame = 'l'
            cmd_msg.position = [float(tool_p[0]), float(tool_p[1]) - 0.026, float(tool_p[2]) + 0.115, 0.0]
            cmd_msg.look     = 'straight'
            cmd_msg.time     = 0.8
            cmd_msg.wrist    = tool_yaw
            
            self.cmd_pub.publish(cmd_msg)
            
        if ans.get('camera_tool_up_3_move'):
            with self.lock:
                tool = self.tool
                tool_p = self.tool_p
                tool_yaw = self.tool_yaw
                
            cmd_msg = DongSooCommand()
            cmd_msg.frame = 'l'
            cmd_msg.position = [float(tool_p[0]), float(tool_p[1]) - 0.026, float(tool_p[2]) + 0.125, 0.0]
            cmd_msg.look     = 'straight'
            cmd_msg.time     = 0.8
            cmd_msg.wrist    = tool_yaw
            
            self.cmd_pub.publish(cmd_msg)
            
        if ans.get('camera_tool_back_move'):
            with self.lock:
                tool = self.tool
                tool_p = self.tool_p
                tool_yaw = self.tool_yaw
                
            cmd_msg = DongSooCommand()
            cmd_msg.frame = 'l'
            cmd_msg.position = [float(tool_p[0]), float(tool_p[1]) + 0.06, float(tool_p[2]) + 0.14, 0.0]
            cmd_msg.look     = 'straight'
            cmd_msg.time     = 3.0
            cmd_msg.wrist    = tool_yaw
            
            self.cmd_pub.publish(cmd_msg)
            
        
        ### Tool Deliver Module ===================
        ### =======================================
        
        ### =======================================
        ###  BOX Deliver Module =================== 
        if ans.get('camera_box_move'):
            with self.lock:
                tool = self.tool
                tool_p = self.tool_p
                tool_yaw = self.tool_yaw
                
                self.get_logger().warn('11111111111111111')
                self.M3_where = self.box_return_goal_memory(tool_p)
                
            cmd_msg = DongSooCommand()
            cmd_msg.frame = 'l'
            cmd_msg.position = [float(tool_p[0]), float(tool_p[1])-0.025, 0.035, 0.0]
            cmd_msg.look     = 'down'
            cmd_msg.time     = 3.0
            cmd_msg.wrist    = tool_yaw
            
            self.cmd_pub.publish(cmd_msg)
            
        if ans.get('camera_box_up_move'):
            with self.lock:
                tool = self.tool
                tool_p = self.tool_p
                tool_yaw = self.tool_yaw
                
            cmd_msg = DongSooCommand()
            cmd_msg.frame = 'l'
            cmd_msg.position = [float(tool_p[0]), float(tool_p[1])+0.01, float(tool_p[2]) + 0.12, 0.0]
            cmd_msg.look     = 'down'
            cmd_msg.time     = 3.0
            cmd_msg.wrist    = tool_yaw
            
            self.cmd_pub.publish(cmd_msg)
        ###  BOX Deliver Module ===================
        ### =======================================
        
        ### =======================================
        ### TOOL Return Module ===================
        if ans.get('camera_return_tool_up_move'):
            with self.lock:
                tool = self.tool
                tool_p = self.tool_p
                tool_yaw = self.tool_yaw
                
            cmd_msg = DongSooCommand()
            cmd_msg.frame = 'l'
            cmd_msg.position = [float(tool_p[0]), float(tool_p[1]), 0.15, 0.0]
            cmd_msg.look     = 'down'
            cmd_msg.time     = 3.0
            cmd_msg.wrist    = 0.0
            
            self.cmd_pub.publish(cmd_msg)
        
        if ans.get('camera_return_tool_move'):
            with self.lock:
                tool = self.tool_lst[0]
                tool_p = self.tool_p
                tool_yaw = self.tool_yaw
                base_deg = self.joint_degrees[0]
                wrist = tool_yaw + base_deg
                self.wrist = wrist
                
            cmd_msg = DongSooCommand()
            cmd_msg.frame = 'l'
            if tool == 'wire_cutter':
                cmd_msg.position = [float(tool_p[0]), float(tool_p[1]), float(tool_p[2]), 0.0]
            elif tool == 'nipper':
                cmd_msg.position = [float(tool_p[0]), float(tool_p[1]), float(tool_p[2]) + 0.003, 0.0]
            else:
                cmd_msg.position = [float(tool_p[0]), float(tool_p[1]), 0.03, 0.0]
                
            cmd_msg.look     = 'down'
            cmd_msg.time     = 2.0
            cmd_msg.wrist    = self.wrist
            
            self.cmd_pub.publish(cmd_msg)
            
        if ans.get('camera_return_tool_grip_up_move'):
            with self.lock:
                tool = self.tool
                tool_p = self.tool_p
                tool_yaw = self.tool_yaw
                base_deg = self.joint_degrees[0]
                wrist = self.wrist
                
            cmd_msg = DongSooCommand()
            cmd_msg.frame = 'l'
            cmd_msg.position = [float(tool_p[0]), float(tool_p[1]), float(tool_p[2]) + 0.1, 0.0]
            cmd_msg.look     = 'down'
            cmd_msg.time     = 1.0
            cmd_msg.wrist    = wrist
            
            self.cmd_pub.publish(cmd_msg)

        if ans.get('return_camera_trigger'):
            cam_msg = String()
            
            with self.lock:
                tool = self.tool_lst[0]
                # 남아있는 도구에서 현재 집으러갈 도구 제거
                
            cam_msg.data = tool
            self.cam_pub.publish(cam_msg)
            self.get_logger().info(f"[AIOT] return camera_trigger")
        
        if ans.get('camera_return_box_up_move'):
            with self.lock:
                tool = self.tool
                tool_p = self.tool_p
                tool_yaw = self.tool_yaw
                
            cmd_msg = DongSooCommand()
            cmd_msg.frame = 'l'
            cmd_msg.position = [float(tool_p[0]), float(tool_p[1]), float(tool_p[2]) + 0.13, 0.0]
            cmd_msg.look     = 'down'
            cmd_msg.time     = 3.0
            cmd_msg.wrist    = tool_yaw
            
            self.cmd_pub.publish(cmd_msg)
        
        if ans.get('camera_return_box_move'):
            with self.lock:
                tool = self.tool
                tool_p = self.tool_p
                tool_yaw = self.tool_yaw
                
            cmd_msg = DongSooCommand()
            cmd_msg.frame = 'l'
            cmd_msg.position = [float(tool_p[0])+ 0.03 * np.cos(np.deg2rad(tool_yaw)), float(tool_p[1]) + 0.03 * np.sin(np.deg2rad(tool_yaw)), float(tool_p[2]) - 0.01, 0.0]
            cmd_msg.look     = 'down'
            cmd_msg.time     = 1.5
            cmd_msg.wrist    = tool_yaw
            
            self.cmd_pub.publish(cmd_msg)
            
        if ans.get('camera_return_box_grip_up_move'):
            with self.lock:
                tool = self.tool
                tool_p = self.tool_p
                tool_yaw = self.tool_yaw
                
            cmd_msg = DongSooCommand()
            cmd_msg.frame = 'l'
            cmd_msg.position = [float(tool_p[0]), float(tool_p[1]), float(tool_p[2]) + 0.01, 0.0]
            cmd_msg.look     = 'down'
            cmd_msg.time     = 1.0
            cmd_msg.wrist    = tool_yaw
            
            self.cmd_pub.publish(cmd_msg)
            
        if ans.get('camera_return_box_drop_top_move'):
            with self.lock:
                M3_where = self.M3_where
                
            cmd_msg = DongSooCommand()
            cmd_msg.frame = 'l'
            
            if M3_where == 'left':
                cmd_msg.position = [0.085, -0.34, 0.18, 0.0]
                cmd_msg.wrist    = 10.0
                
            elif M3_where == 'right':
                cmd_msg.position = [-0.085, -0.34, 0.18, 0.0]
                cmd_msg.wrist    = -10.0
                
            else:
                cmd_msg.position = [0.0, -0.33, 0.18, 0.0]
                cmd_msg.wrist    = 0.0
            
            cmd_msg.look     = 'down'
            cmd_msg.time     = 1.0
            
            self.cmd_pub.publish(cmd_msg)
            
        if ans.get('camera_return_box_drop_move'):
            with self.lock:
                M3_where = self.M3_where
                
            cmd_msg = DongSooCommand()
            cmd_msg.frame = 'l'
            
            if M3_where == 'left':
                cmd_msg.position = [0.085, -0.365, 0.058, 0.0]
                cmd_msg.wrist    = 10.0
                
            elif M3_where == 'right':
                cmd_msg.position = [-0.085, -0.365, 0.058, 0.0]
                cmd_msg.wrist    = -10.0
                
            else:
                cmd_msg.position = [0.0, -0.37, 0.058, 0.0]
                cmd_msg.wrist    = 0.0
            
            cmd_msg.look     = 'down'
            cmd_msg.time     = 1.0
            
            self.cmd_pub.publish(cmd_msg)
            
        if ans.get('return_list_clean'):
            with self.lock:
                self.tool_lst.pop(0)
                
        if ans.get('M3_list_clean'):
            with self.lock:
                self.M3_where = None
                
        ### Return Module ===================
        ### =======================================
        
        ### ======================================= 
        ### LED LED LED LED LED LED LED LED LED LED
        if ans.get('led_on'):
            with self.lock:
                section = self.section
                section = str(section)

            cmd_msg = DongSooCommand()
            cmd_msg.frame = 'j'
            
            if section == '1':
                lst = [1.4, -27.86, -52.83, -62.67]
                lst_float = [float(x) for x in lst]
                cmd_msg.position = lst_float
                
            elif section == '2':
                lst = [1.9, 27.07, -111.09, -54.0]
                lst_float = [float(x) for x in lst]
                cmd_msg.position = lst_float
                
            elif section == '3':
                lst = [-20.65, -28.39, -55.46, -49.52]
                lst_float = [float(x) for x in lst]
                cmd_msg.position = lst_float
                
            cmd_msg.wrist    = 0.0
            cmd_msg.look = 'down'
            cmd_msg.time = 3.0
            
            self.cmd_pub.publish(cmd_msg)
            
            led_msg = Bool()
            led_msg.data = True
            self.led_pub.publish(led_msg)
        
        if ans.get('led_off'):
            led_msg = Bool()
            led_msg.data = False
            self.led_pub.publish(led_msg)
            
                
        
            
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