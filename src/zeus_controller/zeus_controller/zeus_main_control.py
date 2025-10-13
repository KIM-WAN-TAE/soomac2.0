#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String, Float32MultiArray, Float32
from zeus_interfaces.msg import ZeusMainCommand

from zeus_controller.module import *
from zeus_controller.dh_module import *
from zeus_controller.read_json import CameraDHParameters

import numpy as np
import threading
import json

RATE = 10
TIMER_PERIOD = 1/RATE
Z_OFFSET = 200.0
Z_OFFSET_  = 250.0
PICK_Z_OFFSET = 23.0

class MainControlNode(Node):
    def __init__(self):
        super().__init__('main_control')
        self.lock = threading.Lock()
        
        gripper_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # 메시지 전달 보장
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # 늦게 연결된 구독자도 메시지 수신
            history=HistoryPolicy.KEEP_LAST,  # 마지막 N개 메시지 유지
            depth=1  # 큐에 보관할 메시지 개수
        )
        
        self.cmd_pub      = self.create_publisher(ZeusMainCommand, '/zeus/custom/client_command', 10)
        self.grip_cmd_pub = self.create_publisher(String, '/zeus/string/gripper_command', 10, qos_profile=gripper_qos_profile)
        self.cam_pub      = self.create_publisher(String, '/zeus/string/block_order', 10)
        self.done_pub     = self.create_publisher(String, '/zeus/string/drop_done', 10)
        
        self.create_subscription(Float32MultiArray, '/zeus/array/block_pose', self.block_pose_callback, 10)
        self.create_subscription(Float32MultiArray, '/zeus/array/xy_state', self.xy_state_callback, 10)
        self.create_subscription(Float32MultiArray, '/zeus/array/joint_state', self.joint_state_callback, 10)
        self.create_subscription(Float32MultiArray, '/zeus/array/drop_point', self.drop_zone_callback, 10)
        
        self.dh_params = CameraDHParameters()
        
        self.create_timer(TIMER_PERIOD, self.loop)
        
        self.reset_param()
        self.reset_block_pose()
        
        self.handler = Block()
        self.current_flag = 'order'
        
    def reset_param(self):
        with self.lock:
            self.handler = None
            self.current_step = None
            self.next_step = None
            
            self.current_flag = 'idle'

    def reset_block_pose(self):
        with self.lock:
            self.block_pose = None
            
    def cal_base_to_cam(self, joint_angles):
        all_dh_params = self.dh_params.get_all_dh_params(joint_angles[:6])
    
        T_BC = fk(all_dh_params)
        # print(f'\n{T_BC}')
        return T_BC
    
    # ======================================
    # == callback == callback == callback ==
    
    def drop_zone_callback(self, msg : Float32MultiArray):
        if len(msg.data) != 6:
            self.get_logger().warn('[ZEUS] 잘못된 데이터 길이입니다.')
            return
        
        with self.lock:
            self.drop_zone_point = msg.data
            self.get_logger().info(f'[ZEUS] Drop Zone Point : {self.drop_zone_point}')
    
    def joint_state_callback(self, msg):
        with self.lock:
            if len(msg.data) == 6:
                self.joint_coor = msg.data
        # self.get_logger().info(f'[ZEUS] joint_coor: {self.joint_coor}')
                self.base_to_camera_matrix = self.cal_base_to_cam(np.deg2rad(list(self.joint_coor)))
                
    def xy_state_callback(self, msg):
        with self.lock:
            if len(msg.data) == 6:
                self.xy_coor = msg.data
                
    def block_pose_callback(self, msg : Float32MultiArray):
        if len(msg.data) != 16:
            return  
        dims = msg.layout.dim
        
        if len(dims) < 2:
            self.get_logger().warn(' 잘못된 행렬 수신 ')
            return

        rows = dims[0].size
        cols = dims[1].size
        
        if len(msg.data) != rows * cols:
            self.get_logger().warn(f' msg count error : msg_count : {rows*cols}')
            return
        with self.lock:
            self.block_mat = np.asarray(msg.data, dtype=np.float32).reshape(rows, cols)
            if self.base_to_camera_matrix is None:
                self.get_logger().warn('[ZEUS] base_to_camera_matrix is None')
                return
            if not (isinstance(self.base_to_camera_matrix, np.ndarray) and self.base_to_camera_matrix.shape == (4,4)):
                self.get_logger().warn(f'[ZEUS] base_to_camera_matrix shape error: {getattr(self.base_to_camera_matrix, "shape", None)}')
                return
            T_BO = self.base_to_camera_matrix @ self.block_mat
        
        P = T_BO[:3, 3]    
        R = T_BO[:3,:3]
        
        rz, ry, rx = rot_to_euler_zyx(R)
        
        # print(f'rz : {rz}')
        
        # print(rz, ry, rx)
        print("")
        print(P[0], P[1], P[2])
        print("")
        
        with self.lock:
            self.block_pose = [P, rz, ry, rx]
            if self.current_flag == 'waiting':
                self.current_flag = 'done'
    
    # == callback == callback == callback ==
    # ======================================
    
    # ==================================
    # == 동작 정의 함수 == 동작 정의 함수 ==
    def module_translator(self, ans):
        # 일반 움직임
        if ans.get('position'):
            cmd_msg = ZeusMainCommand()
            
            cmd_msg.frame    = ans['frame']
            cmd_msg.position = ans['position']
            cmd_msg.speed    = ans['speed']
            self.cmd_pub.publish(cmd_msg)
            
            self.get_logger().info(f"[ZEUS] Move to position")
        
        # 그리퍼 동작
        if ans.get('gripper'):
            grip_msg = String()
            grip_msg.data = ans['gripper_str']
            
            self.grip_cmd_pub.publish(grip_msg)
            
            self.get_logger().info(f"[ZEUS] Gripper able or disable")
        
        # 드랍 좌표 요청
        if ans.get('drop_coor_order'):
            done_msg = String()
            done_msg.data = 'done'
            
            self.done_pub.publish(done_msg)
            
        # 블록 좌표 요청
        if ans.get('block_order'):
            cam_msg = String()
                
            cam_msg.data = ans['detect_str']
            self.cam_pub.publish(cam_msg)
            self.get_logger().info(f"[ZEUS] First Block Order : {cam_msg.data}")
        
        # 블록 집으러 가는 동작
        if ans.get('block_pick_move'):
            with self.lock:
                block_pose   = self.block_pose
                current_coor = self.xy_coor
                
            cmd_msg = ZeusMainCommand()
            P, rz, ry, rx = block_pose
            yaw = rz
            
            # 1차 Detect 위치로 이동   
            if ans['pick_str'] == 'first':
                pose = [P[0], P[1], Z_OFFSET_, 0.0, 0.0, 0.0]
                pose[3:] = current_coor[3:]
                
                cmd_msg.frame = 'l'
                self.reset_block_pose() # 1차 -> 2차로 넘어갈 땐 새로운 좌표 받아야함
            
            # Yaw 회전만 시행
            elif ans['pick_str'] == 'second':
                if yaw < -90.0:
                    yaw += 90
                    yaw = abs(yaw)
                
                elif yaw >= -90.0:
                    yaw += 90
                    yaw = -abs(yaw)

                pose = [0.0, 0.0, 0.0, yaw, 0.0, 0.0]
                cmd_msg.frame = 't'
            
            elif ans['pick_str'] == 'third':
                pose = [P[0], P[1], P[2] + PICK_Z_OFFSET, 0.0, 0.0, 0.0]
                pose[3:] = current_coor[3:]
                
                cmd_msg.frame = 'l'
            
            cmd_msg.position = pose
            cmd_msg.speed = ans['speed']
            
            self.cmd_pub.publish(cmd_msg)
            
        if ans.get('block_drop_move'):
            with self.lock:
                drop_zone = self.drop_zone_point 
                
            cmd_msg = ZeusMainCommand()
            cmd_msg.frame = 'j'
            cmd_msg.position = drop_zone
            cmd_msg.speed = ans['speed']
            
            self.cmd_pub.publish(cmd_msg)
            
        if ans.get('wait_a_sec'):
            import time
            
            t = ans['time']
            start_time = time.time()
            
            while True:
                print(f'[ZEUS] Waiting Time : {(t - (time.time() - start_time)):.2}')
                if time.time() - start_time >= t:
                    break
            
            with self.lock:
                # 석션 과정 중 혹시 Drop 좌표가 넘어오지 않았을 경우 좌표 2차 요청
                if self.drop_zone_point is None:
                    again_msg = String()
                    again_msg.data = 'again'
                    self.done_pub.publish(again_msg)
                
                if self.current_flag == 'waiting':
                    self.current_flag = 'done'
        
    # == 동작 정의 함수 == 동작 정의 함수 ==        
    # ==================================
    
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
    
    # ==================================
    # == loop == loop == loop == loop == 
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
            
            if requires_ack:
                with self.lock:
                    self.next_step = ans.get('next_step')
                    self.current_flag = 'waiting'
                    
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
            self.reset_param()
            
    # == loop == loop == loop == loop ==        
    # ==================================
    
def main(args=None):
    rclpy.init(args=args)
    node = MainControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nShutting down ZEUS Main Client...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()