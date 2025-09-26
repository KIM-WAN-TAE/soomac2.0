#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, MultiArrayDimension
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from zeus_interfaces.srv import ZeusExecutor

import threading, time
import numpy as np
from .read_json import CameraDHParameters
import copy

Z_OFFSET  = 200.0
PITCH_TOL = 3.0
YAW_TOL   = 3.0
ROLL_TOL  = 10.0
PICK_Z_OFFSET = 10.0

def dh_transform(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    
    T = np.array([[ ct, -st*ca,  st*sa, a*ct],
                  [ st,  ct*ca, -ct*sa, a*st],
                  [  0,      sa,     ca,    d],
                  [  0,       0,      0,    1]])
    return T

def fk(dh_params):
    T = np.eye(4)
    for params in dh_params:
        T = T @ dh_transform(params['theta'], params['d'], params['a'], params['alpha'])
    return T

def rot_to_euler_zyx(R):
    r20 = R[2,0]
    if abs(r20) < 1.0 - 1e-9:
        ry = np.arcsin(-r20)
        rz = np.arctan2(R[1,0], R[0,0])
        rx = np.arctan2(R[2,1], R[2,2])
    else:
        ry = np.pi/2 if r20 <= -1.0 else -np.pi/2
        rz = 0.0
        rx = np.arctan2(-R[0,1], R[1,1])
    return np.rad2deg(rz), np.rad2deg(ry), np.rad2deg(rx)

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
        
        self.lock = threading.Lock()
        
        self.speed_cmd_idx_map = {
            0: ['jntspd', 10],
            1: ['linspd', 40],
            2: ['linspd', 40],
            3: ['posspd', 80],
            4: ['linspd', 5],
            5: ['posspd', 30],
            6: ['linspd', 80],
            7: ['jntspd', 40],
            8: ['linspd', 40],
            9: ['jntspd', 5],
            10: ['jntspd', 10],
            12: ['jntspd', 40]
        }
        self.sent_speed_cmd_idx = set()
        
        self.reset_block_list()

        self.idx = 0
        self.block_trigger = True
        self.is_busy = False
        self.block_rpy = None

        self.dh_params = CameraDHParameters()
        self.base_to_camera_matrix = None
        self.xy_coor = None
        self.block_mat = None
        self.joint_coor = None
        self.block_pose = None
        self.topic_flag = False
        
        self.service_cb_group = ReentrantCallbackGroup()
        self.sub_cb_group = ReentrantCallbackGroup()
        self.timer_cb_group = ReentrantCallbackGroup()
        self.dh_timer_cb_group = ReentrantCallbackGroup()

        self.cli = self.create_client(ZeusExecutor, '/zeus_exec')
        
        self.block_pose_order_pub = self.create_publisher(String, '/zeus/string/block_order', 10)
        self.base_to_camera_pub = self.create_publisher(Float32MultiArray, '/zeus/array/base_to_cam_matrix', 10)
        self.gripper_command_pub = self.create_publisher(String, '/zeus/string/gripper_command', 10)
        self.gripper_wall_command_pub = self.create_publisher(String, '/zeus/string/target_angle', 10)
        self.color_count_pub = self.create_publisher(String, '/zeus/string/drop_done', 10)
        self.binary_cmd = self.create_publisher(String, '/zeus/string/binary_command', 10)
        
        self.create_subscription(Float32MultiArray, '/zeus/array/block_pose', self.block_pose_callback, 10, callback_group=self.sub_cb_group)
        self.create_subscription(Float32MultiArray, '/zeus/array/xy_state', self.xy_state_callback, 10, callback_group=self.sub_cb_group)
        self.create_subscription(Float32MultiArray, '/zeus/array/joint_state', self.joint_state_callback, 10, callback_group=self.sub_cb_group)
        self.create_subscription(Float32MultiArray, '/zeus/array/drop_point', self.drop_zone_callback, 10, callback_group=self.sub_cb_group)
        
        self.create_timer(1/10, self.timer, callback_group=self.timer_cb_group)
        self.create_timer(1/10, self.dh_timer, callback_group=self.dh_timer_cb_group)
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('[ZEUS] ZeusExecutor 서비스 서버 대기 중...')
        self.get_logger().info('[ZEUS] 서비스 연결 완료')
        
        self.send_next_command()
        
    def reset_block_list(self):
        # 동적 리스트들 초기화
        self.BLOCK_PICK_TOP  = []
        self.BLOCK_SPECIAL   = []
        self.BLOCK_MAKE_ORI  = []
        self.BLOCK_PICK      = []
        self.BLOCK_DROP_TOP  = []
        self.BLOCK_DROP      = []

        # 고정값들 정의
        CAM_INIT = ['j', -97.57, -11.23, -73.43, 0.14, -94.65, -97.56]
        BLOCK_DROP_INIT = ['j', -15.75, -27.47, -95.23, 0.20, -57.54, -102.09]
        GRIPPER_TIME    = ['t', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.block_pose = None
        self.topic_flag = False
        self.block_mat = None
        self.idx = 0
        
        # block_list 재구성
        self.block_list = [
            CAM_INIT,             # 0 Joint -> jntspd = 10
            self.BLOCK_PICK_TOP,  # 1 Linear linspd = 
            self.BLOCK_SPECIAL,   # 2 Linear
            self.BLOCK_MAKE_ORI,  # 3 tool-rel -> 여기서 현 Pose Memo
            self.BLOCK_PICK,      # 4 Linear
            GRIPPER_TIME,         # 5 etc
            self.BLOCK_MAKE_ORI,  # 6 Linear   -> 현 Pose 불러오기
            BLOCK_DROP_INIT,      # 7 Joint
            self.BLOCK_DROP_TOP,  # 8 lin 놓는 곳 수직 위치
            self.BLOCK_DROP,      # 9 tool 놓는 곳 
            self.BLOCK_DROP_TOP,  # 10 Joint 놓는 곳 수직 위치
            BLOCK_DROP_INIT,      # 11 Joint jntspd = 8
            CAM_INIT              # 12 Joint jntspd = 20
        ]
        
        self.sent_speed_cmd_idx.clear()
        
        self.get_logger().info('[ZEUS] Block list가 초기화되었습니다.')
        
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
                   
    def cal_base_to_cam(self, joint_angles):
        all_dh_params = self.dh_params.get_all_dh_params(joint_angles[:6])
    
        T_BC = fk(all_dh_params)
        # print(f'\n{T_BC}')
        return T_BC
    
    def dh_timer(self):
        with self.lock:
            if self.base_to_camera_matrix is None:
                return
            
            T = self.base_to_camera_matrix
        rows, cols = T.shape
        
        msg = Float32MultiArray()
        msg.layout.dim.append(MultiArrayDimension(label='rows', size=rows, stride=cols))
        msg.layout.dim.append(MultiArrayDimension(label='cols', size=cols, stride=1))
        msg.layout.data_offset = 0
        msg.data = T.flatten().tolist()
        self.base_to_camera_pub.publish(msg)
        
    def response_callback(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().warn(f'[ZEUS] 서비스 호출 실패: {e}')
            with self.lock:
                self.idx += 1
                self.is_busy = False
            return

        if res.success:
            self.get_logger().info(f'[ZEUS] {self.idx+1}번째 명령 성공')
        else:
            self.get_logger().warn(f'[ZEUS] {self.idx+1}번째 명령 실패')
            
        with self.lock:
            self.idx += 1
            self.is_busy = False
            
    def send_speed(self, frame, value):
        cmd = mparam_command_string(frame, value)
        msg = String()
        msg.data = str(cmd)
        self.binary_cmd.publish(msg)
        
    def send_speed_command_for_idx(self, idx):
        with self.lock:
            if idx in self.speed_cmd_idx_map and idx not in self.sent_speed_cmd_idx:
                param_type, value = self.speed_cmd_idx_map[idx]
                self.send_speed(param_type, value)
                self.sent_speed_cmd_idx.add(idx)
                print(f'Speeeeeeeeeeeed : {idx}, {value}')
        
    def timer(self):
        with self.lock:
            trigger = self.block_trigger
            idx = self.idx
            is_busy = self.is_busy
            block_pose = self.block_pose
            
        if trigger and not is_busy:
            self.send_speed_command_for_idx(idx)
            
            if idx == 0:
                
                self.send_next_command()
                  
            elif idx == 1:
                
                if self.base_to_camera_matrix is None:
                    self.get_logger().info('[ZEUS] Waiting for base_to_camera_matrix...')
                    return
                
                with self.lock:
                    if self.topic_flag is False:
                        s_msg = String()
                        print('block0')
                        s_msg.data = 'block'
                        self.block_pose_order_pub.publish(s_msg)
                        self.topic_flag = True
                
                if block_pose is None:
                    # self.get_logger().info('[ZEUS] Waiting Block Pose')
                    return
                
                wall_msg = String()
                wall_msg.data = 'down'
                self.gripper_wall_command_pub.publish(wall_msg)
                
                P, rz, ry, rx = block_pose
                    
                pose = [P[0]-85.0, P[1], Z_OFFSET, rz, ry, rx]
                pose[3:] = self.xy_coor[3:]
                 
                self.block_list[idx] = ['l'] + pose
                self.send_next_command()
            
                with self.lock:
                    self.block_pose = None
                    self.topic_flag = False
                    
            elif idx == 2:
                with self.lock:
                    if self.topic_flag is False:
                        s_msg = String()
                        print('block1')
                        s_msg.data = 'block'
                        self.block_pose_order_pub.publish(s_msg)
                        self.topic_flag = True
                
                    if self.block_pose is None:
                        # self.get_logger().info('[ZEUS] Waiting Block Pose')
                        return
                    
                    B_P, rz, ry, rx = self.block_pose
            
                yaw, pitch, roll = rz, ry, rx               
                
                Z_HEIGHT = Z_OFFSET - B_P[2]
                move_dis = Z_HEIGHT * np.tan(abs(np.deg2rad(pitch)))
                
                if yaw < -90.0: # 월드 좌표계 기준 YAW의 방향벡터가 3사분면 -> 1사분면으로 이동
                    yaw += 180
                    if pitch > PITCH_TOL: # Pitch 양수 # 1
                        print(1)
                        x_move = - move_dis * np.cos(np.deg2rad(yaw))  
                        y_move = - move_dis * np.sin(np.deg2rad(yaw))
                        
                    elif pitch < - PITCH_TOL: # Pitch 음수 # 2
                        print(2)
                        x_move =   move_dis * np.cos(np.deg2rad(yaw))  
                        y_move =   move_dis * np.sin(np.deg2rad(yaw))
                        
                    else: # 그냥 평평할 경우
                        x_move, y_move = 0.0, 0.0
                        
                elif yaw > -90.0: # 월드 좌표계 기준 YAW의 방향벡터가 4사분면
                    if pitch > PITCH_TOL: # Pitch 양수 # 3
                        print(3)
                        x_move =   move_dis * np.cos(np.deg2rad(yaw))  
                        y_move = - move_dis * np.sin(np.deg2rad(yaw))
                        
                    elif pitch < - PITCH_TOL: # Pitch 음수 # 4
                        print(4)
                        x_move = - move_dis * np.cos(np.deg2rad(yaw))  
                        y_move =   move_dis * np.sin(np.deg2rad(yaw))
                        
                    else: # 그냥 평평할 경우
                        x_move, y_move = 0.0, 0.0
                        
                print(f'\n x : {x_move}, y : {y_move}\n')
                
                pose = [B_P[0] + x_move, B_P[1] + y_move, Z_OFFSET, rz, ry, rx]
                with self.lock:
                    pose[3:] = self.xy_coor[3:]
                
                print(f'POSE : {pose}')
                self.block_list[idx] = ['l'] + pose
                self.send_next_command()
                
                with self.lock:
                    self.topic_flag = False
                    
            elif idx == 3:
                with self.lock:
                    c_rz, c_ry, c_rx = self.xy_coor[3:]
                _, yaw, pitch, roll = block_pose
                    
                e_rz, e_ry, e_rx = yaw - c_rz, pitch - c_ry, roll - c_rx
                
                if abs(e_rx) > 30 and abs(e_rx) < 60:
                    if yaw < -90.0: # 월드 좌표계 기준 YAW의 방향벡터가 3사분면 -> 1사분면으로 이동
                        yaw += 90
                        yaw = abs(yaw)
                        
                    elif yaw > -90.0: # 월드 좌표계 기준 YAW의 방향벡터가 4사분면
                        yaw += 90
                        yaw = -abs(yaw)
                    pitch = 0.0
                    
                else:
                    if yaw < -90.0: # 월드 좌표계 기준 YAW의 방향벡터가 3사분면 -> 1사분면으로 이동
                        yaw += 90
                        yaw = abs(yaw)
                        if pitch > PITCH_TOL or pitch < - PITCH_TOL:
                            pass
                        else: # 그냥 평평할 경우
                            pitch = 0.0
                        
                    elif yaw > -90.0: # 월드 좌표계 기준 YAW의 방향벡터가 4사분면
                        yaw += 90
                        yaw = -abs(yaw)
                        if pitch > PITCH_TOL or pitch < - PITCH_TOL:
                            pass
                        else: # 그냥 평평할 경우
                            pitch = 0.0
                            
                    roll = 0.0
                
                print(f'\nyaw : {yaw}, pitch : {pitch}, roll : {roll}')
                print(f'eyaw : {e_rz}, epitch : {e_ry}, eroll : {e_rx} \n')
                  
                pose = [0.0, 0.0, 0.0, yaw, pitch, 0.0]
                self.block_list[idx] = ['t'] + pose
                self.send_next_command()
                    
            elif idx == 4:
                self.send_speed('linspd', 10)
                P, rz, ry, rx = block_pose
                    
                pose = [P[0], P[1], P[2] + PICK_Z_OFFSET, rz, ry, rx]
                with self.lock:
                    pose[3:] = self.xy_coor[3:]
                 
                self.block_list[idx] = ['l'] + pose
                self.send_next_command()
 
            elif idx == 5:
                gripper_msg = String()
                gripper_msg.data = 's'
                self.gripper_command_pub.publish(gripper_msg)
                
                grip_done_msg = String()
                grip_done_msg.data = 'done'
                self.color_count_pub.publish(grip_done_msg)
                
                time.sleep(2)
                
                self.send_next_command()
            
            elif idx == 6:
                with self.lock:
                    self.block_list[idx] = copy.deepcopy(self.block_list[idx-4])
                
                self.send_next_command()
                
            elif idx == 7: # Move Drop Init Position
                self.send_next_command()
            
            elif idx == 8: # Drop Top Zone
                wall_msg = String()
                wall_msg.data = 'up'
                self.gripper_wall_command_pub.publish(wall_msg)
                
                with self.lock:
                    self.block_list[idx] = ['l'] + list(self.drop_zone_point)
                    
                self.send_next_command()
                
            elif idx == 9:
                with self.lock:
                    self.block_list[idx] = ['t'] + [0.0, 0.0, 10.0, 0.0, 0.0, 0.0]
                    
                self.send_next_command()
                
            elif idx == 10:
                gripper_msg = String()
                gripper_msg.data = 'e'
                self.gripper_command_pub.publish(gripper_msg)
                
                time.sleep(2)
                
                with self.lock:
                    self.block_list[idx] = ['l'] + list(self.drop_zone_point)
                
                self.send_next_command()
                
            elif idx == 11:
                self.send_next_command()
                
            elif idx == 12:
                self.send_next_command()
                
            elif idx == 13:
                self.send_next_command()

        elif trigger and is_busy:
            # self.get_logger().info(f"[ZEUS] I'm moving! ")
            pass
            
        else:
            pass
            # self.get_logger().info(f"[ZEUS] Waiting For True Trigger ")
        
    def send_next_command(self):
        if self.is_busy: # 이미 명령을 보낸 경우 
            return
        
        with self.lock:
            if self.idx >= len(self.block_list):
                self.get_logger().info('[ZEUS] All Coordinate Sended!')
                
                self.reset_block_list()
                print(f'{self.idx}')
                self.block_trigger = True
                self.is_busy = False
                return
        
        self.is_busy = True
           
        block = self.block_list[self.idx]
        frame = block[0]
        coor = block[1:]

        req = ZeusExecutor.Request()
        req.frame = frame
        req.coordinate = [float(x) for x in coor]

        self.get_logger().info(f'[ZEUS] {self.idx+1}/{len(self.block_list)}번째 명령 전송: frame={frame}, coor={coor}')
        future = self.cli.call_async(req)
        future.add_done_callback(self.response_callback)
        
        self.is_busy = True
        
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
        
        with self.lock:
            self.block_pose = [P, rz, ry, rx]
            
        # print(rz, ry, rx)
        # print(P[0], P[1], P[2])
            
def main(args=None):
    rclpy.init(args=args)
    node = ZeusClientNode()
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()