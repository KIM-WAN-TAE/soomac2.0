#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from zeus_interfaces.srv import ZeusExecutor

import threading, time
import numpy as np
from .read_json import CameraDHParameters

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

class ZeusClientNode(Node):
    def __init__(self):
        super().__init__('zeus_client_node')
        
        self.lock = threading.Lock()
        
        CAM_INIT = ['j', -86.16, -15.19, -107.19, 0.0, -56.92, -86.16]
        CAM_TEST = ['j', -70.16, -15.19, -107.19, 0.0, -56.92, -86.16]
        
        self.block_list = [
            CAM_INIT,
            CAM_TEST,
            CAM_INIT,
            CAM_TEST,
            CAM_INIT
        ]
        
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

        self.cli = self.create_client(ZeusExecutor, '/zeus_exec')
        
        self.block_pose_order_pub = self.create_publisher(String, '/zeus/string/block_order', 10)
        
        # self.create_subscription(Float32MultiArray, '/zeus/xyzrpy/block_rpy', self.block_rpy_callback, 10, callback_group=self.sub_cb_group)
        self.create_subscription(Float32MultiArray, '/zeus/array/block_pose', self.block_pose_callback, 10, callback_group=self.sub_cb_group)
        self.create_subscription(Float32MultiArray, '/zeus/array/xy_state', self.xy_state_callback, 10, callback_group=self.sub_cb_group)
        self.create_subscription(Float32MultiArray, '/zeus/array/joint_state', self.joint_state_callback, 10, callback_group=self.sub_cb_group)
        
        self.create_timer(1/10, self.timer, callback_group=self.timer_cb_group)
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('[ZEUS] ZeusExecutor 서비스 서버 대기 중...')
        self.get_logger().info('[ZEUS] 서비스 연결 완료')
        
        self.send_next_command()
        
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

    def timer(self):
        with self.lock:
            trigger = self.block_trigger
            idx = self.idx
            is_busy = self.is_busy
            
        if trigger and not is_busy:
            if idx == 0:
                self.send_next_command()
                
            elif idx == 1:
                with self.lock:
                    if self.topic_flag is False:
                        s_msg = String()
                        print('block')
                        s_msg.data = 'block'
                        self.block_pose_order_pub.publish(s_msg)
                        self.topic_flag = True
                
                with self.lock:
                    if self.block_pose is None:
                        # self.get_logger().info('[ZEUS] Waiting Block Pose')
                        return
                
                with self.lock:
                    P, rz, ry, rx = self.block_pose
                    pose = [P[0], P[1], 300.0, rz, ry, rx]
                    print(self.xy_coor[3:])
                    pose[3:] = self.xy_coor[3:]
                    
                    # P = self.block_rpy[:3]
                    # RPY = self.block_rpy[3:]
                    # pose = [P[0], P[1], 300.0, RPY[2], RPY[1], RPY[0]]
                    
                self.block_list[1] = ['l'] + pose
   
                self.send_next_command()
                
                with self.lock:
                    self.block_pose = None
                    self.topic_flag = False
            
            elif idx == 2:
                with self.lock:
                    if self.topic_flag is False:
                        s_msg = String()
                        print('block')
                        s_msg.data = 'block'
                        self.block_pose_order_pub.publish(s_msg)
                        self.topic_flag = True
                
                with self.lock:
                    if self.block_pose is None:
                        # self.get_logger().info('[ZEUS] Waiting Block Pose')
                        return
                
                time.sleep(2)
                
                with self.lock:
                    # P = self.block_rpy[:3]
                    # RPY = self.block_rpy[3:]
                    # pose = [P[0], P[1], 300.0, RPY[2], RPY[1], RPY[0]]
                    
                    P, rz, ry, rx = self.block_pose
                self.get_logger().info(f'rz : {rz} / ry : {ry} / rx : {rx}')
                pose = [P[0], P[1], 280.0, rz, ry, rx]
                self.block_list[2] = ['l'] + pose
   
                self.send_next_command()
                
                with self.lock:
                    self.block_pose = None
                    self.topic_flag = False
                
            
        elif trigger and is_busy:
            # self.get_logger().info(f"[ZEUS] I'm moving! ")
            pass
            
        else:
            self.get_logger().info(f"[ZEUS] Waiting For True Trigger ")
        
    def send_next_command(self):
        if self.is_busy: # 이미 명령을 보낸 경우 
            return
        
        if self.idx >= len(self.block_list):
            self.get_logger().info('[ZEUS] All Coordinate Sended!')
            self.block_trigger = False
            self.idx = 0
            return

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
        
    # def block_rpy_callback(self, msg : Float32MultiArray):
    #     if len(msg.data) != 6:
    #         return

    #     with self.lock:
    #         self.block_rpy = msg.data
            
    #         print(f'{self.block_rpy}')
        
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
            print(f"완태형의 사고 : {self.block_mat[:3, 3]}")
            T_BO = self.base_to_camera_matrix @ self.block_mat 
        
        P = T_BO[:3, 3]    
        R = T_BO[:3,:3]
        
        rz, ry, rx = rot_to_euler_zyx(R)
        
        with self.lock:
            self.block_pose = [P, rz, ry, rx]
            
        print(rz, ry, rx)
        print(P[0], P[1], P[2])
            
def main(args=None):
    rclpy.init(args=args)
    node = ZeusClientNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()