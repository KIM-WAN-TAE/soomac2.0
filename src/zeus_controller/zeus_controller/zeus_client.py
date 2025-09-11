#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from zeus_interfaces.srv import ZeusExecutor

import threading
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
    # Z-Y-X (yaw-pitch-roll): R = Rz(rz) @ Ry(ry) @ Rx(rx)
    r20 = R[2,0]
    if abs(r20) < 1.0 - 1e-9:
        ry = np.arcsin(-r20)
        rz = np.arctan2(R[1,0], R[0,0])
        rx = np.arctan2(R[2,1], R[2,2])
    else:
        # gimbal lock
        ry = np.pi/2 if r20 <= -1.0 else -np.pi/2
        rz = 0.0
        # when locked, use alternative for rx
        rx = np.arctan2(-R[0,1], R[1,1])
    return rz, ry, rx

class ZeusClientNode(Node):
    def __init__(self):
        super().__init__('zeus_client_node')
        
        self.lock = threading.Lock()

        CAM_INIT = ['j', -86.16, -15.19, -107.19, 0.0, -56.92, 3.84]
        self.block_safety = ['wait_for_block']
        self.block        = ['wait_for_block']
        
        self.block_list = [
            CAM_INIT,
            # self.block_safety,
            # self.block,
            # self.block_safety,
            # CAM_INIT
        ]
        
        self.idx = 0
        
        self.dh_params = CameraDHParameters()
        self.base_to_camera_matrix = None
        self.block_mat = None
        self.joint_coor = None
        self.block_pose = None
        
        self.service_cb_group = ReentrantCallbackGroup()
        self.sub_cb_group = ReentrantCallbackGroup()
        self.timer_cb_group = ReentrantCallbackGroup()

        self.cli = self.create_client(ZeusExecutor, '/zeus_exec')
        self.create_subscription(Float32MultiArray, '/zeus/array/block_pose', self.block_pose_callback, 10, callback_group=self.sub_cb_group)
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
         #self.get_logger().info(f'[ZEUS] joint_coor: {self.joint_coor}')
                self.base_to_camera_matrix = self.cal_base_to_cam(np.deg2rad(list(self.joint_coor)))
                   
    def cal_base_to_cam(self, joint_angles):
        all_dh_params = self.dh_params.get_all_dh_params(joint_angles[:6])
    
        T_BC = fk(all_dh_params)
        return T_BC

    def timer(self):
        pass
        
    def send_next_command(self):
        while self.idx < len(self.block_list):
            cmd = self.block_list[self.idx]
            if cmd[0] == 'wait_for_block':
                self.get_logger().info('[ZEUS] 블록 감지 대기...')
                self.is_waiting_for_block = True
                return
            else:
                break
        
        if self.idx >= len(self.block_list):
            self.get_logger().info('[ZEUS] 초기 위치 이동 완료. 블록 감지 대기 중...')
            self.is_waiting_for_block = True
            self.block_safety, = ['wait_for_block']
            self.block         = ['wait_for_block']
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

    def response_callback(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().warn(f'[ZEUS] 서비스 호출 실패: {e}')
            self.idx += 1
            self.send_next_command()
            return

        if res.success:
            self.get_logger().info(f'[ZEUS] {self.idx+1}번째 명령 성공')
        else:
            self.get_logger().warn(f'[ZEUS] {self.idx+1}번째 명령 실패')

        self.idx += 1
        self.send_next_command()
        
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
        
        T_CO = self.base_to_camera_matrix @ self.block_mat
        
        with self.lock:
            self.block_pose = T_CO
        
        P = T_CO[:3, 3]    
        R = T_CO[:3,:3]
        
        rz, ry, rx = rot_to_euler_zyx(R)
        
        print(np.rad2deg(rz), np.rad2deg(ry), np.rad2deg(rx))
        print(P)
        # with self.lock:
        #     self.block_safety = ['l', P[0], P[1], 350.0, rz, ry, rx]
        #     self.block = ['l', P[0], P[1], 250.0, rz, ry, rx]
            
        #     self.is_waiting_for_block = False
            
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
