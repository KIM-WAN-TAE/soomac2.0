#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from zeus_interfaces.srv import ZeusExecutor

import numpy as np
import threading, time

LIN_FRAME = ['l1', 'l2', 'l3', 'l4', 'l5', 'l6', 'l7' ,'l8']

def angle_diff(goal, current):
    diff = goal - current
    for i in range(len(diff)):
        # x,y,z는 패스, rx,ry,rz(혹은 joint angle)만 보정
        if i >= 3:  # rx, ry, rz 인덱스
            d = diff[i]
            d = (d + 180.0) % 360.0 - 180.0
            diff[i] = d
    return diff

def rpy_to_rotation_matrix(roll, pitch, yaw):
    """RPY(degree)를 rotation matrix로 변환"""
    r = np.radians(roll)
    p = np.radians(pitch)
    y = np.radians(yaw)

    # Roll (X축 회전)
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(r), -np.sin(r)],
                   [0, np.sin(r), np.cos(r)]])

    # Pitch (Y축 회전)
    Ry = np.array([[np.cos(p), 0, np.sin(p)],
                   [0, 1, 0],
                   [-np.sin(p), 0, np.cos(p)]])

    # Yaw (Z축 회전)
    Rz = np.array([[np.cos(y), -np.sin(y), 0],
                   [np.sin(y), np.cos(y), 0],
                   [0, 0, 1]])

    # ZYX 순서로 회전 적용
    R = Rz @ Ry @ Rx
    return R

def pose_to_transform(pose):
    """6D pose [x,y,z,r,p,y]를 4x4 transformation matrix로 변환"""
    x, y, z, roll, pitch, yaw = pose

    R = rpy_to_rotation_matrix(roll, pitch, yaw)

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]

    return T

def transform_to_pose(T):
    """4x4 transformation matrix를 6D pose [x,y,z,r,p,y]로 변환"""
    x, y, z = T[:3, 3]

    R = T[:3, :3]

    # Rotation matrix에서 RPY 추출 (ZYX 순서)
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2))

    if np.abs(np.cos(pitch)) > 1e-6:
        roll = np.arctan2(R[2, 1] / np.cos(pitch), R[2, 2] / np.cos(pitch))
        yaw = np.arctan2(R[1, 0] / np.cos(pitch), R[0, 0] / np.cos(pitch))
    else:
        # Gimbal lock 경우
        roll = 0
        yaw = np.arctan2(-R[0, 1], R[1, 1])

    # Radian to degree
    roll = np.degrees(roll)
    pitch = np.degrees(pitch)
    yaw = np.degrees(yaw)

    return np.array([x, y, z, roll, pitch, yaw])

def dh_transform(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    
    T = np.array([[ ct, -st*ca,  st*sa, a*ct],
                  [ st,  ct*ca, -ct*sa, a*st],
                  [  0,      sa,     ca,    d],
                  [  0,       0,      0,    1]])
    return T

def command_string(frame, arr):
    if len(arr) != 6:
        raise ValueError("입력 리스트의 길이는 6이어야 합니다.")
    
    if frame == 'j' or frame == 'J':
        values_str = ",".join([f"{v:.4f}" for v in arr])
        cmd = f"move_j_abs+{values_str}"
        return cmd
    
    elif frame == 'l1' or frame == 'L1':
        values_str = ",".join([f"{v:.4f}" for v in arr])
        cmd = f"move_l_abs+{values_str},1.0"
        return cmd
    
    elif frame == 'l2' or frame == 'L2':
        values_str = ",".join([f"{v:.4f}" for v in arr])
        cmd = f"move_l_abs+{values_str},2.0"
        return cmd
    
    elif frame == 'l3' or frame == 'L3':
        values_str = ",".join([f"{v:.4f}" for v in arr])
        cmd = f"move_l_abs+{values_str},3.0"
        return cmd
    
    elif frame == 'l4' or frame == 'L4':
        values_str = ",".join([f"{v:.4f}" for v in arr])
        cmd = f"move_l_abs+{values_str},4.0"
        return cmd
    
    elif frame == 'l5' or frame == 'L5':
        values_str = ",".join([f"{v:.4f}" for v in arr])
        cmd = f"move_l_abs+{values_str},5.0"
        return cmd
    
    elif frame == 'l6' or frame == 'L6':
        values_str = ",".join([f"{v:.4f}" for v in arr])
        cmd = f"move_l_abs+{values_str},6.0"
        return cmd
    
    elif frame == 'l7' or frame == 'L7':
        values_str = ",".join([f"{v:.4f}" for v in arr])
        cmd = f"move_l_abs+{values_str},7.0"
        return cmd
    
    elif frame == 'l7' or frame == 'L7':
        values_str = ",".join([f"{v:.4f}" for v in arr])
        cmd = f"move_l_abs+{values_str},8.0"
        return cmd
    
    elif frame == 't' or frame == 'T':
        values_str = ",".join([f"{v:.4f}" for v in arr])
        cmd = f"tool_move+{values_str}"
        return cmd
    
    else:
        raise ValueError("Wrong Frame")

class ZeusServerNode(Node):
    def __init__(self):
        super().__init__('zeus_server_node')
        self.get_logger().info('[ZEUS] Server Node On!')
        
        self.service_cb_group = ReentrantCallbackGroup()
        self.sub_cb_group = ReentrantCallbackGroup()
        
        self.lock = threading.Lock()
        
        self.srv = self.create_service(ZeusExecutor, '/zeus_exec', self.handler, callback_group=self.service_cb_group)
        self.create_subscription(Float32MultiArray, '/zeus/array/xy_state', self.xy_state_callback, 10, callback_group=self.sub_cb_group)
        self.create_subscription(Float32MultiArray, '/zeus/array/joint_state', self.joint_state_callback, 10, callback_group=self.sub_cb_group)
        self.command_pub = self.create_publisher(String, '/zeus/string/binary_command', 10)
        
        self.xy_coor = np.zeros(6, dtype=np.float32)
        self.joint_coor = np.zeros(6, dtype=np.float32)   
        
        self.tol_ang = 1.0
        self.tol_pos = 2.0
        self.tol_tol = 5.0
        
    def xy_state_callback(self, msg):
        with self.lock:
            if len(msg.data) == 6:
                self.xy_coor = msg.data
         # self.get_logger().info(f'[ZEUS] xyz_coor: {self.xy_coor}')

    def joint_state_callback(self, msg):
        with self.lock:
            if len(msg.data) == 6:
                self.joint_coor = msg.data
        # self.get_logger().info(f'[ZEUS] joint_coor: {self.joint_coor}')

            
    def handler(self, req, res):
        res.success = False
        try:
            frame = req.frame
            goal_coor = np.array(req.coordinate)

            com_str = command_string(frame, goal_coor)

            com_msg = String()
            com_msg.data = com_str
            self.command_pub.publish(com_msg)

            target = None
            initial_pose = None
            if frame.lower() == 't':
                with self.lock:
                    initial_pose = np.array(self.xy_coor)  # 초기 pose 저장
                    offset = goal_coor  # [dx,dy,dz,dr,dp,dy]

                    # 초기 EE pose를 transformation matrix로 변환
                    T_initial = pose_to_transform(initial_pose)

                    # Offset을 transformation matrix로 변환
                    T_offset = pose_to_transform(offset)

                    # EE frame 기준으로 offset 적용
                    T_target = T_initial @ T_offset

                    # 최종 목표 pose 계산 (base frame 기준) - 루프 내내 고정
                    target = transform_to_pose(T_target)

                self.get_logger().info(f'[Tool Move] Initial: {initial_pose}, Offset: {offset}, Target: {target}')

            # 안정화 모니터링 변수들
            stable_start_time = None
            last_error = None
            stable_threshold = 0.5  # 1.5초 동안 error 변화 없으면 완료
            error_tolerance = 0.01  # error 변화량 허용 범위

            while True:
                with self.lock:
                    if frame.lower() in LIN_FRAME:
                        current = np.array(self.xy_coor)
                        error = np.linalg.norm(goal_coor - current)
                        self.get_logger().info(f'Linear error : {error}')

                        if int(error) <= self.tol_pos:
                            res.success = True
                            break

                    elif frame.lower() == 'j':
                        current = np.array(self.joint_coor)
                        error = np.linalg.norm(goal_coor - current)
                        self.get_logger().info(f'Joint error : {error}')

                        if int(error) <= self.tol_ang:
                            res.success = True
                            break

                    elif frame.lower() == 't':
                        current_pose = np.array(self.xy_coor)  # [x,y,z,r,p,y]

                        # Position error (xyz)
                        pos_error = np.linalg.norm(target[:3] - current_pose[:3])

                        # Rotation error (rpy) with angle wrapping
                        rot_diff = angle_diff(target[3:], current_pose[3:])
                        rot_error = np.linalg.norm(rot_diff)

                        # Total error
                        error = pos_error + rot_error * 0.1  # rotation에 가중치 적용

                        self.get_logger().info(f'target : {target}, current: {current_pose}, pos_error: {pos_error:.2f}, rot_error: {rot_error:.2f}, total: {error:.2f}')

                        # 정상 완료 조건
                        if int(error) <= self.tol_tol:
                            res.success = True
                            break

                        # 안정화 모니터링: error가 1.5초 이상 변화 없으면 완료
                        current_time = time.time()
                        if last_error is not None:
                            error_change = abs(error - last_error)

                            if error_change <= error_tolerance:
                                # 오차가 안정적으로 유지됨
                                if stable_start_time is None:
                                    stable_start_time = current_time
                                elif current_time - stable_start_time >= stable_threshold:
                                    self.get_logger().info(f'[ZEUS] Tool move completed: error stable for {stable_threshold}s (final error: {error:.2f}mm)')
                                    res.success = True
                                    break
                            else:
                                # 오차가 변화함 - 안정화 시간 리셋
                                stable_start_time = None

                        last_error = error

                time.sleep(0.05)

        except Exception as e:
            self.get_logger().error(f'[ZEUS SERVER] Send Coordinate Fail : {e}')
            
        return res
        
def main(args=None):
    rclpy.init(args=args)
    node = ZeusServerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()