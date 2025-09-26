#/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from dynamixel_sdk import *
import numpy as np
import threading
import os

DEVICE = '/dev/ttyUSB0'
BAUDRATE = 3000000
PROTOCOL_VER = 2.0
GRIPPER_ID = 6

ADDR_TORQUE_ENABLE    = 64
ADDR_OPERATING_MODE   = 11
ADDR_GOAL_CURRENT     = 102
ADDR_PRESENT_CURRENT  = 126
ADDR_PRESENT_POSITION = 132

CURRENT_CONTROL_MODE = 0

MAX_CURRENT_LIMIT = 80

class GripperNode(Node):
    def __init__(self):
        super().__init__('zeus_gripper_node')
        
        self.sub_callback_group = ReentrantCallbackGroup()
        self.read_write_callback_group = ReentrantCallbackGroup()

        self.data_lock = threading.Lock()  # 데이터 보호용
        self.port_lock = threading.Lock()  # 포트 접근 동기화용
        
        self.portHandler = PortHandler(DEVICE)
        self.packetHandler = PacketHandler(PROTOCOL_VER)
        
        self.initialize()
        
        self.pre_position, dxl_comm_result, _ = self.packetHandler.read4ByteTxRx(
                self.portHandler, GRIPPER_ID, ADDR_PRESENT_POSITION)
        
        self.goal_position = self.pre_position
        self.previous_error = 0

        self.kp = 0.0105
        self.kd = 0.0
        
        self.create_subscription(String, '/zeus/gripper/grip', self.goal_callback, 10, callback_group=self.sub_callback_group)
        self.create_timer(0.01, self.read_write_callback, callback_group=self.read_write_callback_group)
        
        self.get_logger().info('Zeus Gripper Node initialized')
        
    def initialize(self):  
        if not self.portHandler.openPort():
            self.get_logger().error("Failed to open the port")
            return

        if not self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().error("Failed to change the baudrate")
            return
        
        dxl_comm_result, _ = self.packetHandler.write1ByteTxRx(
            self.portHandler, GRIPPER_ID, ADDR_TORQUE_ENABLE, 0)

        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"Failed to disable torque: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return False

        dxl_comm_result, _ = self.packetHandler.write1ByteTxRx(
            self.portHandler, GRIPPER_ID, ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE)

        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"Failed to set current control mode: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return False

        dxl_comm_result, _ = self.packetHandler.write1ByteTxRx(
            self.portHandler, GRIPPER_ID, ADDR_TORQUE_ENABLE, 1)

        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"Failed to enable torque: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return False

        self.get_logger().info("Motor initialized successfully in current control mode")
        
    def goal_callback(self, msg : String):
        if msg.data == 'block_grip':
            goal_position = 3100
        
        elif msg.data == 'block_leave':
            goal_position = 3900
        
        else:
            return
        
        with self.data_lock:
            self.goal_position = goal_position

        # self.get_logger().info(f'[ZEUS] GOAL Position: {self.goal_position}')
        
    def read_sensor(self):
        """포트 락을 사용하여 안전하게 센서값 읽기"""
        with self.port_lock:
            dxl_present_position, dxl_comm_result, _ = self.packetHandler.read4ByteTxRx(
                    self.portHandler, GRIPPER_ID, ADDR_PRESENT_POSITION)

            if dxl_comm_result != COMM_SUCCESS:
                return None, None

            dxl_present_current, dxl_comm_result, _ = self.packetHandler.read2ByteTxRx(
                    self.portHandler, GRIPPER_ID, ADDR_PRESENT_CURRENT)

            if dxl_comm_result != COMM_SUCCESS:
                return dxl_present_position, None

            return dxl_present_position, dxl_present_current
        
    def write_goal(self, current_mA):
        """포트 락을 사용하여 안전하게 목표 전류 설정"""
        limited_current = max(-MAX_CURRENT_LIMIT, min(MAX_CURRENT_LIMIT, current_mA))
        current_units = int(limited_current * 2.69)

        with self.port_lock:
            dxl_comm_result, _ = self.packetHandler.write2ByteTxRx(
                self.portHandler, GRIPPER_ID, ADDR_GOAL_CURRENT, current_units)

            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f"Failed to write goal current: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                return False

        return True
        
    def pd_control(self, goal_pos, current_pos):
        error = goal_pos - current_pos
        p_term = self.kp * error
        d_term = self.kd * (error - self.previous_error)
        output_current = p_term + d_term
        self.previous_error = error
        # self.get_logger().info(f'\n error : {error}')
        return output_current
        
    def read_write_callback(self):
        # 센서값 읽기 (포트 락 사용)
        pre_position, pre_current = self.read_sensor()

        # 데이터 유효성 확인 및 저장
        if pre_position is not None and pre_current is not None:
            with self.data_lock:
                self.pre_position = pre_position
                self.pre_current = pre_current

        # 목표값 복사 (데이터 락 사용)
        with self.data_lock:
            goal_pose = self.goal_position

        # 현재 위치가 유효한 경우에만 제어 수행
        if hasattr(self, 'pre_position') and self.pre_position is not None:
            goal_current = self.pd_control(goal_pose, self.pre_position)

            # 목표 전류 설정 (포트 락 사용)
            success = self.write_goal(goal_current)
            if not success:
                self.get_logger().warn(f'[ZEUS] Gripper Error')

        self.get_logger().info(f'\n Present Pos : {pre_position} \n Present Cur : {pre_current / 2.69}')
        
def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()