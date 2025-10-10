#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from dynamixel_sdk import PortHandler, PacketHandler
import time

DEVICENAME            = '/dev/ttyUSB0'
BAUDRATE              = 3000000
PROTOCOL_VERSION      = 2.0

ID = 6

ADDR_TORQUE_ENABLE    = 64
ADDR_OPERATING_MODE   = 11
ADDR_GOAL_CURRENT     = 102
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION  = 132
ADDR_PRESENT_CURRENT   = 126

CURRENT_MODE  = 0
POSITION_MODE = 3

RATE = 10
CURR_UNIT_A = 0.00269

RELEASE_POSITION = 2150
GRIPPER_INIT_POSITION = 2800
GRIP_CURRENT     = 18

class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')
        
        self.porthandler = PortHandler(DEVICENAME)
        self.packethandler = PacketHandler(PROTOCOL_VERSION)
        
        if not self.porthandler.openPort():
            self.get_logger().error(f"[AIOT] 포트를 열 수 없습니다: {DEVICENAME}")
            return

        if not self.porthandler.setBaudRate(BAUDRATE):
            self.get_logger().error(f"[AIOT] 보드레이트 설정 실패: {BAUDRATE}")
            return
        
        self.motor_init()
        self.create_subscription(String, '/zeus/string/gripper_command', self.gripper_callback, 10)
        
        self.status_timer = self.create_timer(1/RATE, self.timer_callback)
        self.current_pub = self.create_publisher(Float32, '/zeus/float/gripper_present_current', 10)
        self.active_pub = self.create_publisher(String, '/zeus/string/gripper_done', 10)
        
    def motor_init(self):
        _, dxl_error = self.packethandler.write1ByteTxRx(
            self.porthandler, ID,
            ADDR_TORQUE_ENABLE, 0
        )
        if dxl_error != 0:
            self.get_logger().warn(f"[AIOT] ID {ID}: 토크 비활성화 오류 ({dxl_error})")

        _, dxl_error = self.packethandler.write1ByteTxRx(
            self.porthandler, ID,
            ADDR_OPERATING_MODE, POSITION_MODE
        )
        if dxl_error != 0:
            self.get_logger().warn(f"[AIOT] ID {ID}: 운영 모드 설정 오류 ({dxl_error})")

        _, dxl_error = self.packethandler.write1ByteTxRx(
            self.porthandler, ID,
            ADDR_TORQUE_ENABLE, 1
        )
        if dxl_error != 0:
            self.get_logger().warn(f"[AIOT] ID {ID}: 토크 활성화 오류 ({dxl_error})")
        
        # 초기에 Gripper 개방 상태로 유지    
        _, dxl_error = self.packethandler.write4ByteTxRx(
            self.porthandler, ID, ADDR_GOAL_POSITION, RELEASE_POSITION
        )
        
        if dxl_error != 0:
            self.get_logger().warn(f"[AIOT] ID {ID}: Gripper 개방 오류 ({dxl_error})")
            
    def make_position_mode(self):
        _, dxl_error = self.packethandler.write1ByteTxRx(
            self.porthandler, ID,
            ADDR_TORQUE_ENABLE, 0
        )
        if dxl_error != 0:
            self.get_logger().warn(f"[AIOT] ID {ID}: 토크 비활성화 오류 ({dxl_error})")

        _, dxl_error = self.packethandler.write1ByteTxRx(
            self.porthandler, ID,
            ADDR_OPERATING_MODE, POSITION_MODE
        )
        if dxl_error != 0:
            self.get_logger().warn(f"[AIOT] ID {ID}: 운영 모드 설정 오류 ({dxl_error})")

        _, dxl_error = self.packethandler.write1ByteTxRx(
            self.porthandler, ID,
            ADDR_TORQUE_ENABLE, 1
        )
        if dxl_error != 0:
            self.get_logger().warn(f"[AIOT] ID {ID}: 토크 활성화 오류 ({dxl_error})")
        
    def make_current_mode(self):
        _, dxl_error = self.packethandler.write1ByteTxRx(
            self.porthandler, ID,
            ADDR_TORQUE_ENABLE, 0
        )
        if dxl_error != 0:
            self.get_logger().warn(f"[AIOT] ID {ID}: 토크 비활성화 오류 ({dxl_error})")

        _, dxl_error = self.packethandler.write1ByteTxRx(
            self.porthandler, ID,
            ADDR_OPERATING_MODE, CURRENT_MODE
        )
        if dxl_error != 0:
            self.get_logger().warn(f"[AIOT] ID {ID}: 운영 모드 설정 오류 ({dxl_error})")

        _, dxl_error = self.packethandler.write1ByteTxRx(
            self.porthandler, ID,
            ADDR_TORQUE_ENABLE, 1
        )
        if dxl_error != 0:
            self.get_logger().warn(f"[AIOT] ID {ID}: 토크 활성화 오류 ({dxl_error})")
        
    def gripper_callback(self, msg : String):
        cmd = msg.data
        print(f'{msg.data}')
        
        grip_msg = String()
        grip_msg.data = 'done'
        
        current_time = time.time()
        last_time = current_time
        
        if cmd == 'open':
            self.make_position_mode()
            
            while True:
                    current_time = time.time()
                    if current_time - last_time > 0.1:
                        break
            
            _, dxl_error = self.packethandler.write4ByteTxRx(
            self.porthandler, ID, ADDR_GOAL_POSITION, RELEASE_POSITION
            )
        
            if dxl_error != 0:
                self.get_logger().warn(f"[AIOT] ID {ID}: Gripper 개방 오류 ({dxl_error})")
            else:
                while True:
                    current_time = time.time()
                    if current_time - last_time > 0.8:
                        break
                    
                self.active_pub.publish(grip_msg)
                
        elif cmd == 'close':
            while True:
                    current_time = time.time()
                    if current_time - last_time > 0.1:
                        break
            self.make_current_mode()
            
            _, dxl_error = self.packethandler.write2ByteTxRx(
                self.porthandler, ID, ADDR_GOAL_CURRENT, GRIP_CURRENT & 0xFFFF
            )
            
            while True:
                    current_time = time.time()
                    if current_time - last_time > 1.0:
                        break
            
            self.active_pub.publish(grip_msg)
            
        elif cmd == 'gripinit':
            self.make_position_mode()
            
            while True:
                    current_time = time.time()
                    if current_time - last_time > 0.1:
                        break
            
            _, dxl_error = self.packethandler.write4ByteTxRx(
            self.porthandler, ID, ADDR_GOAL_POSITION, GRIPPER_INIT_POSITION
            )
            
            if dxl_error != 0:
                self.get_logger().warn(f"[AIOT] ID {ID}: Gripper 개방 오류 ({dxl_error})")
            else:
                while True:
                    current_time = time.time()
                    if current_time - last_time > 0.8:
                        break
                    
                self.active_pub.publish(grip_msg)
                
        else:
            self.get_logger().warn('[AIOT] Wrong Command!')
            
    def timer_callback(self):
        raw_cur, _, dxl_error = self.packethandler.read2ByteTxRx(
            self.porthandler, ID, ADDR_PRESENT_CURRENT)
        
        if raw_cur > 32767:
            raw_cur = raw_cur - 65536
        
        present_current = raw_cur * CURR_UNIT_A * 1000
        
        # print(f"[AIOT] Current: {present_current:.1f} mA")
        
        cur_msg = Float32()
        cur_msg.data = present_current
        self.current_pub.publish(cur_msg)
            
def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nShutting down Dongsoo Gripper Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()