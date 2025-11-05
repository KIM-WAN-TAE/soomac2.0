#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, Int32MultiArray
import threading
import numpy as np
import signal
import sys
import time
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite, GroupBulkRead

DEVICENAME            = '/dev/ttyUSB1'
BAUDRATE              = 3000000
PROTOCOL_VERSION      = 2.0

JOINT_IDS             = [1, 2, 3, 4]
WRIST_ID              = 5

ADDR_TORQUE_ENABLE    = 64
ADDR_OPERATING_MODE   = 11
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_VELOCITY    = 104
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION  = 132
ADDR_PRESENT_VELOCITY  = 128
ADDR_PRESENT_CURRENT   = 126

LEN_GOAL_POSITION      = 4
LEN_PRESENT_POSITION   = 4
LEN_PRESENT_VELOCITY   = 4
LEN_PRESENT_CURRENT    = 2

RATE = 50

def deg_to_pulse(degree):
    if isinstance(degree, (list, tuple, np.ndarray)):
        deg_arr = np.asarray(degree, dtype=np.float64)
        p = np.rint(deg_arr * (4096.0 / 360.0) + 2048).astype(int)
        return np.clip(p, 0, 4095).tolist()
    else:
        p = int(np.round(degree * (4096.0 / 360.0) + 2048))
        return max(0, min(4095, p))


def pulse_to_deg(pulse):
    if isinstance(pulse, (list, tuple, np.ndarray)):
        pulse_arr = np.asarray(pulse, dtype=int)
        pulse_arr = np.clip(pulse_arr, 0, 4095)
        return ((pulse_arr - 2048) * (360.0 / 4096.0)).tolist()
    else:
        pulse = max(0, min(4095, int(pulse)))
        return (pulse - 2048) * (360.0 / 4096.0)


def rad_to_pulse(rad):
    if isinstance(rad, (list, tuple, np.ndarray)):
        rad_arr = np.asarray(rad, dtype=np.float64)
        p = np.rint(rad_arr * (4096.0 / (2*np.pi)) + 2048).astype(int)
        return np.clip(p, 0, 4095).tolist()
    else:
        p = int(np.round(rad * (4096.0 / (2*np.pi)) + 2048))
        return max(0, min(4095, p))


def pulse_to_rad(pulse):
    if isinstance(pulse, (list, tuple, np.ndarray)):
        pulse_arr = np.asarray(pulse, dtype=int)
        pulse_arr = np.clip(pulse_arr, 0, 4095)
        return ((pulse_arr - 2048) * (2*np.pi / 4096.0)).tolist()
    else:
        pulse = max(0, min(4095, int(pulse)))
        return (pulse - 2048) * (2*np.pi / 4096.0)
    

class MotorConnectNode(Node):
    def __init__(self):
        super().__init__('motor_connet')

        self.lock = threading.Lock()

        self.porthandler = PortHandler(DEVICENAME)
        self.packethandler = PacketHandler(PROTOCOL_VERSION)

        if not self.porthandler.openPort():
            self.get_logger().error(f"[AIOT] 포트를 열 수 없습니다: {DEVICENAME}")
            return

        if not self.porthandler.setBaudRate(BAUDRATE):
            self.get_logger().error(f"[AIOT] 보드레이트 설정 실패: {BAUDRATE}")
            return

        self.groupSyncWrite = GroupSyncWrite(self.porthandler, self.packethandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

        self.groupBulkRead = GroupBulkRead(self.porthandler, self.packethandler)

        self.motor_init()
        self.setup_bulk_read()
        self.get_logger().info('[AIOT] Motor Ready!')
        
        # 목표 각도
        self.create_subscription(Float32MultiArray, 
                                 '/aiot/array/target_motor_deg', 
                                 self.target_pulse_callback, 10)
        
        # 5번 각도
        self.create_subscription(Float32,
                                 '/aiot/float/target_wrist_deg',
                                 self.target_wrist_callback, 10)
        
        # 모터 속도
        self.create_subscription(Float32MultiArray,
                                 '/aiot/array/motor_speed',
                                 self.target_speed_callback, 10)
        
        self.motor_pulse_pub = self.create_publisher(Int32MultiArray, '/aiot/array/present_motor_pulse', 10)
        self.motor_current_pub = self.create_publisher(Float32MultiArray, '/aiot/array/present_current', 10)             
        
        timer_period = 1/RATE
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        
    def motor_init(self):
        for motor_id in JOINT_IDS + [WRIST_ID]:
            _, dxl_error = self.packethandler.write1ByteTxRx(
                self.porthandler, motor_id,
                ADDR_TORQUE_ENABLE, 0
            )
            if dxl_error != 0:
                self.get_logger().warn(f"[AIOT] ID {motor_id}: 토크 비활성화 오류 ({dxl_error})")

            _, dxl_error = self.packethandler.write1ByteTxRx(
                self.porthandler, motor_id,
                ADDR_OPERATING_MODE, 3
            )
            if dxl_error != 0:
                self.get_logger().warn(f"[AIOT] ID {motor_id}: 운영 모드 설정 오류 ({dxl_error})")

            _, dxl_error = self.packethandler.write1ByteTxRx(
                self.porthandler, motor_id,
                ADDR_TORQUE_ENABLE, 1
            )
            if dxl_error != 0:
                self.get_logger().warn(f"[AIOT] ID {motor_id}: 토크 활성화 오류 ({dxl_error})")

            _, dxl_error = self.packethandler.write4ByteTxRx(
                self.porthandler, motor_id,
                ADDR_PROFILE_VELOCITY, 0
            )
            if dxl_error != 0:
                self.get_logger().warn(f"[AIOT] ID {motor_id}: 초기 속도 설정 오류 ({dxl_error})")

    def setup_bulk_read(self):
        """BulkRead를 위한 파라미터 추가 - 위치만 읽기"""
        for motor_id in JOINT_IDS + [WRIST_ID]:
            # 위치만 읽기 (BulkRead는 각 모터당 하나의 연속된 메모리 영역만 읽을 수 있음)
            dxl_addparam_result = self.groupBulkRead.addParam(motor_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            if not dxl_addparam_result:
                self.get_logger().error(f"[AIOT] ID {motor_id}: BulkRead 파라미터 추가 실패 (위치)")

    def target_pulse_callback(self, msg : Float32MultiArray):
        if len(msg.data) < len(JOINT_IDS):
            self.get_logger().warn('[AIOT] Wrong DATA length')
            return

        goal_positions = []
        for idx, motor_id in enumerate(JOINT_IDS):
            goal_pulse = deg_to_pulse(msg.data[idx])
            goal_positions.append(goal_pulse)

            param_goal_position = [
                goal_pulse & 0xFF,
                (goal_pulse >> 8) & 0xFF,
                (goal_pulse >> 16) & 0xFF,
                (goal_pulse >> 24) & 0xFF
            ]
            dxl_addparam_result = self.groupSyncWrite.addParam(motor_id, param_goal_position)
            if not dxl_addparam_result:
                self.get_logger().warn(f"[AIOT] ID {motor_id}: SyncWrite 파라미터 추가 실패")

        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != 0:
            self.get_logger().warn(f"[AIOT] SyncWrite 통신 오류: {dxl_comm_result}")
        else:
            self.get_logger().info(f'[AIOT] Goal Joint Positions: {goal_positions}')

        self.groupSyncWrite.clearParam()

        time.sleep(0.05)
        for motor_id in JOINT_IDS:
            try:
                _, dxl_error = self.packethandler.write4ByteTxRx(
                    self.porthandler, motor_id, ADDR_GOAL_VELOCITY, 0
                )
                if dxl_error != 0:
                    self.get_logger().warn(f"[AIOT] Motor {motor_id}: Goal Velocity 0 설정 오류 ({dxl_error})")
                else:
                    self.get_logger().info(f"[AIOT] Motor {motor_id}: Goal Velocity 0 설정 완료")
            except Exception as e:
                self.get_logger().warn(f"[AIOT] Motor {motor_id}: Goal Velocity 설정 통신 오류 {e}")


    def target_wrist_callback(self, msg : Float32):
        if msg.data is False or msg.data is None:
            self.get_logger().warn('[AIOT] Wrong DATA (wrist)')
            return
        
        goal_pulse = deg_to_pulse(msg.data)
        _, dxl_error = self.packethandler.write4ByteTxRx(
                self.porthandler, WRIST_ID,
                ADDR_GOAL_POSITION, goal_pulse)
        if dxl_error != 0:
            self.get_logger().warn(f"[AIOT] WRIST : 목표 위치 쓰기 오류 ({dxl_error})")
        else:
            self.get_logger().info(f'[AIOT] Goal Wrist Pulse : {goal_pulse}')
        
    def target_speed_callback(self, msg : Float32MultiArray):
        if len(msg.data) < len(JOINT_IDS):
            self.get_logger().warn("[AIOT] Speed 데이터 길이 부족")
            return
        
        for idx, motor_id in enumerate(JOINT_IDS):
            vel = int(msg.data[idx])
            _, dxl_error = self.packethandler.write4ByteTxRx(
                self.porthandler, motor_id,
                ADDR_PROFILE_VELOCITY, vel
            )
            if dxl_error != 0:
                self.get_logger().warn(f"[AIOT] ID {motor_id}: 프로파일 속도 쓰기 오류 ({dxl_error})")
            else:
                self.get_logger().info(f'[AIOT] Goal Velocity : {vel:.2f}')
                
    def timer_callback(self):
        # BulkRead 실행 (위치만 읽기)
        dxl_comm_result = self.groupBulkRead.txRxPacket()
        if dxl_comm_result != 0:
            self.get_logger().warn(f"[AIOT] BulkRead 통신 오류: {dxl_comm_result}")
            return

        positions = []
        currents = []

        for motor_id in JOINT_IDS + [WRIST_ID]:
            # 위치 데이터 읽기 (BulkRead)
            dxl_getdata_result = self.groupBulkRead.isAvailable(motor_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            if dxl_getdata_result:
                present_pos = self.groupBulkRead.getData(motor_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                positions.append(int(present_pos))
            else:
                self.get_logger().warn(f"[AIOT] ID {motor_id}: 위치 데이터 읽기 실패")
                positions.append(0)

            # 전류 데이터 읽기 (개별 읽기)
            present_curr, _, dxl_error = self.packethandler.read2ByteTxRx(
                self.porthandler, motor_id, ADDR_PRESENT_CURRENT
            )
            if dxl_error != 0:
                self.get_logger().warn(f"[AIOT] ID {motor_id}: 전류 읽기 오류 ({dxl_error})")
                currents.append(0.0)
            else:
                if present_curr > 32767:
                    present_curr -= 65536
                currents.append(float(present_curr))

        pulse_msg = Int32MultiArray()
        pulse_msg.data = positions
        self.motor_pulse_pub.publish(pulse_msg)

        current_msg = Float32MultiArray()
        current_msg.data = currents
        self.motor_current_pub.publish(current_msg)
        
    def destroy_node(self):
        self.get_logger().info("[AIOT] 안전한 종료 시작...")

        for motor_id in JOINT_IDS + [WRIST_ID]:
            try:
                self.packethandler.write4ByteTxRx(
                    self.porthandler, motor_id, ADDR_GOAL_VELOCITY, 0
                )
                time.sleep(0.1)
                self.packethandler.write1ByteTxRx(
                    self.porthandler, motor_id, ADDR_TORQUE_ENABLE, 0
                )
                self.get_logger().info(f"[AIOT] Motor {motor_id} 안전 종료 완료")
            except Exception as e:
                self.get_logger().warn(f"[AIOT] Motor {motor_id} 종료 오류: {e}")

        try:
            self.porthandler.closePort()
            self.get_logger().info("[AIOT] 포트 닫기 완료")
        except:
            self.get_logger().warn("[AIOT] 포트 닫기 실패")

        super().destroy_node()

node = None

def signal_handler(sig, frame):
    global node
    print("\n[AIOT] Ctrl+C 감지 - 모터 토크 비활성화 중...")
    if node and hasattr(node, 'porthandler') and node.porthandler.is_open:
        for motor_id in JOINT_IDS + [WRIST_ID]:
            try:
                node.packethandler.write4ByteTxRx(
                    node.porthandler, motor_id, ADDR_GOAL_VELOCITY, 0
                )
                time.sleep(0.1)
                _, error = node.packethandler.write1ByteTxRx(
                    node.porthandler, motor_id, ADDR_TORQUE_ENABLE, 0
                )
                if error == 0:
                    print(f"[AIOT] Motor {motor_id} 토크 비활성화 완료")
                else:
                    print(f"[AIOT] Motor {motor_id} 토크 비활성화 오류: {error}")
            except Exception as e:
                print(f"[AIOT] Motor {motor_id} 통신 오류: {e}")

        try:
            node.porthandler.closePort()
            print("[AIOT] 포트 닫기 완료")
        except:
            print("[AIOT] 포트 닫기 실패")

    print("[AIOT] 안전 종료 완료")
    sys.exit(0)

def main(args=None):
    global node
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    rclpy.init(args=args)
    node = MotorConnectNode()

    if not hasattr(node, 'porthandler') or not node.porthandler.is_open:
        print("Motor connection failed - 종료")
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()