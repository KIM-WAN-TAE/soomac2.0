#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray # 사용자 정의 메시지 대신 임시 사용
from dynamixel_sdk import *
import time

#================================================================================
# 다이나믹셀 상수 정의
#================================================================================
#----- 제어 테이블 주소 (XH/XM 시리즈 공통) -----
ADDR_OPERATING_MODE      = 11
ADDR_TORQUE_ENABLE       = 64
ADDR_GOAL_CURRENT        = 102
ADDR_GOAL_POSITION       = 116
ADDR_PRESENT_POSITION    = 132

#----- 데이터 길이 -----
LEN_GOAL_CURRENT         = 2
LEN_GOAL_POSITION        = 4
LEN_PRESENT_POSITION     = 4

#----- 동작 모드 -----
CURRENT_BASED_POSITION_CONTROL_MODE = 5

#----- 프로토콜 버전 -----
PROTOCOL_VERSION         = 2.0

#================================================================================
# 모터 설정 (사용자 수정 영역)
#================================================================================
# [중요] 이 리스트만 수정하면 포트가 다른 모터도 쉽게 추가/관리할 수 있습니다.
MOTOR_CONFIG = [
    {'id': 1, 'model': 'XH540', 'port': '/dev/ttyUSB0', 'baudrate': 1000000},
    {'id': 2, 'model': 'XH540', 'port': '/dev/ttyUSB0', 'baudrate': 1000000},
    {'id': 3, 'model': 'XH430', 'port': '/dev/ttyUSB0', 'baudrate': 1000000},
    {'id': 4, 'model': 'XH430', 'port': '/dev/ttyUSB0', 'baudrate': 1000000},
    # 예시: 만약 5번 모터가 다른 포트에 연결된다면 아래와 같이 추가
    # {'id': 5, 'model': 'XM540', 'port': '/dev/ttyUSB1', 'baudrate': 1000000},
]

# 단위 변환 상수
CURRENT_TO_RAW_VALUE_CONSTANT = 2.69  # 1(raw) = 2.69mA (모터 모델별로 확인 필요)
POSITION_TO_RADIAN_CONSTANT = (3.14159265359 * 2) / 4096 # 1회전 = 4096 pulse


class MotorConnectNode(Node):
    """
    다이나믹셀 모터와 직접 통신하며, 하드웨어 드라이버 역할을 수행하는 노드.
    - 여러 USB 포트에 연결된 모터를 동시에 제어할 수 있도록 설계되었습니다.
    """
    def __init__(self):
        super().__init__('motor_connect_node')
        self.get_logger().info("MotorConnectNode를 시작합니다...")

        # 1. 초기 설정 (Initialization & Setup)
        self.motors = MOTOR_CONFIG
        self.dxl_ids = [m['id'] for m in self.motors]

        # 포트별 핸들러 관리를 위한 딕셔너리
        self.port_handlers = {}
        self.packet_handlers = {}
        self.group_bulk_reads = {}
        self.group_bulk_writes = {}

        self._initialize_ports_and_handlers()
        self._setup_motors()

        # 2. ROS 2 통신 설정
        self.joint_state_publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.command_subscriber_ = self.create_subscription(
            Float64MultiArray,
            'joint_commands',
            self._command_callback,
            10)

        # 3. 메인 기능
        timer_period = 0.01  # 100Hz
        self.timer = self.create_timer(timer_period, self._timer_callback)

        self.get_logger().info("노드 초기화 완료. 제어를 시작합니다.")

    def _initialize_ports_and_handlers(self):
        """설정된 모든 포트를 열고, 포트별 핸들러를 생성 및 초기화합니다."""
        # 설정에서 유니크한 포트 목록 가져오기
        unique_ports = set(m['port'] for m in self.motors)

        for port_name in unique_ports:
            # 포트 핸들러 생성 및 포트 열기
            portHandler = PortHandler(port_name)
            try:
                portHandler.openPort()
                self.get_logger().info(f"시리얼 포트 '{port_name}' 열기 성공.")
            except Exception as e:
                self.get_logger().fatal(f"포트 '{port_name}' 열기 실패: {e}")
                raise e
            
            # 보드레이트 설정 (해당 포트의 첫 번째 모터 설정 따름)
            baudrate = next(m['baudrate'] for m in self.motors if m['port'] == port_name)
            try:
                portHandler.setBaudRate(baudrate)
                self.get_logger().info(f"포트 '{port_name}' 보드레이트 {baudrate} 설정 성공.")
            except Exception as e:
                self.get_logger().fatal(f"포트 '{port_name}' 보드레이트 설정 실패: {e}")
                raise e

            # 포트별 핸들러 저장
            self.port_handlers[port_name] = portHandler
            self.packet_handlers[port_name] = PacketHandler(PROTOCOL_VERSION)
            self.group_bulk_reads[port_name] = GroupBulkRead(portHandler, self.packet_handlers[port_name])
            self.group_bulk_writes[port_name] = GroupBulkWrite(portHandler, self.packet_handlers[port_name])

    def _setup_motors(self):
        """모든 모터의 동작 모드 설정, 토크 활성화, BulkRead 등록을 수행합니다."""
        self.get_logger().info("모터 설정 시작...")
        for motor in self.motors:
            port_name = motor['port']
            dxl_id = motor['id']
            packetHandler = self.packet_handlers[port_name]
            portHandler = self.port_handlers[port_name]
            groupBulkRead = self.group_bulk_reads[port_name]

            # 토크 비활성화 -> 모드 변경 -> 토크 활성화
            packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
            packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, CURRENT_BASED_POSITION_CONTROL_MODE)
            time.sleep(0.1)
            packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)

            # 포트별 BulkRead 파라미터 등록
            if not groupBulkRead.addParam(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                self.get_logger().error(f"ID {dxl_id} ({port_name}): BulkRead 파라미터 등록 실패")
        
        self.get_logger().info("모든 모터 설정 및 토크 활성화 완료.")

    def _command_callback(self, msg):
        """명령 수신 시, 포트별로 BulkWrite를 통해 모터에 전달합니다."""
        num_joints = len(self.motors)
        if len(msg.data) != num_joints * 2:
            self.get_logger().warn(f"잘못된 명령 길이 수신: {len(msg.data)}. 기대값: {num_joints*2}")
            return

        positions = msg.data[:num_joints]
        currents = msg.data[num_joints:]

        # 포트별 BulkWrite 파라미터 초기화
        for port_name in self.group_bulk_writes:
            self.group_bulk_writes[port_name].clearParam()

        # 각 모터에 맞는 포트의 BulkWrite 핸들러에 파라미터 추가
        for i, motor in enumerate(self.motors):
            port_name = motor['port']
            dxl_id = motor['id']
            groupBulkWrite = self.group_bulk_writes[port_name]

            pos_raw = int(positions[i] / POSITION_TO_RADIAN_CONSTANT)
            pos_data = [DXL_LOBYTE(DXL_LOWORD(pos_raw)), DXL_HIBYTE(DXL_LOWORD(pos_raw)),
                        DXL_LOBYTE(DXL_HIWORD(pos_raw)), DXL_HIBYTE(DXL_HIWORD(pos_raw))]

            cur_raw = int(currents[i] / CURRENT_TO_RAW_VALUE_CONSTANT)
            cur_data = [DXL_LOBYTE(cur_raw), DXL_HIBYTE(cur_raw)]

            groupBulkWrite.addParam(dxl_id, ADDR_GOAL_POSITION, LEN_GOAL_POSITION, pos_data)
            groupBulkWrite.addParam(dxl_id, ADDR_GOAL_CURRENT, LEN_GOAL_CURRENT, cur_data)

        # 모든 포트에 대해 BulkWrite 명령 전송
        for port_name in self.group_bulk_writes:
            dxl_comm_result = self.group_bulk_writes[port_name].txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f"포트 '{port_name}' 전송 실패: {self.packet_handlers[port_name].getTxRxResult(dxl_comm_result)}")

    def _timer_callback(self):
        """주기적으로 모든 포트의 모터 상태를 읽고 하나의 토픽으로 발행합니다."""
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [f'joint_{m["id"]}' for m in self.motors]
        
        all_positions = []

        # 모든 포트에 대해 BulkRead 실행
        for port_name in self.group_bulk_reads:
            dxl_comm_result = self.group_bulk_reads[port_name].txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f"포트 '{port_name}' 수신 실패: {self.packet_handlers[port_name].getTxRxResult(dxl_comm_result)}")

        # 모든 모터의 데이터 취합
        for motor in self.motors:
            port_name = motor['port']
            dxl_id = motor['id']
            groupBulkRead = self.group_bulk_reads[port_name]

            pos_rad = 0.0 # 기본값
            if groupBulkRead.isAvailable(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                pos_raw = groupBulkRead.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                pos_rad = pos_raw * POSITION_TO_RADIAN_CONSTANT
            
            all_positions.append(pos_rad)
        
        joint_state_msg.position = all_positions
        self.joint_state_publisher_.publish(joint_state_msg)

    def destroy_node(self):
        """노드 종료 시 모든 모터의 토크를 비활성화하고 모든 포트를 닫습니다."""
        self.get_logger().info("노드를 종료합니다. 모터 토크를 비활성화합니다.")
        for motor in self.motors:
            port_name = motor['port']
            dxl_id = motor['id']
            packetHandler = self.packet_handlers[port_name]
            portHandler = self.port_handlers[port_name]
            packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
        
        for port_name in self.port_handlers:
            self.port_handlers[port_name].closePort()
            self.get_logger().info(f"포트 '{port_name}' 닫기 완료.")
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    motor_connect_node = MotorConnectNode()
    try:
        rclpy.spin(motor_connect_node)
    except KeyboardInterrupt:
        motor_connect_node.get_logger().info('키보드 인터럽트로 노드 종료')
    finally:
        motor_connect_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
