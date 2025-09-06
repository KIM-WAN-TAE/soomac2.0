#!/usr/bin/env python3
# finger_test/finger_test/dxl_pd_current_node.py
import math
import time
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Int32MultiArray

from dynamixel_sdk import (
    PortHandler, PacketHandler,
    GroupBulkRead, GroupSyncWrite,
    COMM_SUCCESS
)

class DxlPdCurrentNode(Node):
    # --- Dynamixel control table (X-series 공통) ---
    ADDR_BAUD_RATE          = 8
    ADDR_OPERATING_MODE     = 11
    ADDR_TORQUE_ENABLE      = 64
    ADDR_CURRENT_LIMIT      = 38
    ADDR_GOAL_CURRENT       = 102
    LEN_GOAL_CURRENT        = 2
    ADDR_PRESENT_CURRENT    = 126
    LEN_PRESENT_CURRENT     = 2
    ADDR_PRESENT_POSITION   = 132
    LEN_PRESENT_POSITION    = 4

    TORQUE_DISABLE = 0
    TORQUE_ENABLE  = 1
    MODE_CURRENT   = 0
    PROTOCOL_VER   = 2.0

    def __init__(self):
        super().__init__('dxl_pd_current_node')

        # --- Parameters (공통 게인/주기/한계) ---
        self.declare_parameter('device_name', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 3_000_000)
        self.declare_parameter('dxl_ids', [1, 2])
        self.declare_parameter('kp', 0.10)            # [raw_current / tick]
        self.declare_parameter('kd', 0.002)           # [raw_current / tick_per_sec]
        self.declare_parameter('current_limit', 200)  # Goal Current 제한(raw, ±)
        self.declare_parameter('control_period', 0.005)  # 200 Hz
        self.declare_parameter('deadband', 5)         # tick deadband

        self.device_name: str = self.get_parameter('device_name').get_parameter_value().string_value
        self.baudrate: int = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.ids: List[int] = list(self.get_parameter('dxl_ids').get_parameter_value().integer_array_value)
        self.kp: float = self.get_parameter('kp').get_parameter_value().double_value
        self.kd: float = self.get_parameter('kd').get_parameter_value().double_value
        self.current_limit: int = self.get_parameter('current_limit').get_parameter_value().integer_value
        self.dt: float = self.get_parameter('control_period').get_parameter_value().double_value
        self.deadband: int = self.get_parameter('deadband').get_parameter_value().integer_value

        # --- SDK 초기화 ---
        self.port = PortHandler(self.device_name)
        self.packet = PacketHandler(self.PROTOCOL_VER)

        if not self.port.openPort():
            raise RuntimeError(f'Cannot open port: {self.device_name}')
        if not self.port.setBaudRate(self.baudrate):
            raise RuntimeError(f'Cannot set baudrate: {self.baudrate}')

        # 안전을 위해 토크 OFF 후 설정
        for i in self.ids:
            self._write1(i, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            self._write1(i, self.ADDR_OPERATING_MODE, self.MODE_CURRENT)  # Current mode
            # 전류 제한(raw) 설정 - 모터 스펙 내에서 조정
            self._write2(i, self.ADDR_CURRENT_LIMIT, abs(self.current_limit))
            self._write1(i, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)

        # --- 통신 그룹 객체 ---
        self.bulk = GroupBulkRead(self.port, self.packet)
        for i in self.ids:
            # Present Position
            self._bulk_add(i, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
            # Present Current
            self._bulk_add(i, self.ADDR_PRESENT_CURRENT, self.LEN_PRESENT_CURRENT)

        self.sync_current = GroupSyncWrite(self.port, self.packet, self.ADDR_GOAL_CURRENT, self.LEN_GOAL_CURRENT)

        # --- 상태 변수 ---
        n = len(self.ids)
        # 초기 Read 후 현재 위치를 목표로 설정(점프 방지)
        pres_pos = self._read_present()[0]
        self.target_pos = pres_pos[:] if pres_pos else [0]*n
        self.prev_err = [0]*n

        # --- ROS Pub/Sub ---
        self.pub_curr = self.create_publisher(Int16MultiArray, 'dxl/present_current', 10)
        self.pub_pos  = self.create_publisher(Int32MultiArray, 'dxl/present_position', 10)
        self.sub_target = self.create_subscription(Int32MultiArray, 'dxl/target_positions', self.cb_target, 10)

        # --- 제어 루프 ---
        self.timer = self.create_timer(self.dt, self.control_step)
        self.get_logger().info(f'Initialized: ids={self.ids}, port={self.device_name}, baud={self.baudrate}bps, dt={self.dt}s')

    # ----------------- ROS Callbacks -----------------
    def cb_target(self, msg: Int32MultiArray):
        if len(msg.data) != len(self.ids):
            self.get_logger().warn(f'/dxl/target_positions length({len(msg.data)}) != num_ids({len(self.ids)})')
            return
        self.target_pos = list(msg.data)

    def control_step(self):
        pres_pos, pres_cur = self._read_present()
        if pres_pos is None or pres_cur is None:
            return  # 통신 실패 시 스킵

        # 모니터링 퍼블리시
        self._publish_states(pres_pos, pres_cur)

        # PD 제어 (Goal Current raw 계산)
        packets = []
        for idx, dxl_id in enumerate(self.ids):
            err = int(self.target_pos[idx]) - int(pres_pos[idx])
            if abs(err) < self.deadband:
                err = 0
            derr = (err - self.prev_err[idx]) / self.dt
            u = self.kp * err + self.kd * derr  # raw current 명령

            # 포화 및 형변환
            u_raw = int(max(-self.current_limit, min(self.current_limit, u)))
            self.prev_err[idx] = err
            packets.append((dxl_id, u_raw))

        # Sync Write (Goal Current)
        for dxl_id, u_raw in packets:
            param = self._int16_to_2bytes(u_raw)
            if not self.sync_current.addParam(dxl_id, param):
                self.get_logger().error(f'Failed addParam SyncWrite for ID {dxl_id}')
        dxl_comm_result = self.sync_current.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'SyncWrite txPacket failed: {self.packet.getTxRxResult(dxl_comm_result)}')
        self.sync_current.clearParam()

    # ----------------- Low-level helpers -----------------
    def _write1(self, dxl_id, addr, val):
        dxl_comm_result, dxl_error = self.packet.write1ByteTxRx(self.port, dxl_id, addr, int(val))
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'ID {dxl_id} write1 err: {self.packet.getTxRxResult(dxl_comm_result)}')
        elif dxl_error != 0:
            self.get_logger().error(f'ID {dxl_id} write1 status err: {self.packet.getRxPacketError(dxl_error)}')

    def _write2(self, dxl_id, addr, val):
        val = int(val) & 0xFFFF
        dxl_comm_result, dxl_error = self.packet.write2ByteTxRx(self.port, dxl_id, addr, val)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'ID {dxl_id} write2 err: {self.packet.getTxRxResult(dxl_comm_result)}')
        elif dxl_error != 0:
            self.get_logger().error(f'ID {dxl_id} write2 status err: {self.packet.getRxPacketError(dxl_error)}')

    def _bulk_add(self, dxl_id, addr, length):
        if not self.bulk.addParam(dxl_id, addr, length):
            self.get_logger().error(f'BulkRead addParam failed: id={dxl_id}, addr={addr}, len={length}')

    def _read_present(self):
        # BulkRead: pos(4B) + current(2B)
        dxl_comm_result = self.bulk.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'BulkRead txRxPacket failed: {self.packet.getTxRxResult(dxl_comm_result)}')
            return None, None

        pres_pos = []
        pres_cur = []
        for dxl_id in self.ids:
            ok_pos = self.bulk.isAvailable(dxl_id, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
            ok_cur = self.bulk.isAvailable(dxl_id, self.ADDR_PRESENT_CURRENT,  self.LEN_PRESENT_CURRENT)
            if not (ok_pos and ok_cur):
                self.get_logger().warn(f'BulkRead not available for ID {dxl_id}')
                return None, None
            p = self.bulk.getData(dxl_id, self.ADDR_PRESENT_POSITION, self.LEN_PRESENT_POSITION)
            c_u16 = self.bulk.getData(dxl_id, self.ADDR_PRESENT_CURRENT,  self.LEN_PRESENT_CURRENT)
            c = self._to_signed(c_u16, 16)  # signed int16 변환
            pres_pos.append(int(p))
            pres_cur.append(int(c))
        return pres_pos, pres_cur

    @staticmethod
    def _to_signed(val, bits):
        if val >= (1 << (bits - 1)):
            val -= (1 << bits)
        return val

    @staticmethod
    def _int16_to_2bytes(v):
        v16 = v & 0xFFFF  # two's complement 처리
        return [v16 & 0xFF, (v16 >> 8) & 0xFF]

    def shutdown(self):
        # 안전 종료: 토크 OFF, 포트 닫기
        try:
            for i in self.ids:
                self._write1(i, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        finally:
            try:
                self.port.closePort()
            except Exception:
                pass
        self.get_logger().info('DXL torque disabled & port closed.')

    def _publish_states(self, pres_pos, pres_cur):
        msg_cur = Int16MultiArray()
        msg_cur.data = pres_cur
        self.pub_curr.publish(msg_cur)

        msg_pos = Int32MultiArray()
        msg_pos.data = pres_pos
        self.pub_pos.publish(msg_pos)

def main():
    rclpy.init()
    node = DxlPdCurrentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
