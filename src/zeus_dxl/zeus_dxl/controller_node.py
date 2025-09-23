#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import math, ctypes
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncRead

# ------------------- 설정 -------------------
PORT_NAME = "/dev/ttyUSB0"
BAUD_RATE = 1000000
DXL_ID = 1
PROTOCOL_VERSION = 2.0

ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_CURRENT     = 102
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_VELOCITY = 128
ADDR_OPERATING_MODE   = 11
PRESENT_CURRENT_ADDR  = 126

CURR_UNIT_MA = 2.69  # mA 단위 (기존 0.00269 A = 2.69 mA)
VEL_UNIT_RPM = 0.229
RPM_TO_RAD_S = 2.0 * math.pi / 60.0
VEL_UNIT_RAD_S = VEL_UNIT_RPM * RPM_TO_RAD_S
TICK_TO_RAD = 2.0 * math.pi / 4096.0

I_MAX_MA = 800.0  # 최대 전류 800mA (기존 0.8A)
Kp = 0.06
Kd = 0.006
CONTROL_DT = 0.01

MIN_ANGLE = -90.0
MAX_ANGLE = -5.0
INITIAL_ANGLE = -70.0

# 충돌 감지
CURRENT_THRESHOLD_MA = 60.0  # 60mA (기존 0.06A)
STACK_LIMIT = 2
COLLISION_ANGLE = -90.0

def ma_to_raw_i(ma):
    """mA를 raw 전류값으로 변환"""
    raw = int(round(ma / CURR_UNIT_MA))
    return max(-32768, min(32767, raw))

def raw_vel_to_rad_s(raw):
    return ctypes.c_int32(raw).value * VEL_UNIT_RAD_S

def angle_to_tick(angle_deg):
    return int((angle_deg + 180.0) / 360.0 * 4096.0)

class DynamixelController(Node):
    def __init__(self):
        super().__init__('dynamixel_pd_node')

        # 포트 초기화
        self.port = PortHandler(PORT_NAME)
        self.ph = PacketHandler(PROTOCOL_VERSION)
        if not self.port.openPort():
            self.get_logger().error("포트 열기 실패")
            raise SystemExit
        if not self.port.setBaudRate(BAUD_RATE):
            self.get_logger().error("보레이트 설정 실패")
            raise SystemExit

        # Sync Read 초기화 (Present Current 읽기용)
        self.sync_read_present_current = GroupSyncRead(self.port, self.ph, PRESENT_CURRENT_ADDR, 2)
        if not self.sync_read_present_current.addParam(DXL_ID):
            self.get_logger().error("Present Current Sync Read 파라미터 추가 실패")
            raise SystemExit

        # Sync Read 초기화 (Goal Current 읽기용)
        self.sync_read_goal_current = GroupSyncRead(self.port, self.ph, ADDR_GOAL_CURRENT, 2)
        if not self.sync_read_goal_current.addParam(DXL_ID):
            self.get_logger().error("Goal Current Sync Read 파라미터 추가 실패")
            raise SystemExit

        # 토크 및 모드 초기화
        self._write1(ADDR_TORQUE_ENABLE, 0)
        self._write1(ADDR_OPERATING_MODE, 0)
        self._write1(ADDR_TORQUE_ENABLE, 1)

        pos_raw = self._read4(ADDR_PRESENT_POSITION)
        self.hold_target_tick = pos_raw
        self.target_angle = INITIAL_ANGLE
        self.moving = False

        # 충돌 감지 변수
        self.current_stack = 0
        self.collision_active = False

        self.get_logger().info(f"초기 위치 유지, 목표 각도: {INITIAL_ANGLE}°")

        # ROS2
        self.sub = self.create_subscription(String, '/zeus/string/target_angle',
                                            self.target_callback, 10)
        self.timer = self.create_timer(CONTROL_DT, self.control_loop)
        self.current_pub = self.create_publisher(Float32, '/motor_current_ma', 10)
        self.external_current_pub = self.create_publisher(Float32, '/external_current_ma', 10)
        self.create_timer(0.1, self.publish_current)

    # --- Low-level ---
    def _write1(self, addr, val):
        r, e = self.ph.write1ByteTxRx(self.port, DXL_ID, addr, val)
        if r != 0 or e != 0:
            self.get_logger().warn(f"write1 err: addr={addr} res={r} err={e}")

    def _write2(self, addr, val):
        raw = val & 0xFFFF
        r, e = self.ph.write2ByteTxRx(self.port, DXL_ID, addr, raw)
        if r != 0 or e != 0:
            self.get_logger().warn(f"write2 err: addr={addr} res={r} err={e}")

    def _read4(self, addr):
        v, r, e = self.ph.read4ByteTxRx(self.port, DXL_ID, addr)
        if r != 0 or e != 0:
            self.get_logger().warn(f"read4 err: addr={addr} res={r} err={e}")
        return ctypes.c_int32(v).value

    def _read2(self, addr):
        v, r, e = self.ph.read2ByteTxRx(self.port, DXL_ID, addr)
        if r != 0 or e != 0:
            self.get_logger().warn(f"read2 err: addr={addr} res={r} err={e}")
        return ctypes.c_int16(v).value

    # --- 최적화된 연속 읽기로 전류 읽기 (mA 단위) ---
    def read_currents_fast(self):
        """Present Current와 Goal Current를 최소 지연으로 연속 읽기 (mA 단위 반환)"""
        # Goal Current 먼저 읽기 (이미 쓴 값이므로 변화가 적음)
        goal_curr_raw = self._read2(ADDR_GOAL_CURRENT)
        # Present Current 바로 읽기
        present_curr_raw = self._read2(PRESENT_CURRENT_ADDR)
        
        # 전류 값 변환 (mA 단위)
        present_ma = present_curr_raw * CURR_UNIT_MA
        goal_ma = goal_curr_raw * CURR_UNIT_MA
        
        # 외력 전류 = Present Current - Goal Current (mA 단위)
        external_current_ma = present_ma - goal_ma
        
        return external_current_ma, present_ma, goal_ma

    # --- Sync Read로 전류 읽기 (대안) ---
    def read_currents_sync(self):
        """Sync Read를 사용하여 전류 읽기 - 필요시 사용 (mA 단위 반환)"""
        try:
            # Present Current 읽기
            dxl_comm_result = self.sync_read_present_current.txRxPacket()
            if dxl_comm_result != 0:
                self.get_logger().warn(f"Present Current Sync Read 실패: {dxl_comm_result}")
                return self.read_currents_fast()
            
            if not self.sync_read_present_current.isAvailable(DXL_ID, PRESENT_CURRENT_ADDR, 2):
                self.get_logger().warn("Present Current 데이터 없음")
                return self.read_currents_fast()
            
            present_curr_raw = self.sync_read_present_current.getData(DXL_ID, PRESENT_CURRENT_ADDR, 2)
            
            # Goal Current 읽기
            dxl_comm_result = self.sync_read_goal_current.txRxPacket()
            if dxl_comm_result != 0:
                self.get_logger().warn(f"Goal Current Sync Read 실패: {dxl_comm_result}")
                goal_curr_raw = self._read2(ADDR_GOAL_CURRENT)
            else:
                if self.sync_read_goal_current.isAvailable(DXL_ID, ADDR_GOAL_CURRENT, 2):
                    goal_curr_raw = self.sync_read_goal_current.getData(DXL_ID, ADDR_GOAL_CURRENT, 2)
                else:
                    goal_curr_raw = self._read2(ADDR_GOAL_CURRENT)
            
            # int16으로 변환
            present_curr_raw = ctypes.c_int16(present_curr_raw).value
            goal_curr_raw = ctypes.c_int16(goal_curr_raw).value
            
            # 전류 값 변환 (mA 단위)
            present_ma = present_curr_raw * CURR_UNIT_MA
            goal_ma = goal_curr_raw * CURR_UNIT_MA
            
            # 외력 전류 계산 (mA 단위)
            external_current_ma = present_ma - goal_ma
            
            return external_current_ma, present_ma, goal_ma
        
        except Exception as e:
            self.get_logger().error(f"Sync Read 에러: {e}")
            return self.read_currents_fast()

    # --- ROS2 Callbacks ---
    def target_callback(self, msg: String):
        cmd = msg.data
        if cmd == 'up':
            ang = -90
        elif cmd == 'down':
            ang = -30
        else:
            self.get_logger().warn('Wrong Command')
            return
            
        angle = max(MIN_ANGLE, min(MAX_ANGLE, ang))
        if not self.collision_active:  # 충돌 모드가 아닐 때만 목표 갱신
            if angle != self.target_angle:
                self.get_logger().info(f" Received target: {angle}°")
                self.target_angle = angle
                self.moving = True

    # --- Control Loop ---
    def control_loop(self):
        # 위치와 속도 읽기
        pos_raw = self._read4(ADDR_PRESENT_POSITION)
        vel_raw = self._read4(ADDR_PRESENT_VELOCITY)
        current_angle_deg = (pos_raw / 4096.0 * 360.0) - 180.0

        # 전류 빠르게 연속 읽기 (외력 전류 계산, mA 단위)
        external_current_ma, present_ma, goal_ma = self.read_currents_fast()
        
        # 충돌 감지용 스택 업데이트 (외력 전류 기준, mA 단위)
        if abs(external_current_ma) > CURRENT_THRESHOLD_MA and not self.collision_active:
            self.current_stack += 1
        else:
            self.current_stack = 0

        # 충돌 감지
        if self.current_stack >= STACK_LIMIT and not self.collision_active:
            self.get_logger().warn(f"⚠️ 충돌 감지! 외력 전류 초과 {self.current_stack}회 (외력: {external_current_ma:6.1f}mA)")
            self.target_angle = COLLISION_ANGLE
            self.moving = True
            self.collision_active = True
            self.current_stack = 0

        # PD 제어
        if self.moving:
            pos_err_rad = (angle_to_tick(self.target_angle) - pos_raw) * TICK_TO_RAD
            vel_rad_s = raw_vel_to_rad_s(vel_raw)
            
            # PD 제어 계산 (mA 단위로 변경)
            i_cmd_ma = (Kp * pos_err_rad - Kd * vel_rad_s) * 1000.0  # A를 mA로 변환
            
            if self.target_angle > current_angle_deg:
                i_cmd_ma = abs(i_cmd_ma)
            else:
                i_cmd_ma = -abs(i_cmd_ma)

            i_cmd_ma = max(-I_MAX_MA, min(I_MAX_MA, i_cmd_ma))
            self._write2(ADDR_GOAL_CURRENT, ma_to_raw_i(i_cmd_ma))

            # 목표 각도 근접 시
            if abs(current_angle_deg - self.target_angle) < 0.5:
                self.moving = False
                self.hold_target_tick = pos_raw
                self._write2(ADDR_GOAL_CURRENT, 0)

                # 충돌 이동 완료 시 복귀
                if self.collision_active:
                    self.collision_active = False
                    self.get_logger().info("충돌 모드 종료, 정상 제어 복귀")

        else:
            pos_err_rad = (self.hold_target_tick - pos_raw) * TICK_TO_RAD
            vel_rad_s = raw_vel_to_rad_s(vel_raw)
            
            # PD 제어 계산 (mA 단위)
            i_cmd_ma = (Kp * pos_err_rad - Kd * vel_rad_s) * 1000.0  # A를 mA로 변환
            i_cmd_ma = max(-I_MAX_MA, min(I_MAX_MA, i_cmd_ma))
            self._write2(ADDR_GOAL_CURRENT, ma_to_raw_i(i_cmd_ma))

        # 전류 퍼블리시 (mA 단위)
        msg = Float32()
        msg.data = present_ma  # Present Current 퍼블리시 (mA)
        self.current_pub.publish(msg)
        
        # 외력 전류 퍼블리시 (mA 단위)
        ext_msg = Float32()
        ext_msg.data = external_current_ma  # 외력 전류 퍼블리시 (mA)
        self.external_current_pub.publish(ext_msg)

        # 디버깅용 로그 (mA 단위)
        self.get_logger().info(f"Present: {present_ma:6.1f}mA, Goal: {goal_ma:6.1f}mA, External: {external_current_ma:6.1f}mA")

    def publish_current(self):
        # 0.1s 주기 전류 퍼블리시 (mA 단위)
        external_current_ma, present_ma, goal_ma = self.read_currents_fast()
        
        msg = Float32()
        msg.data = present_ma  # mA 단위
        self.current_pub.publish(msg)
        
        ext_msg = Float32()
        ext_msg.data = external_current_ma  # mA 단위
        self.external_current_pub.publish(ext_msg)
        
        # 디버깅용 로그 (필요시 활성화)
        # self.get_logger().info(f"[0.1s] Present: {present_ma:6.1f}mA, Goal: {goal_ma:6.1f}mA, External: {external_current_ma:6.1f}mA")

    def shutdown(self):
        self._write2(ADDR_GOAL_CURRENT, 0)
        self._write1(ADDR_TORQUE_ENABLE, 0)
        self.port.closePort()
        self.get_logger().info("모터 종료")

def main(args=None):
    rclpy.init(args=args)
    node = DynamixelController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()