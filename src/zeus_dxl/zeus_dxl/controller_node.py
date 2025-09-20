#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import math, ctypes
from dynamixel_sdk import PortHandler, PacketHandler

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
PRESENT_CURRENT_ADDR  = 126  # 모델별 확인 필요

CURR_UNIT_A = 0.00269
VEL_UNIT_RPM = 0.229
RPM_TO_RAD_S = 2.0 * math.pi / 60.0
VEL_UNIT_RAD_S = VEL_UNIT_RPM * RPM_TO_RAD_S
TICK_TO_RAD = 2.0 * math.pi / 4096.0

I_MAX_A = 0.8
Kp = 0.06
Kd = 0.005
CONTROL_DT = 0.01

MIN_ANGLE = -90.0
MAX_ANGLE = -5.0
INITIAL_ANGLE = -70.0

# 충돌 감지
CURRENT_THRESHOLD = 0.05
STACK_LIMIT = 15
COLLISION_ANGLE = -90.0

def a_to_raw_i(a):
    raw = int(round(a / CURR_UNIT_A))
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
        self.current_pub = self.create_publisher(Float32, '/motor_current', 10)
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

    # --- ROS2 Callbacks ---
    def target_callback(self, msg: String):
        cmd = msg.data
        if cmd == 'up':
            ang = -90
        elif cmd == 'down':
            ang = -30
        else:
            self.get_logger().warn('Wrong Command')
            
        angle = max(MIN_ANGLE, min(MAX_ANGLE, ang))
        if not self.collision_active:  # 충돌 모드가 아닐 때만 목표 갱신
            if angle != self.target_angle:
                self.get_logger().info(f" Received target: {angle}°")
                self.target_angle = angle
                self.moving = True

    # --- Control Loop ---
    def control_loop(self):
        pos_raw = self._read4(ADDR_PRESENT_POSITION)
        vel_raw = self._read4(ADDR_PRESENT_VELOCITY)
        current_angle_deg = (pos_raw / 4096.0 * 360.0) - 180.0

        # 전류 읽기 & 스택 업데이트
        curr_raw = self._read2(PRESENT_CURRENT_ADDR)
        current_a = curr_raw * CURR_UNIT_A
        if current_a > CURRENT_THRESHOLD and not self.collision_active:
            self.current_stack += 1
        else:
            self.current_stack = 0

        # 충돌 감지
        if self.current_stack >= STACK_LIMIT and not self.collision_active:
            self.get_logger().warn(f"⚠️ 충돌 감지! 전류 초과 {self.current_stack}회")
            self.target_angle = COLLISION_ANGLE
            self.moving = True
            self.collision_active = True
            self.current_stack = 0

        # PD 제어
        if self.moving:
            pos_err_rad = (angle_to_tick(self.target_angle) - pos_raw) * TICK_TO_RAD
            vel_rad_s = raw_vel_to_rad_s(vel_raw)
            i_cmd = Kp * pos_err_rad - Kd * vel_rad_s

            if self.target_angle > current_angle_deg:
                i_cmd = abs(i_cmd)
            else:
                i_cmd = -abs(i_cmd)

            i_cmd = max(-I_MAX_A, min(I_MAX_A, i_cmd))
            self._write2(ADDR_GOAL_CURRENT, a_to_raw_i(i_cmd))

            # 목표 각도 근접 시
            if abs(current_angle_deg - self.target_angle) < 0.5:
                self.moving = False
                self.hold_target_tick = pos_raw
                self._write2(ADDR_GOAL_CURRENT, 0)

                # 충돌 이동 완료 시 복귀
                if self.collision_active:
                    self.collision_active = False
                    self.get_logger().info("충돌 모드 종료, 정상 제어 복귀")
                    # 목표각도 유지 루프 재활성화
                    self.moving = False  # 여기서 외부 pub이 들어오면 바로 moving=True로 설정됨

        else:
            pos_err_rad = (self.hold_target_tick - pos_raw) * TICK_TO_RAD
            vel_rad_s = raw_vel_to_rad_s(vel_raw)
            i_cmd = Kp * pos_err_rad - Kd * vel_rad_s
            i_cmd = max(-I_MAX_A, min(I_MAX_A, i_cmd))
            self._write2(ADDR_GOAL_CURRENT, a_to_raw_i(i_cmd))

        # 전류 퍼블리시
        msg = Float32()
        msg.data = current_a
        self.current_pub.publish(msg)

    def publish_current(self):
        # 0.1s 주기 전류 퍼블리시
        curr_raw = self._read2(PRESENT_CURRENT_ADDR)
        current_a = curr_raw * CURR_UNIT_A
        msg = Float32()
        msg.data = current_a
        self.current_pub.publish(msg)

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