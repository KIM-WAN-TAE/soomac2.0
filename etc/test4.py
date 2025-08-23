#!/usr/bin/env python3
import time
import math
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# ————— 설정 —————
DEVICENAME           = '/dev/ttyUSB0'
BAUDRATE             = 3_000_000
PROTOCOL_VERSION     = 2.0

MOTOR_ID             = 1                 # XH540-V270-R
ADDR_OPERATING_MODE  = 11
ADDR_TORQUE_ENABLE   = 64
ADDR_GOAL_CURRENT    = 102
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_POSITION= 132

# — PD 제어 파라미터 (홀드 모드) — 
KP                   = 0.05              # 비례 이득 (raw per count)
KD                   = 0.005             # 미분 이득
CONTROL_PERIOD       = 0.005             # 초 (200 Hz)

# — Threshold 임계 토크 설정 — 
Kt                   = 4.71              # Nm per Arms (예시값)
THRESHOLD_TORQUE_NM  = 0.15              # Nm
# raw 전류 단위로 변환 (1 raw ≒2.69 mA)
THRESHOLD_CURRENT_RAW= int((THRESHOLD_TORQUE_NM / Kt) / 2.69e-3)

# CCW(반시계) 문턱 배수
CCW_THRESHOLD_FACTOR = 5.0               
# CW(시계) 문턱은 50%로 낮게
THR_CW               = int(THRESHOLD_CURRENT_RAW * 0.5)
THR_CCW              = int(THRESHOLD_CURRENT_RAW * CCW_THRESHOLD_FACTOR)

# 반시계 속도 지연 배율
CCW_SPEED_FACTOR     = 0.001               # 10% 속도로 느리게 반응

# 전류(raw) 제한
I_LIMIT              = 2400              # ±2400 raw

# ————— 초기화 —————
port = PortHandler(DEVICENAME)
pkt  = PacketHandler(PROTOCOL_VERSION)
if not port.openPort() or not port.setBaudRate(BAUDRATE):
    raise RuntimeError("포트 열기 또는 통신 속도 설정 실패")

# 전류 제어 모드 설정 및 토크 활성화
pkt.write1ByteTxRx(port, MOTOR_ID, ADDR_OPERATING_MODE, 0)
pkt.write1ByteTxRx(port, MOTOR_ID, ADDR_TORQUE_ENABLE, 1)

# — 초기 상태 변수 — 
# 첫 hold_position은 현재 위치로 설정
resp, _, _         = pkt.read4ByteTxRx(port, MOTOR_ID, ADDR_PRESENT_POSITION)
hold_position      = float(resp)
previous_error     = 0.0
last_time          = time.time()
last_goal_current  = 0

try:
    while True:
        # 1) 측정 전류(raw) 읽기
        raw_present, _, _ = pkt.read2ByteTxRx(port, MOTOR_ID, ADDR_PRESENT_CURRENT)
        if raw_present > 0x7FFF:
            raw_present -= 0x10000

        # 2) 외력 전류 분리
        external_raw = raw_present - last_goal_current

        # 3) 방향별 Threshold 선택
        if external_raw > 0:
            threshold = THR_CCW   # CCW 진입 문턱 (타이트)
        else:
            threshold = THR_CW    # CW 진입 문턱

        # 타이밍 계산
        now = time.time()
        dt  = now - last_time if now > last_time else CONTROL_PERIOD
        last_time = now

        if abs(external_raw) <= threshold:
            # ─ Mode1: PD로 Hold Position ─
            pos_raw, _, _ = pkt.read4ByteTxRx(port, MOTOR_ID, ADDR_PRESENT_POSITION)
            pos = float(pos_raw)

            error   = hold_position - pos
            d_error = (error - previous_error) / dt
            previous_error = error

            # PD → raw 전류
            p_term = KP * error
            d_term = KD * d_error
            goal_current = int(p_term + d_term)

        else:
            # ─ Mode2: Compliant(외력 반응) ─
            if external_raw > 0:
                # CCW: 속도를 느리게
                goal_current = int(-external_raw * CCW_SPEED_FACTOR)
            else:
                # CW: 기존 감속 비율
                goal_current = int(-external_raw / 2)

            # 새로운 홀드 포지션 갱신
            pos_raw, _, _ = pkt.read4ByteTxRx(port, MOTOR_ID, ADDR_PRESENT_POSITION)
            hold_position = float(pos_raw)
            previous_error = 0.0

        # 디버그 출력
        print(f"Meas:{raw_present:+5d}  Ext:{external_raw:+5d}  Th:{threshold}  Goal:{goal_current}")

        # 4) 목표 전류(raw) 제한 및 전송
        goal_current = max(min(goal_current, I_LIMIT), -I_LIMIT)
        pkt.write2ByteTxRx(port, MOTOR_ID, ADDR_GOAL_CURRENT, goal_current)
        last_goal_current = goal_current

        time.sleep(CONTROL_PERIOD)

except KeyboardInterrupt:
    # 안전 정지
    pkt.write1ByteTxRx(port, MOTOR_ID, ADDR_TORQUE_ENABLE, 0)
    port.closePort()
    print("제어 종료")
