#!/usr/bin/env python3
import time
import math
from dynamixel_sdk import PortHandler, PacketHandler

# ————— 설정 —————
DEVICENAME           = '/dev/ttyUSB0'
BAUDRATE             = 3_000_000
PROTOCOL_VERSION     = 2.0

MOTOR_ID             = 1                # XH540-V270-R
ADDR_OPERATING_MODE  = 11
ADDR_TORQUE_ENABLE   = 64
ADDR_GOAL_CURRENT    = 102
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_POSITION= 132

# 토크 상수 Kt = 9.2 Nm / 2.4 A ≈ 3.83 Nm/A (예시: 4.71 사용)
Kt = 4.71

# — 모드 분기 임계 토크 설정 — 
THRESHOLD_TORQUE_NM  = 0.10   # Nm
THRESHOLD_CURRENT_RAW= int((THRESHOLD_TORQUE_NM / Kt) / 2.69e-3)
CCW_THRESHOLD_FACTOR = 3.0    # 반시계(CCW) 방향 민감도 배수
THR_CW   = THRESHOLD_CURRENT_RAW
THR_CCW  = int(THRESHOLD_CURRENT_RAW * CCW_THRESHOLD_FACTOR)

# — PD 제어 파라미터 (홀드 모드) — 
Kp = 0.05    # 위치 오차 비례 이득
Kd = 0.015   # 속도 오차 미분 이득

# 전류 제한 (raw 단위)
I_LIMIT = 2400

# 제어 주기
DT = 0.005  # 200 Hz

# ————— 초기화 —————
port = PortHandler(DEVICENAME)
pkt  = PacketHandler(PROTOCOL_VERSION)
port.openPort()
port.setBaudRate(BAUDRATE)

# 전류 제어 모드 활성화
pkt.write1ByteTxRx(port, MOTOR_ID, ADDR_OPERATING_MODE, 0)
pkt.write1ByteTxRx(port, MOTOR_ID, ADDR_TORQUE_ENABLE, 1)

# 초기 위치 읽어 홀드 포지션으로 설정
resp, _, _     = pkt.read4ByteTxRx(port, MOTOR_ID, ADDR_PRESENT_POSITION)
hold_position  = float(resp)
prev_pos       = hold_position
last_goal_current = 0

try:
    while True:
        # — 1) 측정된 전류(raw) 읽기 —
        raw_present, _, _ = pkt.read2ByteTxRx(port, MOTOR_ID, ADDR_PRESENT_CURRENT)
        if raw_present > 0x7FFF:
            raw_present -= 0x10000

        # — 2) 외력 전류 분리 —
        external_raw = raw_present - last_goal_current

        # — 3) 방향별 임계값 선택 —
        if external_raw > 0:
            threshold = THR_CCW
        else:
            threshold = THR_CW

        # 디버그 출력
        print(f"Measured: {raw_present:+5d}, External: {external_raw:+5d}, Thresh: {threshold}")

        # — 4) 모드 분기 —
        if abs(external_raw) <= threshold:
            # Mode1: Hold position via PD control
            # 현재 위치 읽기
            pos_raw, _, _  = pkt.read4ByteTxRx(port, MOTOR_ID, ADDR_PRESENT_POSITION)
            pos = float(pos_raw)
            now = time.time()
            vel = (pos - prev_pos) / DT

            # PD 토크 계산
            error    = hold_position - pos
            torque_pd= Kp * error - Kd * vel
            # 토크→전류(raw)로 변환
            goal_current = int((torque_pd / Kt) / 2.69e-3)

            # 홀드 모드에서는 hold_position 고정
            prev_pos = pos

        else:
            # Mode2: Compliant, 외력만큼 회전
            # 명령 전류 = –external_raw / 2 (감속 비율)
            goal_current = int(-external_raw / 2)
            # 외력이 임계 넘어설 때 새로운 홀드 포지션 갱신
            pos_raw, _, _ = pkt.read4ByteTxRx(port, MOTOR_ID, ADDR_PRESENT_POSITION)
            hold_position = float(pos_raw)
            prev_pos      = hold_position

        # — 5) 명령 전류(raw) 제한 및 전송 —
        goal_current = max(min(goal_current, I_LIMIT), -I_LIMIT)
        pkt.write2ByteTxRx(port, MOTOR_ID, ADDR_GOAL_CURRENT, goal_current)
        last_goal_current = goal_current

        time.sleep(DT)

except KeyboardInterrupt:
    # 안전 정지
    pkt.write1ByteTxRx(port, MOTOR_ID, ADDR_TORQUE_ENABLE, 0)
    port.closePort()
    print("제어 종료")
