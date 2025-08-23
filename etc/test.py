import os
import time
from dynamixel_sdk import *

# --- 주요 설정 값 ---
# 사용자가 이 부분을 수정하여 제어를 튜닝할 수 있습니다.
# -----------------------------------------------------

# PD 제어 게인
KP = 0.05  # 비례 게인. 목표에 도달하게 하는 주된 힘.
KD = 0.005 # 미분 게인. 오버슛과 진동을 줄여 안정성을 확보.

# 목표 위치 (0 ~ 4095 사이의 값)
# 2048은 대략 중앙 위치입니다.
TARGET_POSITION = 3500

# 안전을 위한 최대 허용 전류 값 (XH430-W210 기준, Stall Current의 약 50%)
# 단위: 1 = 약 2.69mA
# 예: 1000 -> 2.69A (매우 높음). 안전하게 300~500 사이에서 시작하세요.
CURRENT_LIMIT = 100

# 제어 주기 (초 단위)
# 0.005초 = 200Hz. 통신 속도에 따라 조절이 필요할 수 있습니다.
CONTROL_PERIOD = 0.005

# --- 다이나믹셀 설정 (고정 값) ---
# -----------------------------------------------------
DXL_ID                      = 1                 # 다이나믹셀 ID
BAUDRATE                    = 3000000           # 통신 속도
DEVICENAME                  = '/dev/ttyUSB0'    # 포트 이름 (Linux 기준)
PROTOCOL_VERSION            = 2.0               # 프로토콜 버전

# 제어 테이블 주소 (XH/XM 시리즈 기준)
ADDR_OPERATING_MODE         = 11
ADDR_TORQUE_ENABLE          = 64
ADDR_CURRENT_LIMIT          = 38
ADDR_GOAL_CURRENT           = 102
ADDR_PRESENT_POSITION       = 132
ADDR_PRESENT_CURRENT        = 126

# 동작 모드 상수
CURRENT_CONTROL_MODE        = 0                 # 전류 제어 모드

# --- 코드 시작 ---
# -----------------------------------------------------

# 포트 및 패킷 핸들러 초기화
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def setup_dynamixel(dxl_id):
    """다이나믹셀을 전류 제어 모드로 설정하고 토크를 켭니다."""
    # 포트 열기
    if not portHandler.openPort():
        print("포트를 여는 데 실패했습니다.")
        quit()
    print("포트를 성공적으로 열었습니다.")

    # 통신 속도 설정
    if not portHandler.setBaudRate(BAUDRATE):
        print("통신 속도를 설정하는 데 실패했습니다.")
        quit()
    print("통신 속도를 성공적으로 설정했습니다.")

    # (1) 토크 끄기 (모드 변경 전 필수)
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)

    # (2) 동작 모드를 '전류 제어 모드' (0)으로 변경
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"ID {dxl_id}: 동작 모드 변경 실패 - {packetHandler.getTxRxResult(dxl_comm_result)}")
        quit()
    elif dxl_error != 0:
        print(f"ID {dxl_id}: 동작 모드 변경 에러 - {packetHandler.getRxPacketError(dxl_error)}")
        quit()
    print(f"ID {dxl_id}: 전류 제어 모드로 설정 완료.")

    # (3) 전류 제한 값 설정 (안전 장치)
    packetHandler.write2ByteTxRx(portHandler, dxl_id, ADDR_CURRENT_LIMIT, CURRENT_LIMIT)

    # (4) 토크 켜기
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"ID {dxl_id}: 토크 활성화 실패")
        quit()
    print(f"ID {dxl_id}: 토크가 활성화되었습니다.")


def clamp(value, min_val, max_val):
    """값을 지정된 범위 내로 제한하는 함수"""
    return max(min_val, min(value, max_val))


def main_control_loop(dxl_id):
    """PD 제어를 수행하는 메인 루프"""
    print("PD 제어 루프를 시작합니다. Ctrl+C를 눌러 종료하세요.")

    previous_error = 0
    last_time = time.time()

    while True:
        # (A) 현재 위치 읽기
        present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            # 통신 에러 발생 시 루프를 계속 진행
            print("현재 위치 읽기 실패")
            continue

        # (B) 시간 변화량(dt) 계산
        current_time = time.time()
        dt = current_time - last_time
        if dt <= 0:
            continue

        # (C) 오차 계산
        error = TARGET_POSITION - present_position

        # (D) PD 제어량 계산
        p_term = KP * error
        d_term = KD * (error - previous_error) / dt

        # (E) 최종 목표 전류 계산
        goal_current = p_term + d_term
        
        # (F) 목표 전류 값을 안전 범위 내로 제한
        clamped_current = int(clamp(goal_current, -CURRENT_LIMIT, CURRENT_LIMIT))

        # (G) 목표 전류 명령 전송
        packetHandler.write2ByteTxRx(portHandler, dxl_id, ADDR_GOAL_CURRENT, clamped_current)

        # (H) 다음 루프를 위해 상태 업데이트
        previous_error = error
        last_time = current_time

        # 현재 상태 출력
        print(f"목표: {TARGET_POSITION} | 현재: {present_position} | 오차: {error:4d} | 목표 전류: {clamped_current:4d}", end='\r')

        # 제어 주기 유지
        time.sleep(CONTROL_PERIOD)


if __name__ == '__main__':
    setup_dynamixel(DXL_ID)
    try:
        main_control_loop(DXL_ID)
    except KeyboardInterrupt:
        print("\n프로그램을 종료합니다.")
    finally:
        # 프로그램 종료 시 안전하게 토크 끄고 포트 닫기
        print("토크를 끄고 포트를 닫습니다.")
        packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, 0)
        portHandler.closePort()