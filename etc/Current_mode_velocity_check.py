#!/usr/bin/env python3
import time
import random
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# ————— 설정 —————
DEVICENAME           = '/dev/ttyUSB0'
BAUDRATE             = 3_000_000
PROTOCOL_VERSION     = 2.0
DXL_IDS              = [1, 2]  # 모터 ID 리스트

# 제어 테이블 주소 (XH 시리즈, Protocol 2.0 기준)
ADDR_OPERATING_MODE   = 11
ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_CURRENT     = 102
ADDR_PRESENT_VELOCITY = 128

# 모드 값
CURRENT_CONTROL_MODE = 0
TORQUE_ENABLE        = 1

def clamp(value, vmin, vmax):
    return max(vmin, min(vmax, value))

def main():
    portHandler  = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    if not portHandler.openPort():
        print(f"포트 열기 실패: {DEVICENAME}")
        return
    if not portHandler.setBaudRate(BAUDRATE):
        print(f"보율 설정 실패: {BAUDRATE}")
        return

    # 모터 초기화
    for dxl_id in DXL_IDS:
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE)
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    print("초기화 완료. 5초마다 랜덤 Current 부과, 0.1초마다 속도 출력 시작.")

    last_current_time = time.time()
    try:
        while True:
            now = time.time()
            # 5초마다 랜덤 current 부과
            if now - last_current_time >= 2.0:
                for dxl_id in DXL_IDS:
                    raw_current = random.randint(-15, 15)
                    packetHandler.write2ByteTxRx(
                        portHandler, dxl_id, ADDR_GOAL_CURRENT,
                        clamp(raw_current, -1023, 1023)
                    )
                last_current_time = now

            # 0.1초마다 Present Velocity 읽어 출력
            for dxl_id in DXL_IDS:
                vel_raw, com_result, dxl_error = packetHandler.read4ByteTxRx(
                    portHandler, dxl_id, ADDR_PRESENT_VELOCITY
                )
                if com_result != COMM_SUCCESS or dxl_error != 0:
                    err = packetHandler.getRxPacketError(dxl_error)
                    print(f"[ID {dxl_id}] 속도 읽기 오류: {err}")
                else:
                    # signed 변환
                    if vel_raw & 0x80000000:
                        vel_raw -= 0x100000000
                    print(f"[ID {dxl_id}] Present Velocity: {vel_raw}")
            print("---")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("프로그램 종료 중...")

    # 마무리: 토크 비활성화 및 포트 닫기
    for dxl_id in DXL_IDS:
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
    portHandler.closePort()

if __name__ == '__main__':
    main()
