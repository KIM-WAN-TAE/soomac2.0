#!/usr/bin/env python3
import sys
import time
import signal
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

#— 제어 테이블 주소 —#
ADDR_OPERATING_MODE       = 11
ADDR_TORQUE_ENABLE        = 64
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY     = 112
ADDR_GOAL_CURRENT         = 102
ADDR_GOAL_POSITION        = 116
ADDR_PRESENT_POSITION     = 132

#— 모드 및 설정 값 —#
OPERATING_MODE_CURRENT_BASED_POSITION = 5
TORQUE_ENABLE  = 1
DEVICENAME     = '/dev/ttyUSB0'
BAUDRATE       = 3000000
PROTOCOL_VER   = 2.0
DXL_ID         = 1

#— 제어 파라미터 —#
goal_position = 204
Kp = 0.5                 # 비례 이득 (tune 필요)
current_limit = 10     # 최대 전류 한계 (raw 값)

running = True
def exit_handler(signum, frame):
    global running
    running = False

signal.signal(signal.SIGINT, exit_handler)

def clamp(val, vmin, vmax):
    return max(vmin, min(vmax, val))

def main():
    port = PortHandler(DEVICENAME)
    pkt  = PacketHandler(PROTOCOL_VER)
    if not port.openPort() or not port.setBaudRate(BAUDRATE):
        print("포트/보레이트 설정 실패"); sys.exit(1)

    # 모드 설정 및 토크 온
    pkt.write1ByteTxRx(port, DXL_ID, ADDR_OPERATING_MODE, OPERATING_MODE_CURRENT_BASED_POSITION)
    pkt.write1ByteTxRx(port, DXL_ID, ADDR_TORQUE_ENABLE,    TORQUE_ENABLE)
    pkt.write4ByteTxRx(port, DXL_ID, ADDR_PROFILE_ACCELERATION, 50)
    pkt.write4ByteTxRx(port, DXL_ID, ADDR_PROFILE_VELOCITY,     50)

    # 메인 제어 루프
    while running:
        # 현재 위치 읽기
        pos, comm, err = pkt.read4ByteTxRx(port, DXL_ID, ADDR_PRESENT_POSITION)
        if comm != COMM_SUCCESS:
            print(f"읽기 에러: {pkt.getRxPacketError(err)}")
            continue

        # 위치 오차 계산
        error = goal_position - pos

        # 전류 목표치 계산 (비례제어)
        target_current = clamp(int(abs(error) * Kp), 0, current_limit)

        # 방향 고려 (부호)
        if error < 0:
            target_current = -target_current

        # 목표 전류/위치 지속 전송
        pkt.write2ByteTxRx(port, DXL_ID, ADDR_GOAL_CURRENT,  target_current)
        pkt.write4ByteTxRx(port, DXL_ID, ADDR_GOAL_POSITION, goal_position)

        print(f"현재:{pos}  오차:{error}  전류:{target_current}")
        time.sleep(0.02)

    # 종료 시 토크 오프
    pkt.write1ByteTxRx(port, DXL_ID, ADDR_TORQUE_ENABLE, 0)
    port.closePort()

if __name__ == '__main__':
    main()
