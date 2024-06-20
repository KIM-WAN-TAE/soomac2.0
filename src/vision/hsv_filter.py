import cv2
import numpy as np

def update(val=0):
    # 트랙바에서 HSV 범위 값을 가져오기
    h_low = cv2.getTrackbarPos('H_low', 'Result')
    h_high = cv2.getTrackbarPos('H_high', 'Result')
    s_low = cv2.getTrackbarPos('S_low', 'Result')
    s_high = cv2.getTrackbarPos('S_high', 'Result')
    v_low = cv2.getTrackbarPos('V_low', 'Result')
    v_high = cv2.getTrackbarPos('V_high', 'Result')

    # 마스크 생성
    lower_bound = np.array([h_low, s_low, v_low])
    upper_bound = np.array([h_high, s_high, v_high])
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    # 결과 이미지 생성
    result = cv2.bitwise_and(frame, frame, mask=mask)
    cv2.imshow('Result', result)

cap = cv2.VideoCapture(4)  # 웹캠을 사용하여 캡처 객체 생성

# 캡처 객체가 정상적으로 열렸는지 확인
if not cap.isOpened():
    raise Exception("Could not open video device")

ret, frame = cap.read()  # 첫 프레임 읽기
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

cv2.namedWindow('Result')
cv2.createTrackbar('H_low', 'Result', 0, 179, update)
cv2.createTrackbar('H_high', 'Result', 10, 179, update)
cv2.createTrackbar('S_low', 'Result', 69, 255, update)
cv2.createTrackbar('S_high', 'Result', 255, 255, update)
cv2.createTrackbar('V_low', 'Result', 118, 255, update)
cv2.createTrackbar('V_high', 'Result', 255, 255, update)

while True:
    ret, frame = cap.read()  # 프레임 읽기
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # BGR을 HSV로 변환

    cv2.imshow('Original', frame)
    update()  # 이미지 업데이트

    if cv2.waitKey(1) & 0xFF == ord('q'):  # 'q' 키를 누르면 종료
        break

cap.release()  # 캡처 객체 해제
cv2.destroyAllWindows()
