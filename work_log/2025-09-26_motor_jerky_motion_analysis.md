# Work Log - 2025년 9월 26일

## 작업 개요
- **프로젝트**: soomac_ws 5자유도 로봇팔 제어 시스템
- **개발 브랜치**: dev_1
- **주요 작업**: 모터 동작 끊김 현상("뚝, 뚝") 분석 및 해결 방안 연구

---

## 문제 상황

### 발생 현상
- **Service 호출을 통한 목표점 이동은 정상 동작**
- **이동 과정에서 모터가 "뚝, 뚝" 끊겨 보이는 불연속적 동작**
- 부자연스러운 움직임으로 인한 사용성 저하

### 사용자 예상 원인
```
goal Current가 최소로 동작하는데 필요한 point가 누적되다 보니
특정 point가 누적되면 그제서야 current 값이 움직일 수 있을 정도의
goal Current가 되어 동작
```

---

## 시스템 아키텍처 분석

### 1. dongsoo_cpp_pkg/motor_connect.cpp (C++ 실시간 제어)
```cpp
// 핵심 사양
- 제어 주기: 5ms (200Hz)
- 제어 모드: Current Control Mode
- PID + 중력보상 알고리즘
- 토픽: /motor/command_position (구독)
```

**주요 제어 로직:**
- **세트포인트 램핑**: 최대 3000 cnt/sec로 급격한 변화 방지
- **소프트 전류 제한**: XH540(900), XH430(500)
- **적분 누적 제한**: MAX_INTEGRAL_ERROR = 2000.0f (수정 후)

### 2. dongsoo_py_pkg/dongsoo_server.py (Python 상위 제어)
```python
# 핵심 사양
- Service 서버: dongsoo_executor
- Trajectory 생성: 200Hz (5ms 간격)
- req.time 인자로 동작 시간 제어
- 토픽: /motor/command_position (발행)
```

**Trajectory 전송 방식:**
```python
for i, q_s in enumerate(q_list):  # 개별 point 전송
    q_pulse = rad_to_pulse(q_s)
    q_msg.data = q_pulse
    self.motor_control_pub.publish(q_msg)
    time.sleep(sleep_time)  # 0.005초
```

---

## 원인 분석

### **사용자 예상이 정확함!**

#### 1. 전류 제어 모드의 Deadband 문제
```cpp
// motor_connect.cpp:599
float cmd_raw = pid_raw + KI_POS_GAINS[i] * integral_error_[i] + ff_raw;
```

**문제점:**
- **작은 위치 오차 → 작은 전류 명령**
- **모터의 최소 동작 전류(deadband) 이하**에서는 모터가 반응하지 않음
- 오차가 **적분항에 계속 누적**됨

#### 2. 적분 누적으로 인한 갑작스러운 큰 전류
```cpp
// motor_connect.cpp:591-592
integral_error_[i] += pos_err * dt + K_AW * saturation_error * dt;
integral_error_[i] = std::clamp(integral_error_[i], -MAX_INTEGRAL_ERROR, MAX_INTEGRAL_ERROR);
```

**메커니즘:**
- 작은 오차들이 적분되어 **누적**
- 임계점 도달 시 **갑작스럽게 큰 전류** 발생
- 결과: **불연속적인 "뚝, 뚝" 동작**

#### 3. 통신 오버헤드 및 타이밍 문제
- **Python-C++ 간 1000번 개별 전송** (trajectory points)
- **비동기 통신**으로 인한 불규칙한 timing jitter
- **5ms 제어 주기와 5ms 통신 주기**의 동기화 문제

---

## 시도된 해결 방안

### 1. 적분 제한 강화 ✅ (부분적 효과)
```cpp
// 기존: MAX_INTEGRAL_ERROR(5000.0f)
// 개선: MAX_INTEGRAL_ERROR(2000.0f)
```
- 과도한 적분 누적 방지
- 갑작스러운 큰 출력 완화

### 2. P게인 조정 ✅ (부분적 효과)
```cpp
// KP_POS_GAINS 조정: 4번 축 1.2f → 1.5f
KP_POS_GAINS({1.0f, 2.0f, 1.8f, 1.5f})
```
- 즉각적인 반응성 향상

### 3. 최소 전류 보장 메커니즘 ❌ (진동 악화)
```cpp
// 시도했으나 롤백함 - 강제적인 전류 주입으로 진동 악화
if (std::abs(pos_err) > 5.0f) {
    const float MIN_CURRENT = 30.0f;
    if (std::abs(cmd_raw) < MIN_CURRENT && std::abs(cmd_raw) > 1.0f) {
        cmd_raw = (cmd_raw > 0) ? MIN_CURRENT : -MIN_CURRENT;
    }
}
```

---

## 근본적 해결 방안 (미적용)

### **C 방식: Trajectory 전송 방식 개선 (권장)**

#### 현재 문제점
```python
# 현재: 개별 point 1000번 전송
for i, q_s in enumerate(q_list):  # Python → C++ 통신 오버헤드
    self.motor_control_pub.publish(q_msg)
    time.sleep(0.005)  # 타이밍 동기화 문제
```

#### 개선 계획
1. **새로운 메시지 타입 정의**
```msg
# dongsoo_interfaces/msg/TrajectoryCommand.msg
float64 total_time           # 실제 동작 시간 (req.time)
int32[] start_positions      # 시작 위치 [5개 모터]
int32[] end_positions        # 목표 위치 [5개 모터]
string trajectory_type       # "smooth" or "linear"
```

2. **Python: 한 번에 전체 trajectory 정보 전송**
```python
traj_msg = TrajectoryCommand()
traj_msg.total_time = work_time
traj_msg.start_positions = rad_to_pulse(q_start)
traj_msg.end_positions = rad_to_pulse(q_end)
self.trajectory_pub.publish(traj_msg)  # 1번만 전송!
```

3. **C++: 시간 기반 내부 interpolation**
```cpp
class TrajectoryInterpolator {
    std::chrono::steady_clock::time_point start_time_;
    double total_time_;

    std::vector<float> getNextPoint() {
        double elapsed = /* 경과 시간 계산 */;
        double t = elapsed / total_time_;  // 0.0 ~ 1.0
        double alpha = (traj_type_ == "smooth") ?
                      (3*t*t - 2*t*t*t) : t;
        // 실시간 interpolation
    }
};
```

#### 예상 효과
- **통신 오버헤드 제거**: 1000번 → 1번 전송
- **정확한 타이밍**: C++에서 5ms마다 정확한 interpolation
- **부드러운 동작**: 일정한 제어 주기 보장
- **실제 시간 제어**: req.time 준수

---

## 유의사항

### 1. 게인 튜닝 한계
- **PID 게인 조정만으로는 근본적 해결 어려움**
- 전류 제어 모드 자체의 deadband 특성이 주 원인

### 2. 대안 제어 모드 고려사항
```cpp
// Position Control Mode로 변경 고려
ADDR_OPERATING_MODE(11), POSITION_CONTROL_MODE(3)  // Current(0) → Position(3)
```
- 더 부드러운 동작 가능
- 하지만 중력보상 구현 복잡성 증가

### 3. 통신 아키텍처의 중요성
- **실시간 제어에서 통신 주기와 제어 주기의 분리** 필요
- Python 상위 제어 ↔ C++ 실시간 제어 간 역할 명확화

---

## 결론

모터 동작 끊김 현상의 **주 원인은 전류 제어 모드에서의 적분 누적과 deadband 특성**이며, 사용자의 예상이 정확했다.

**단기 해결책**: 적분 제한 강화로 부분적 개선 달성
**장기 해결책**: Trajectory 전송 방식 개선으로 근본적 해결 가능

현재 상태에서도 사용 가능하나, 더 부드러운 동작을 위해서는 **C++ 내부 interpolation 방식 도입**을 권장한다.

---

**작성일**: 2025년 9월 26일
**작성자**: Claude Code 세션
**브랜치**: dev_1