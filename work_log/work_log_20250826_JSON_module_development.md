# Work Log - 2025년 8월 26일: JSON 모듈화 및 Monitoring Hub 개발

## 세션 개요
**날짜**: 2025년 8월 26일  
**브랜치**: dev_1  
**주요 작업**: DH 파라미터 JSON 읽기 모듈화 및 실시간 모니터링 시스템 구축  

## 주요 성과

### 1. JSON 읽기 모듈 개발 (`read_json.py`)

#### 1-1. 문제 상황
- `data_hub.py`에서 JSON 관련 코드가 너무 많은 비중 차지 (가독성 저하)
- JSON 파싱 로직의 중복 사용
- 모듈화 필요성 대두

#### 1-2. 솔루션 구현
**파일 생성**: `src/dongsoo_py_pkg/dongsoo_py_pkg/read_json.py`

**주요 클래스 구조**:
```python
class DHParameters:           # 기본 DH 파라미터 관리 클래스
class CameraDH(DHParameters): # 카메라 DH 파라미터 전용
class GripperDH(DHParameters): # 그리퍼 DH 파라미터 전용
create_dh_reader()            # 팩토리 함수
```

**핵심 기능**:
- `get_joint_parameter(joint_id, parameter_name)`: 개별 파라미터 조회
- `get_all_parameters(joint_id)`: 특정 조인트 전체 파라미터
- `get_parameter_list(parameter_name)`: 특정 파라미터의 전체 리스트
- `get_dh_matrix_params(joint_id, q_values)`: DH 변환 행렬용 파라미터
- `get_all_dh_params(q_values)`: 모든 조인트 계산된 파라미터

#### 1-3. 사용법 예시
```python
from dongsoo_py_pkg.read_json import CameraDH, GripperDH

cam_dh = CameraDH()
grip_dh = GripperDH()

# 개별 파라미터 조회
d1 = cam_dh.get_joint_parameter(1, 'd')

# 전체 파라미터 리스트
all_a = grip_dh.get_parameter_list('a')

# DH 행렬 파라미터 계산
q_values = [0.1, 0.2, 0.3, 0.4, 0.5]
dh_params = cam_dh.get_all_dh_params(q_values)
```

### 2. Monitoring Hub 완전 재구축

#### 2-1. 기존 문제점
- JSON 파싱 로직 중복
- 단순한 로그 출력
- 스레드 안전성 부족

#### 2-2. 새로운 기능 구현

**멀티스레딩 및 동기화**:
```python
self.data_lock = threading.Lock()

with self.data_lock:
    # 모든 데이터 접근에 lock 사용
```

**실시간 데이터 처리**:
- Joint pulse, degree, radian 변환
- Forward kinematics 계산
- Roll-Pitch-Yaw 각도 추출
- 0.1Hz (10초) 간격 출력

**출력 형식 개선**:
```
==================== MONITORING HUB ====================

=== JOINT STATUS ===
Joint #1 : Pulse:   1234 | DEG:  108.28 | Current:  0.125mA | Velocity:   12.34rpm
Joint #2 : Pulse:   2048 | DEG:  180.00 | Current:  0.087mA | Velocity:    5.67rpm
...

=== GRIPPER POSE ===
Position X Y Z :   0.4521 |   0.1234 |   0.2876
Roll Pitch Yaw :    12.45° |   -3.67° |   45.23°

=== CAMERA POSE ===
Position X Y Z :   0.5234 |   0.0987 |   0.3456
Roll Pitch Yaw :     8.92° |   -1.45° |   52.18°

==================== MONITORING HUB ====================
```

### 3. 코드 최적화 성과

#### 3-1. monitoring_hub.py 개선
- **129줄 → 223줄**: 기능 대폭 확장하면서도 구조화
- **JSON 관련 코드 완전 제거**: 23줄 삭제 (18% 코드 정리)
- **모듈화**: DH 파라미터 로직을 `read_json.py`로 분리

#### 3-2. 재사용성 향상
- 다른 Python 노드에서 `read_json` 모듈 쉽게 import 가능
- DH 파라미터 변경 시 한 곳(`read_json.py`)만 수정하면 됨

## 기술적 구현 세부사항

### Forward Kinematics 계산
```python
def dh_transform(theta, d, a, alpha):
    # 4x4 변환 행렬 생성
    T = np.array([[ ct, -st*ca,  st*sa, a*ct],
                  [ st,  ct*ca, -ct*sa, a*st],
                  [  0,      sa,     ca,    d],
                  [  0,       0,      0,    1]])
    return T

def rotation_matrix_to_rpy(R):
    # 회전 행렬을 Roll-Pitch-Yaw로 변환
    roll = np.arctan2(R[2,1], R[2,2])
    pitch = np.arctan2(-R[2,0], sy)
    yaw = np.arctan2(R[1,0], R[0,0])
    return roll, pitch, yaw
```

### 스레드 안전성 구현
```python
# MultiThreadedExecutor 사용
exec = MultiThreadedExecutor(num_threads=4)

# 데이터 동기화
with self.data_lock:
    # 모든 공유 데이터 접근
```

## 문제 해결 과정

### 1. Import 경로 오류 해결
**문제**: `ModuleNotFoundError: No module named 'read_json'`
**해결**: ROS 2 패키지 내 import 경로 수정
```python
# 오류
from read_json import CameraDH, GripperDH

# 수정
from dongsoo_py_pkg.read_json import CameraDH, GripperDH
```

### 2. 파라미터 추출 방식 개선
**요청**: 각 조인트별 파라미터 개별 변수 할당
**해결**: 여러 방법 제시 및 최적 방안 구현
```python
# 방법 1: 리스트 언패킹
grip_a_values = grip_dh.get_parameter_list('a')
grip_a1, grip_a2, grip_a3, grip_a4, grip_a5 = grip_a_values

# 방법 2: 딕셔너리 사용 (추천)
grip_params = {
    'a': grip_dh.get_parameter_list('a'),
    'd': grip_dh.get_parameter_list('d')
}
```

## 다음 단계 제안

### 1. 추가 기능 확장
- 역기구학(Inverse Kinematics) 계산 모듈
- 궤적 계획(Trajectory Planning) 통합
- 실시간 시각화 도구

### 2. 성능 최적화
- DH 파라미터 캐싱 시스템
- 계산 최적화 (NumPy 벡터화)
- 메모리 사용량 모니터링

### 3. 확장성 고려사항
- 다양한 로봇 구성 지원
- 동적 DH 파라미터 변경
- JSON 스키마 검증

## 파일 변경 사항 요약

### 새로 생성된 파일
- `src/dongsoo_py_pkg/dongsoo_py_pkg/read_json.py` (161줄)

### 수정된 파일
- `src/dongsoo_py_pkg/dongsoo_py_pkg/monitoring_hub.py` (완전 재작성, 223줄)

### 빌드 및 테스트
- 패키지 빌드 성공: `colcon build --packages-select dongsoo_py_pkg`
- 기능 테스트 완료: 모든 메서드 정상 동작 확인

## 결론

이번 세션에서 DH 파라미터 관리의 모듈화와 실시간 모니터링 시스템의 완전한 재구축을 성공적으로 완료했습니다. 코드의 재사용성과 가독성이 크게 향상되었으며, 멀티스레딩 환경에서의 안전성도 확보했습니다. 

특히 `read_json.py` 모듈은 다른 노드들에서도 쉽게 활용할 수 있는 범용 모듈로 설계되어, 향후 시스템 확장 시 중요한 기반이 될 것입니다.

---
**작성자**: Claude Code Assistant  
**작성일**: 2025년 8월 26일  
**워크스페이스**: soomac_ws