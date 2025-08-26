# CLAUDE.md

이 파일은 Claude Code가 soomac_ws 워크스페이스에서 작업할 때 가이드를 제공합니다.

## ⚠️ 세션 시작 시 필수 작업

### 최근 작업 상황 파악
Claude가 soomac_ws에서 작업을 시작할 때는 **반드시** 다음 단계를 수행해야 합니다:

1. **최신 work_log 확인**: `work_log/` 디렉토리에서 가장 최근 날짜의 파일을 읽어 이전 작업 내용 파악
2. **현재 브랜치 상태 확인**: `git status` 및 최근 커밋 확인
3. **주요 파일 변경사항 검토**: 핵심 파일들의 최신 상태 확인

```bash
# 세션 시작 시 실행할 명령어들
ls -la work_log/ | tail -5          # 최근 work log 확인
git log --oneline -5                # 최근 커밋 확인  
git status                          # 현재 상태 확인
```

**중요**: work_log를 통해 이전 세션의 작업 내용, 구현된 기능, 해결된 문제점들을 이해한 후 작업을 시작하세요.

## 세션 컨텍스트 (Session Context)

### 현재 프로젝트 상태
- **워크스페이스**: soomac_ws (5자유도 로봇팔 제어)
- **개발 브랜치**: dev_1
- **최근 작업**: DH 파라미터 JSON 모듈화, 실시간 모니터링 시스템 구축 (2025/08/26)
- **주요 패키지**: dongsoo_cpp_pkg, dongsoo_py_pkg, dongsoo_description

### 최근 주요 성과 (2025/08/26)
- **JSON 모듈화**: `read_json.py` 모듈 개발로 DH 파라미터 관리 체계화
- **Monitoring Hub 재구축**: 멀티스레딩, thread lock, 실시간 pose 계산 구현
- **코드 최적화**: JSON 파싱 로직 분리로 재사용성 및 가독성 향상
- **Forward Kinematics**: Roll-Pitch-Yaw 각도 계산 및 실시간 출력

### 이전 세션 요약
- 워크스페이스 구조 분석 완료 (2025/08/25)
- 작업 로그 시스템 구축 (`work_log/` 디렉토리)
- 실시간 모터 제어 및 중력보상 시스템 구현
- DH 파라미터 기반 순기구학 계산 구현

## 프로젝트 개요

### 주요 패키지

#### dongsoo_cpp_pkg (C++ 실시간 제어)
- **motor_connect.cpp**: XH540-V270-R 모터 제어 노드
  - 5자유도 로봇팔 제어 (ID 1-5)
  - PID + 중력보상 알고리즘
  - JSON 기반 설정 로드
  - 5ms 제어 주기

#### dongsoo_py_pkg (Python 상위 제어)
- **data_hub.py**: 순기구학 계산 및 데이터 처리
- **read_json.py**: DH 파라미터 JSON 읽기 모듈 (CameraDH, GripperDH 클래스)
- **monitoring_hub.py**: 실시간 모니터링 (멀티스레딩, pose 계산)
- **trajectory_test.py**: 궤적 계획 테스트

#### dongsoo_description (설정 패키지)
- DH 파라미터 JSON 파일들
- 링크 관성 정보

### 일반적인 명령어

#### 빌드 명령어
```bash
# 전체 워크스페이스 빌드
cd soomac_ws && colcon build

# 특정 패키지 빌드
cd soomac_ws && colcon build --packages-select dongsoo_cpp_pkg
cd soomac_ws && colcon build --packages-select dongsoo_py_pkg
```

#### 실행 명령어
```bash
# 소스 설정
source install/setup.bash

# 모터 제어 노드 실행
ros2 run dongsoo_cpp_pkg motor_connect

# 데이터 허브 실행
ros2 run dongsoo_py_pkg data_hub_py

# 모니터링 허브 실행 (실시간 pose 출력)
ros2 run dongsoo_py_pkg monitoring_hub
```

### ROS 2 Topic 구조
```
/motor/position         - 현재 모터 위치 (Int32MultiArray)
/motor/current          - 현재 모터 전류 (Float32MultiArray)
/motor/velocity         - 현재 모터 속도 (Float32MultiArray)
/motor/command_position - 목표 위치 명령 (Int32MultiArray)
/datahub/grip_pose      - 그리퍼 위치 (Float32MultiArray)
/datahub/cam_pose       - 카메라 위치 (Float32MultiArray)
```

## 개발 가이드라인

### 코딩 규칙
- C++ 패키지: ament_cmake 빌드 시스템 사용
- Python 패키지: ament_python 빌드 시스템 사용
- JSON 설정 파일을 통한 파라미터 관리
- 실시간 제어는 C++, 상위 로직은 Python 사용

### 안전 규칙
- 항상 전류 제한 설정 확인
- 시그널 핸들러를 통한 안전한 종료 구현
- 모터 토크 비활성화 후 프로그램 종료

### 파일 관리
- 작업 로그는 `work_log/` 디렉토리에 저장
- JSON 설정은 `dongsoo_description/config/`에 저장
- 공유 컨텍스트는 `shared_context/`에 저장

## 하드웨어 정보

### 모터 사양
- **ID 1-3**: XH540-V270-R (기어비 272.5:1)
- **ID 4**: XH430-V350-R (기어비 353.5:1)
- **ID 5**: 별도 제어용 모터
- **통신**: RS-485, 3Mbps

### DH 파라미터 (5자유도)
- Joint 1: d=0.11575m, alpha=π/2
- Joint 2: a=0.250m, theta_offset=π/2
- Joint 3: a=0.250m
- Joint 4: alpha=π/2, theta_offset=π/2
- Joint 5: d=0.2159m

## 문제 해결

### 일반적인 문제
1. **모터 통신 실패**: USB 포트 및 권한 확인
2. **빌드 실패**: 의존성 패키지 설치 확인
3. **JSON 로드 실패**: 파일 경로 및 형식 확인

### 디버깅 팁
- 중력보상 디버그 로그는 5초마다 출력
- ROS 2 topic echo로 실시간 데이터 모니터링
- 전류 제한 설정으로 안전성 확보

---
**마지막 업데이트**: 2025년 8월 26일
**현재 개발자**: Claude Code 사용자