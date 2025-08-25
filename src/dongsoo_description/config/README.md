# Dongsoo Robot Configuration Files

이 디렉토리에는 Dongsoo 5자유도 로봇 팔의 설정 파일들이 포함되어 있습니다.

## 파일 구조

```
config/
├── README.md                    # 이 파일
├── gravity_dh_param.json       # 중력보상용 4-DOF DH 파라미터
├── link_inertial.json          # 링크 질량 및 질량중심 정보
├── base_to_gripper_dh.json     # 완전한 5-DOF DH 파라미터 (Base → Gripper)
└── base_to_camera_dh.json      # 확장된 7-frame DH 파라미터 (Base → Camera)
```

## 파일별 상세 설명

### 1. `gravity_dh_param.json`
**용도**: motor_connect.cpp에서 중력보상 계산에 사용  
**특징**: 
- 4자유도 DH 파라미터 (단순화된 중력보상용)
- 중력 벡터 정보 포함
- motor_connect.cpp가 자동으로 로드하여 사용

### 2. `link_inertial.json`
**용도**: motor_connect.cpp에서 중력보상 계산에 사용  
**특징**:
- 4개 링크의 질량 및 질량중심 좌표
- CAD 데이터에서 변환된 정확한 물성치
- motor_connect.cpp가 자동으로 로드하여 사용

### 3. `base_to_gripper_dh.json` ⭐ **새로 추가**
**용도**: 완전한 5자유도 로봇 운동학을 위한 DH 파라미터  
**특징**:
- Base에서 Gripper까지 전체 5개 관절
- 순방향/역방향 운동학 계산용
- 궤적 계획 및 시뮬레이션용
- 다른 소프트웨어에서 가공하여 사용 가능

**DH Table**:
```
Frame | θ        | d(mm)  | a(mm) | α     | Description
------|----------|--------|-------|-------|------------------
0->1  | θ₁       | 115.75 | 0.0   | π/2   | Base rotation
1->2  | θ₂+π/2   | 0.0    | 250.0 | 0.0   | Shoulder pitch  
2->3  | θ₃       | 0.0    | 250.0 | 0.0   | Elbow pitch
3->4  | θ₄+π/2   | 0.0    | 0.0   | π/2   | Wrist pitch
4->H  | θ₅       | 215.9  | 0.0   | 0.0   | Wrist roll → Gripper
```

### 4. `base_to_camera_dh.json` ⭐ **새로 추가**  
**용도**: 카메라 프레임까지의 완전한 운동학 체인  
**특징**:
- 5개 모터 관절 + 2개 고정 프레임 = 7 프레임
- 카메라 기반 비전 애플리케이션용
- Hand-eye 캘리브레이션용
- 정밀한 카메라 포즈 계산용

**확장된 DH Table**:
```
Frame | θ        | d(mm)  | a(mm) | α     | Type  | Description
------|----------|--------|-------|-------|-------|------------------
0->1  | θ₁       | 115.75 | 0.0   | π/2   | Motor | Base rotation
1->2  | θ₂+π/2   | 0.0    | 250.0 | 0.0   | Motor | Shoulder pitch
2->3  | θ₃       | 0.0    | 250.0 | 0.0   | Motor | Elbow pitch
3->4  | θ₄+π/2   | 0.0    | 0.0   | π/2   | Motor | Wrist pitch
4->d₁ | θ₅       | 122.0  | 0.0   | 0.0   | Motor | Wrist roll
d₁->d₂| 0.0      | 0.0    | 40.3  | 0.0   | Fixed | Camera mount 1
d₂->C | π/2      | 0.0    | 11.4  | 0.0   | Fixed | Camera frame
```

## 사용 방법

### Motor Control (motor_connect.cpp)
```cpp
// 자동으로 로드됨 - 사용자 개입 불필요
// gravity_dh_param.json + link_inertial.json 사용
```

### Python/C++에서 DH 파라미터 사용
```python
import json

# Gripper까지 완전한 5-DOF 운동학
with open('base_to_gripper_dh.json', 'r') as f:
    gripper_dh = json.load(f)
    
# Camera까지 확장된 운동학 체인  
with open('base_to_camera_dh.json', 'r') as f:
    camera_dh = json.load(f)
```

### MATLAB/Robotics Toolbox 사용
```matlab
% JSON 파일을 읽어서 DH 파라미터 추출
gripper_data = jsondecode(fileread('base_to_gripper_dh.json'));
dh_params = gripper_data.dh_parameters.joints;

% Robotics Toolbox로 로봇 모델 생성
L = []; 
for i = 1:length(dh_params)
    joint = dh_params(i);
    L = [L Link([joint.dh_params.theta, joint.dh_params.d, ...
                 joint.dh_params.a, joint.dh_params.alpha])];
end
robot = SerialLink(L, 'name', 'Dongsoo5DOF');
```

## 파일 용도별 가이드

| 목적 | 사용할 파일 |
|------|-------------|  
| **모터 제어** | `gravity_dh_param.json` + `link_inertial.json` |
| **5-DOF 운동학** | `base_to_gripper_dh.json` |
| **카메라 비전** | `base_to_camera_dh.json` |
| **시뮬레이션** | `base_to_gripper_dh.json` |
| **Hand-eye 캘리브레이션** | `base_to_camera_dh.json` |

## 좌표계 규약

- **DH Convention**: Standard Denavit-Hartenberg 
- **좌표계**: Right-handed coordinate system
- **단위**: 각도(radians), 길이(meters)
- **각도 오프셋**: Joint 2, 4는 π/2 오프셋 존재

## 주의사항

1. **motor_connect.cpp**: `gravity_dh_param.json`과 `link_inertial.json`만 사용
2. **새로운 DH 파일**: 다른 소프트웨어에서 가공하여 사용하는 용도
3. **단위 통일**: 모든 거리는 meters, 각도는 radians
4. **파일 경로**: `/home/pc/soomac_ws/src/dongsoo_description/config/` 위치 고정

## 업데이트 방법

로봇의 물리적 변경 시:
1. **기구적 변경**: 해당 DH 파라미터 JSON 파일 수정  
2. **질량 변경**: `link_inertial.json` 수정
3. **재컴파일**: motor_connect.cpp는 재컴파일 필요 없음 (JSON 자동 로드)