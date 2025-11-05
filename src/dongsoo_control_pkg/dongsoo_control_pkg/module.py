INIT_POSE = {
    'frame' : 'l',
    'pose' : [0.2, -0.2, 0.1],
    'posture' : 'down',
    'wrist' : 0.0
}

class Deliver:
    def step(self, step):
        # 초기 위치
        if step == 'step_1':
            ans = {
                'frame'    : INIT_POSE['frame'],
                'position' : INIT_POSE['pose'],
                'look'     : INIT_POSE['posture'],
                'time'     : 3.0,
                'wrist'    : INIT_POSE['wrist'],
                'requires_ack' : True,
                'next_step' : 'step_2'
            }
            return ans
        
        # 디텍 위치
        elif step == 'step_2':
            ans = {
                'frame'    : 'j',
                'position' : [-90.0, 71.63, -116.19, -46.76],
                'look'     : 'straight',
                'time'     : 2.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_3'
            }
            return ans
        
        # 잡으러 진입
        elif step == 'step_3':
            ans = {
                'camera_trigger' : True,
                'requires_ack' : True,
                'next_step' : 'step_4'
            }
            return ans

        elif step == 'step_4':
            ans = {
                'camera_tool_move' : True,
                'requires_ack' : True,
                'next_step' : 'step_5'
            }
            return ans
        
        elif step == 'step_5':
            ans = {
                'gripper' : 'close',
                'requires_ack' : True,
                'next_step': 'step_6'
            }
            return ans
        
        elif step == 'step_6':
            ans = {
                'camera_tool_up_1_move' : True,
                'requires_ack' : True,
                'next_step' : 'step_7_1'
            }
            return ans
        
        elif step == 'step_7_1':
            ans = {
                'camera_tool_up_2_move' : True,
                'requires_ack' : True,
                'next_step' : 'step_7_2'
            }
            return ans
        
        elif step == 'step_7_2':
            ans = {
                'camera_tool_up_3_move' : True,
                'requires_ack' : True,
                'next_step' : 'step_7'
            }
            return ans
        
        elif step == 'step_7':
            ans = {
                'camera_tool_back_move' : True,
                'requires_ack' : True,
                'next_step' : 'step_8_0'
            }
            return ans
        
        # 경유점
        elif step == 'step_8_0':
            ans = {
                'frame'    : INIT_POSE['frame'],
                'position' : INIT_POSE['pose'],
                'look'     : INIT_POSE['posture'],
                'time'     : 2.0,
                'wrist'    : INIT_POSE['wrist'],
                'requires_ack' : True,
                'next_step' : 'step_8'
            }
            return ans
        
        # 전달 위치로 이동
        elif step == 'step_8':
            ans = {
                'frame'    : 'l',
                'position' : [0.4, 0.0, 0.06],
                'look'     : 'down',
                'time'     : 3.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_9'
            }
            return ans
        
        # 그리퍼 개방
        elif step == 'step_9':
            ans = {
                'gripper' : 'open',
                'requires_ack' : True,
                'next_step': 'step_10'
            }
            return ans
        
        # 초기 위치로 이동
        elif step == 'step_10':
            ans = {
                'frame'    : 'l',
                'position' : [0.4, 0.0, 0.15],
                'look'     : 'down',
                'time'     : 0.7,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_11'
            }
            return ans
        
        # 초기 위치로 이동
        elif step == 'step_11':
            ans = {
                'frame'    : INIT_POSE['frame'],
                'position' : INIT_POSE['pose'],
                'look'     : INIT_POSE['posture'],
                'time'     : 1.5,
                'wrist'    : INIT_POSE['wrist'],
                'requires_ack' : True,
                'next_step' : 'None'
            }
            return ans
        
class Box:
    def step(self, step):
        # 초기 위치
        if step == 'step_1':
            ans = {
                'frame'    : INIT_POSE['frame'],
                'position' : INIT_POSE['pose'],
                'look'     : INIT_POSE['posture'],
                'time'     : 3.0,
                'wrist'    : INIT_POSE['wrist'],
                'requires_ack' : True,
                'next_step' : 'step_2'
            }
            return ans
        
        # 디텍 위치
        # 이거 좌표 따야함
        elif step == 'step_2':
            ans = {
                'frame'    : 'l',
                'position' : [0.0, -0.27, 0.27],
                'look'     : 'down',
                'time'     : 2.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_3'
            }
            return ans
        
        # 좌표 요청
        elif step == 'step_3':
            ans = {
                'camera_trigger' : True,
                'requires_ack' : True,
                'next_step' : 'step_5'
            }
            return ans
        
        # 박스 위치로 이동
        elif step == 'step_5':
            ans = {
                'camera_box_move' : True,
                'requires_ack' : True,
                'next_step' : 'step_6'
            }
            return ans
        
        # 잡기
        elif step == 'step_6':
            ans = {
                'gripper' : 'close',
                'requires_ack' : True,
                'next_step': 'step_7'
            }
            return ans
        
        # 잡고 상부로 이동
        elif step == 'step_7':
            ans = {
                'camera_box_up_move' : True,
                'requires_ack' : True,
                'next_step' : 'step_8'
            }
            return ans
        
        # 전달 위치로 이동
        elif step == 'step_8':
            ans = {
                'frame'    : 'l',
                'position' : [0.4, 0.0, 0.06],
                'look'     : 'down',
                'time'     : 3.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_9'
            }
            return ans
        
        # 그리퍼 개방
        elif step == 'step_9':
            ans = {
                'gripper' : 'open',
                'requires_ack' : True,
                'next_step': 'step_10'
            }
            return ans
        
        # 초기 위치로 이동
        elif step == 'step_10':
            ans = {
                'frame'    : 'l',
                'position' : [0.4, 0.0, 0.15],
                'look'     : 'down',
                'time'     : 0.7,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_11'
            }
            return ans
        
        # 초기 위치로 이동
        elif step == 'step_11':
            ans = {
                'frame'    : INIT_POSE['frame'],
                'position' : INIT_POSE['pose'],
                'look'     : INIT_POSE['posture'],
                'time'     : 1.5,
                'wrist'    : INIT_POSE['wrist'],
                'requires_ack' : True,
                'next_step' : 'None'
            }
            return ans

class Start:
    def step(self, step):
        if step == 'step_1':
            ans = {
                'frame'    : INIT_POSE['frame'],
                'position' : INIT_POSE['pose'],
                'look'     : INIT_POSE['posture'],
                'time'     : 3.0,
                'wrist'    : INIT_POSE['wrist'],
                'requires_ack' : True,
                'next_step' : 'step_2'
            }
            return ans
        
         # 그러퍼 잡기
        elif step == 'step_2':
            ans = {
                'gripper' : 'close',
                'requires_ack' : True,
                'next_step': 'step_3'
            }
            return ans
        
        # 조명 키기 초기 위치
        elif step == 'step_3':
            ans = {
                'frame'    : 'j',
                'position' : [-93.96, 24.70, -109.07, 0.7],
                'look'     : 'straight',
                'time'     : 2.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_4'
            }
            return ans
        
        # 수직으로 진입하는 함수
        elif step == 'step_4':
            ans = {
                'frame'    : 'l',
                'position' : [-0.04, -0.46, 0.45],
                'look'     : 'straight',
                'time'     : 0.8,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_5'
            }
            return ans
        
        # 수직으로 퇴장
        elif step == 'step_5':
            ans = {
                'frame'    : 'j',
                'position' : [-93.96, 24.70, -109.07, 0.7],
                'look'     : 'straight',
                'time'     : 2.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_6'
            }
            return ans
        
        # 초기화 위치로 이동
        elif step == 'step_6':
            ans = {
                'frame'    : INIT_POSE['frame'],
                'position' : INIT_POSE['pose'],
                'look'     : INIT_POSE['posture'],
                'time'     : 3.0,
                'wrist'    : INIT_POSE['wrist'],
                'requires_ack' : True,
                'next_step' : 'step_7'
            }
            return ans
        
        elif step == 'step_7':
            ans = {
                'gripper' : 'open',
                'requires_ack' : True,
                'next_step': 'None'
            }
            return ans
         
class Finish:
    def step(self, step):
        if step == 'step_1':
            ans = {
                'frame'    : INIT_POSE['frame'],
                'position' : INIT_POSE['pose'],
                'look'     : INIT_POSE['posture'],
                'time'     : 3.0,
                'wrist'    : INIT_POSE['wrist'],
                'requires_ack' : True,
                'next_step' : 'step_2'
            }
            return ans
        
         # 그러퍼 잡기
        elif step == 'step_2':
            ans = {
                'gripper' : 'close',
                'requires_ack' : True,
                'next_step': 'step_3'
            }
            return ans
        
        # 조명 키기 초기 위치
        elif step == 'step_3':
            ans = {
                'frame'    : 'j',
                'position' : [-93.96, 24.70, -109.07, 0.7],
                'look'     : 'straight',
                'time'     : 2.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_4'
            }
            return ans
        
        # 수직으로 진입하는 함수
        elif step == 'step_4':
            ans = {
                'frame'    : 'l',
                'position' : [-0.042, -0.46, 0.427],
                'look'     : 'straight',
                'time'     : 0.8,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_5'
            }
            return ans
        
        # 수직으로 퇴장
        elif step == 'step_5':
            ans = {
                'frame'    : 'j',
                'position' : [-93.96, 24.70, -109.07, 0.7],
                'look'     : 'straight',
                'time'     : 2.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_6'
            }
            return ans
        
        # 초기화 위치로 이동
        elif step == 'step_6':
            ans = {
                'frame'    : INIT_POSE['frame'],
                'position' : INIT_POSE['pose'],
                'look'     : INIT_POSE['posture'],
                'time'     : 3.0,
                'wrist'    : INIT_POSE['wrist'],
                'requires_ack' : True,
                'next_step' : 'step_7'
            }
            return ans
        
        elif step == 'step_7':
            ans = {
                'gripper' : 'open',
                'requires_ack' : True,
                'next_step': 'None'
            }
            return ans
        
class Return:
    def step(self, step):
        # 초기 위치
        if step == 'step_1':
            ans = {
                'frame'    : INIT_POSE['frame'],
                'position' : INIT_POSE['pose'],
                'look'     : INIT_POSE['posture'],
                'time'     : 3.0,
                'wrist'    : INIT_POSE['wrist'],
                'requires_ack' : True,
                'next_step' : 'step_2'
            }
            return ans
        
        # 디텍 위치
        # 정반 바닥을 보고 있는 좌표를 하나 따야함
        elif step == 'step_2':
            ans = {
                'frame'    : 'l',
                'position' : [0.20, 0.0, 0.28],
                'look'     : 'down',
                'time'     : 2.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_3'
            }
            return ans
        
        # 잡으러 가는 좌표 요청
        elif step == 'step_3':
            ans = {
                'return_camera_trigger' : True,
                'requires_ack' : True,
                'next_step' : 'step_4'
            }
            return ans
        
        # 도구 수직으로 이동
        elif step == 'step_4':
            ans = {
                'camera_return_tool_up_move' : True,
                'requires_ack' : True,
                'next_step' : 'step_5'
            }
            return ans
        
        # 도구 잡으러 이동
        elif step == 'step_5':
            ans = {
                'camera_return_tool_move' : True,
                'requires_ack' : True,
                'next_step' : 'step_6'
            }
            return ans
        
        elif step == 'step_6':
            ans = {
                'gripper' : 'close',
                'requires_ack' : True,
                'next_step': 'step_7'
            }
            return ans
        
        elif step == 'step_7':
            ans = {
                'camera_return_tool_grip_up_move' : True,
                'requires_ack' : True,
                'next_step' : 'step_8_1'
            }
            return ans
        
        elif step == 'step_8_1':
            ans = {
                'frame'    : 'l',
                'position' : [0.0, -0.25, 0.2],
                'look'     : 'down',
                'time'     : 2.0,
                'wrist'    : 0.0,
                'return_list_clean' : True,
                'requires_ack' : True,
                'next_step' : 'step_8_2'
            }
            return ans
        
        # 좌표 따야함
        elif step == 'step_8_2':
            ans = {
                'frame'    : 'l',
                'position' : [-0.29, -0.3, 0.1],
                'look'     : 'down',
                'time'     : 3.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_8'
            }
            return ans
        
        elif step == 'step_8':
            ans = {
                'frame'    : 'l',
                'position' : [-0.29, -0.3, 0.0],
                'look'     : 'down',
                'time'     : 1.0,
                'wrist'    : -40.0,
                'requires_ack' : True,
                'next_step' : 'step_9'
            }
            return ans
        
        elif step == 'step_9':
            ans = {
                'gripper' : 'open',
                'requires_ack' : True,
                'next_step': 'step_10'
            }
            return ans
        
        elif step == 'step_10':
            ans = {
                'frame'    : 'l',
                'position' : [-0.29, -0.3, 0.15],
                'look'     : 'down',
                'time'     : 1.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_11'
            }
            return ans
        
        elif step == 'step_11':
            ans = {
                'frame'    : 'l',
                'position' : [0.0, -0.25, 0.2],
                'look'     : 'down',
                'time'     : 2.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'check'
            }
            return ans
        
        ### 구분선 구분선 구분선
        
        elif step == 'M3_step_1':
            ans = {
                'frame'    : 'l',
                'position' : [0.25, 0.0, 0.28],
                'look'     : 'down',
                'time'     : 2.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'M3_step_2'
            }
            return ans

        elif step == 'M3_step_2':
            ans = {
                'return_camera_trigger' : True,
                'requires_ack' : True,
                'next_step' : 'M3_step_3'
            }
            return ans

        elif step == 'M3_step_3':
            ans = {
                'camera_return_tool_up_move' : True,
                'requires_ack' : True,
                'next_step' : 'M3_step_4'
            }
            return ans
        
        elif step == 'M3_step_4':
            ans = {
                'camera_return_box_move' : True,
                'requires_ack' : True,
                'next_step' : 'M3_step_5'
            }
            return ans
        
        elif step == 'M3_step_5':
            ans = {
                'gripper' : 'close',
                'requires_ack' : True,
                'next_step': 'M3_step_6'
            }
            return ans
        
        elif step == 'M3_step_6':
            ans = {
                'camera_return_tool_grip_up_move' : True,
                'requires_ack' : True,
                'next_step' : 'M3_step_7'
            }
            return ans
        ## 드랍 위치 상부로 진입
        elif step == 'M3_step_7':
            ans = {
                'camera_return_box_drop_top_move' : True,
                'return_list_clean' : True,
                'requires_ack' : True,
                'next_step' : 'M3_step_7_1'
            }
            return ans
        
        # 드랍 위치로 이동
        elif step == 'M3_step_7_1':
            ans = {
                'camera_return_box_drop_move' : True,
                'requires_ack' : True,
                'next_step' : 'M3_step_7_2'
            }
            return ans
        
        elif step == 'M3_step_7_2':
            ans = {
                'gripper' : 'open',
                'requires_ack' : True,
                'next_step': 'M3_step_8'
            }
            return ans
        
        # 다시 상승
        elif step == 'M3_step_8':
            ans = {
                'camera_return_box_drop_top_move' : True,
                'M3_list_clean' : True,
                'requires_ack' : True,
                'next_step' : 'final'
            }
            return ans

    
        
        elif step == 'final':
            ans = {
                'frame'    : INIT_POSE['frame'],
                'position' : INIT_POSE['pose'],
                'look'     : INIT_POSE['posture'],
                'time'     : 2.0,
                'wrist'    : INIT_POSE['wrist'],
                'requires_ack' : True,
                'next_step' : 'None'
            }
            return ans

class Fuck:
    def step(self, step):
        # 초기 위치
        if step == 'step_1':
            ans = {
                'frame'    : INIT_POSE['frame'],
                'position' : INIT_POSE['pose'],
                'look'     : INIT_POSE['posture'],
                'time'     : 3.0,
                'wrist'    : INIT_POSE['wrist'],
                'requires_ack' : True,
                'next_step' : 'step_2'
            }
            return ans
        
        if step == 'step_2':
            ans = {
                    'led_on' : True,
                    'requires_ack' : True,
                    'next_step' : 'None'
                }
            return ans
        
class Shit:
    def step(self, step):
        # 초기 위치
        if step == 'step_1':
            ans = {
                    'led_off' : True,
                    'requires_ack' : False,
                    'next_step' : 'step_2'
                }
            return ans
        
        elif step == 'step_2':
            ans = {
                'frame'    : INIT_POSE['frame'],
                'position' : INIT_POSE['pose'],
                'look'     : INIT_POSE['posture'],
                'time'     : 3.0,
                'wrist'    : INIT_POSE['wrist'],
                'requires_ack' : True,
                'next_step' : 'None'
            }
            return ans