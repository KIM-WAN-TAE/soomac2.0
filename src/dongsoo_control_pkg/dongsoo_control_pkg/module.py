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
                'time'     : 3.0,
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
                'camera_tool_up_move' : True,
                'requires_ack' : True,
                'next_step' : 'step_7'
            }
            return ans
        
        elif step == 'step_7':
            ans = {
                'camera_tool_back_move' : True,
                'requires_ack' : True,
                'next_step' : 'step_8'
            }
            return ans
        
        elif step == 'step_8':
            ans = {
                'frame'    : INIT_POSE['frame'],
                'position' : INIT_POSE['pose'],
                'look'     : INIT_POSE['posture'],
                'time'     : 3.0,
                'wrist'    : INIT_POSE['wrist'],
                'requires_ack' : True,
                'next_step' : 'step_9'
            }
            return ans
        
        elif step == 'step_9':
            ans = {
                'gripper' : 'open',
                'requires_ack' : True,
                'next_step': 'None'
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
        elif step == 'step_2':
            ans = {
                'frame'    : 'l',
                'position' : [0.0, -0.25, 0.3],
                'look'     : 'down',
                'time'     : 2.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_3'
            }
            return ans
        
        # 잡으러 진입
        elif step == 'step_3':
            ans = {
                'frame'    : 'l',
                'position' : [0.02, -0.4, 0.05],
                'look'     : 'down',
                'time'     : 2.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_4'
            }
            return ans
        
        # 잡기
        elif step == 'step_4':
            ans = {
                'gripper' : 'close',
                'requires_ack' : True,
                'next_step': 'step_6'
            }
            return ans
        
        # 뒤로 빼기
        elif step == 'step_6':
            ans = {
                'frame'    : 'l',
                'position' : [0.0, -0.25, 0.3],
                'look'     : 'down',
                'time'     : 3.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_7'
            }
            return ans
        
        # 떨어뜨리러 가는 초기 위치
        elif step == 'step_7':
            ans = {
                'frame'    : 'l',
                'position' : [0.3, 0.0, 0.2],
                'look'     : 'down',
                'time'     : 2.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_8'
            }
            return ans
        
        elif step == 'step_8':
            ans = {
                'frame'    : 'l',
                'position' : [0.3, 0.0, 0.03],
                'look'     : 'down',
                'time'     : 2.0,
                'wrist'    : 0.0,
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
                'position' : [0.32, 0.0, 0.15],
                'look'     : 'down',
                'time'     : 2.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_11'
            }
            return ans
        
        if step == 'step_11':
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