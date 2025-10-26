INIT_POSE = {
    'pose' : [0.0, 0.0, 0.0],
    'posture' : 'down',
    'wrist' : 0.0
}

class Test:
    def step(self, step):    
        if step == 'step_1':
            ans = {
                'position' : [0.0, -0.3, 0.2],
                'look'     : 'down',
                'time'     : 3.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_2'
            }
            return ans
        
        elif step == 'step_2':
            ans = {
                'position' : [0.25, 0.0, 0.25],
                'look'     : 'down',
                'time'     : 3.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_3'
            }
            return ans
        
        elif step == 'step_3':
            ans = {
                'camera_trigger' : True,
                'requires_ack' : True,
                'next_step' : 'step_4'
            }
            return ans
        
        elif step == 'step_4':
            ans = {
                'camera_move' : True,
                'time'       : 5.0,
                'look'       : 'down',
                'requires_ack' : True,
                'next_step': 'step_5'
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
                'position' : [0.3, 0.0, 0.2],
                'look'     : 'down',
                'time'     : 3.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_7'
            }
            return ans
        
        elif step == 'step_7':
            ans = {
                'position' : [0.0, -0.3, 0.2],
                'look'     : 'down',
                'time'     : 3.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_8'
            }
            return ans
        
        elif step == 'step_8':
            ans = {
                'position' : [0.0, -0.3, 0.04],
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
        
        if step == 'step_10':
            ans = {
                'position' : [0.0, -0.3, 0.2],
                'look'     : 'down',
                'time'     : 1.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'none'
            }
            return ans
        
class Deliver_Normal:
    def step(self, step):
        if step == 'step_1':
            ans = {
                'position' : INIT_POSE['pose'],
                'look'     : INIT_POSE['posture'],
                'time'     : 1.0,
                'wrist'    : INIT_POSE['wrist'],
                'requires_ack' : True,
                'next_step' : 'step_2'
            }
            return ans
        # 카메라 Detect Pose 좌표 따야함
        elif step == 'step_2':
            ans = {
                'position' : [0.25, 0.0, 0.25],
                'look'     : 'down',
                'time'     : 3.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_3'
            }
            return ans
        
        # 카메라 좌표 요청
        elif step == 'step_3':
            ans = {
                'camera_trigger' : True,
                'requires_ack' : True,
                'next_step' : 'step_4'
            }
            return ans
        
        # 카메가 준 좌표 중앙으로 이동
        elif step == 'step_4':
            ans = {
                'camera_move' : True,
                'time'       : 5.0,
                'look'       : 'down',
                'requires_ack' : True,
                'next_step': 'step_5'
            }
            return ans
        
        # 진입 함수
        elif step == 'step_5':
            ans = {
                'move_only_one_axis' : [0.0, 0.0, 0.0],
                'look'     : 'down',
                'time'     : 3.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_6'
            }
            return ans
        
        # 그러퍼 잡기
        elif step == 'step_6':
            ans = {
                'gripper' : 'close',
                'requires_ack' : True,
                'next_step': 'step_7'
            }
            return ans
        
        # 상승 -> 상승 함수 만들어야 할듯, 현 위치를 추종해서 해당 위치에서 Z 값 상승하는 것
        elif step == 'step_7':
            ans = {
                'move_only_one_axis' : [0.0, 0.0, 0.0],
                'look'     : 'down',
                'time'     : 3.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_8'
            }
            return ans
        
        # 떨어뜨릴 위치 상부로 이동
        elif step == 'step_8':
            ans = {
                'position' : [0.0, -0.3, 0.2],
                'look'     : 'down',
                'time'     : 3.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_9'
            }
            return ans
        
        # 떨어뜨리러 하강
        elif step == 'step_9':
            ans = {
                'move_only_one_axis' : [0.0, 0.0, 0.0],
                'look'     : 'down',
                'time'     : 3.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_10'
            }
            return ans
        
        # 그리퍼 오픈
        elif step == 'step_10':
            ans = {
                'gripper' : 'open',
                'requires_ack' : True,
                'next_step': 'step_11'
            }
            return ans
        
        # 다시 상승 -> 이것도 상승 함수 쓰면 될 듯
        elif step == 'step_11':
            ans = {
                'move_only_one_axis' : [0.0, 0.0, 0.0],
                'look'     : 'down',
                'time'     : 3.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_12'
            }
            return ans
        
        elif step == 'step_12':
            ans = {
                'position' : INIT_POSE['pose'],
                'look'     : INIT_POSE['posture'],
                'time'     : 1.0,
                'wrist'    : INIT_POSE['wrist'],
                'requires_ack' : True,
                'next_step' : 'None'
            }
            return ans
        
class START:
    def step(self, step):
        if step == 'step_1':
            ans = {
                'position' : INIT_POSE['pose'],
                'look'     : INIT_POSE['posture'],
                'time'     : 1.0,
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
        
        # 수직 위치 진입
        elif step == 'step_3':
            ans = {
                'move_only_one_axis' : [0.0, 0.0, 0.0],
                'look'     : 'down',
                'time'     : 3.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_4'
            }
            return ans
        
        # 수직으로 진입하는 함수
        elif step == 'step_4':
            ans = {
                'position' : [0.0, -0.3, 0.04],
                'look'     : 'down',
                'time'     : 2.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_5'
            }
            return ans
        
        # 수직으로 퇴장
        elif step == 'step_5':
            ans = {
                'move_only_one_axis' : [0.0, 0.0, 0.0],
                'look'     : 'down',
                'time'     : 3.0,
                'wrist'    : 0.0,
                'requires_ack' : True,
                'next_step' : 'step_6'
            }
            return ans
        
        # 초기화 위치로 이동
        elif step == 'step_6':
            ans = {
                'position' : INIT_POSE['pose'],
                'look'     : INIT_POSE['posture'],
                'time'     : 1.0,
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