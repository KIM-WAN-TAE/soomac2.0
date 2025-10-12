
class Deliver_Box:
    def step(self, step):
        # 초기 위치
        if step == 'step_1':
            ans = {
                'frame'    : 'j',
                'position' : INIT_POSE,
                'speed'    : 10.0,
                'requires_ack' : True,
                'next_step': 'step_2'
            }
            return ans
        
        # 서랍 집는 초기 위치로 진입
        elif step == 'step_2':
            ans = {
                'frame'    : 'j',
                'position' : [-151.05, 46.98, 71.70, 1.77, 59.20, 28.91],
                'speed'    : 10.0,
                'requires_ack' : True,
                'next_step': 'step_3'
            }
            return ans
        
        # 서랍 오픈 위치
        elif step == 'step_3':
            ans = {
                'frame'    : 'j',
                'position' : [-150.60,   55.35,   73.20,    0.39,   51.16,   30.11],
                'speed'    : 10.0,
                'requires_ack' : True,
                'next_step': 'step_4'
            }
            return ans
        
        # 서랍 잡기
        elif step == 'step_4':
            ans = {
                'gripper' : 'close',
                'requires_ack' : True,
                'next_step': 'step_5'
            }
            return ans
        
        # 서랍 열기
        elif step == 'step_5':
            ans = {
                'frame'    : 'j',
                'position' : [-156.69,   51.16,   83.48,    0.47,   45.11,   23.93],
                'speed'    : 5.0,
                'requires_ack' : True,
                'next_step': 'step_6'
            }
            return ans
        
        # 서랍 놓기
        elif step == 'step_6':
            ans = {
                'gripper' : 'open',
                'requires_ack' : True,
                'next_step': 'step_7'
            }
            return ans
        
        # 이제 디텍을 위한 상승
        elif step == 'step_7':
            ans = {
                'frame'    : 'j',
                'position' : [-156.74,   43.64,   80.43,    0.40,   55.67,   23.99],
                'speed'    : 10.0,
                'requires_ack' : True,
                'next_step': 'step_8'
            }
            return ans

        if step == 'step_8':
            ans = {
                'frame'    : 'j',
                'position' : [-165.93,   40.74,   86.66,    0.47,   52.40,   14.75],
                'speed'    : 10.0,
                'requires_ack' : True,
                'next_step': 'step_9'
            }
            return ans
        
        # 박스 디텍을 위한 이동
        elif step == 'step_9':
            ans = {
                'camera_trigger' : True,
                'requires_ack' : True,
                'next_step': 'step_10'
            }
            return ans
        
        # 디텍 한걸로 이동
        elif step == 'step_10':
            ans = {
                'camera_move' : True,
                'requires_ack' : True,
                'speed'       : 15.0,
                'next_step': 'step_11'
            }
            return ans
    
        # 박스 잡기를 위한 이동
        elif step == 'step_11':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, 160.0, 0.0, 0.0, 0.0],
                'speed'    : 30.0,
                'requires_ack' : True,
                'next_step': 'step_12'
            }
            return ans
        
        # 박스 집기
        elif step == 'step_12':
            ans = {
                'gripper' : 'close',
                'requires_ack' : True,
                'next_step': 'step_13'
            }
            return ans
        
        # 박스 수거 상승
        elif step == 'step_13':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, -160.0, 0.0, 0.0, 0.0],
                'speed'    : 20.0,
                'requires_ack' : True,
                'next_step': 'step_14'
            }
            return ans
        
        # 파라미터 초기화
        elif step == 'step_14':
            ans = {
                'clear' : True,
                'requires_ack' : False,
                'next_step': 'step_15'
            }
            return ans
        
        elif step == 'step_15':
            ans = {
                'frame'    : 'j',
                'position' : [-181.37,   36.53,   95.64,    0.56,   47.73,   -0.78],
                'speed'    : 10.0,
                'requires_ack' : True,
                'next_step': 'step_16'
            }
            return ans
        
        # 도구 디텍 위치
        elif step == 'step_16':
            ans = {
                'target_trigger' : True,
                'requires_ack' : True,
                'next_step': 'step_17'
            }
            return ans
        
        elif step == 'step_17':
            ans = {
                'target_move' : True,
                'requires_ack' : True,
                'speed'       : 10.0,
                'next_step': 'step_18'
            }
            return ans
        
        elif step == 'step_18':
            ans = {
                'boxbox' : True,
                'requires_ack' : True,
                'speed'       : 20.0,
                'next_step': 'step_19'
            }
            return ans
            
        elif step == 'step_19':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, 140.0, 0.0, 0.0, 0.0],
                'speed'    : 20.0,
                'requires_ack' : True,
                'next_step': 'step_20'
            }
            return ans
    
        elif step == 'step_20':
            ans = {
                'gripper' : 'open',
                'requires_ack' : True,
                'next_step': 'step_21'
            }
            return ans
        
        elif step == 'step_21':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, -110.0, 0.0, 0.0, 0.0],
                'speed'    : 20.0,
                'requires_ack' : True,
                'next_step': 'step_22'
            }
            return ans