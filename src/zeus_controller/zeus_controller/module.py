# Zeus Free Mission Module

# Init Pose Joint Coordinate
INIT_POSE = [-170.55,  -19.30,  147.09,   -0.30,   54.12,   98.71]
        
class Test:
    def step(self, step):
        if step == 'step_1':
            ans = {
                'frame'    : 'j',
                'position' : INIT_POSE,
                'speed'    : 10.0,
                'requires_ack' : True,
                'next_step': 'step_2'
            }
            return ans
        # 열기 위치로 이동    
        elif step == 'step_2':
            ans = {
                'frame'    : 'j',
                'position' : [-144.80,   42.52,   93.16,    0.09,   44.89,   33.89],
                'speed'    : 5.0,
                'requires_ack' : True,
                'next_step': 'step_3'
            }
            return ans
        # 손잡이 위치로 이동
        elif step == 'step_3':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, 100.0, 0.0, 0.0, 0.0],
                'speed'    : 10.0,
                'requires_ack' : True,
                'next_step': 'step_4'
            }
            return ans
        # 손잡이 잡기
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
                'frame'    : 't',
                'position' : [0.0, -120.0, 0.0, 0.0, 0.0, 0.0],
                'speed'    : 10.0,
                'requires_ack' : True,
                'next_step': 'step_6'
            }
            return ans
        # 손잡이 놓기
        elif step == 'step_6':
            ans = {
                'gripper' : 'open',
                'requires_ack' : True,
                'next_step': 'step_7'
            }
            return ans
        # 약간의 상승
        elif step == 'step_7':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, -50.0, 0.0, 0.0, 0.0],
                'speed'    : 10.0,
                'requires_ack' : True,
                'next_step': 'step_8'
            }
            return ans
        # Detect 위치로 이동
        elif step == 'step_8':
            ans = {
                'frame'    : 'j',
                'position' : [-155.06,   31.34,   95.49,   -0.05,   53.74,   23.72],
                'speed'    : 5.0,
                'requires_ack' : True,
                'next_step': 'None'
            }
            return ans
        
        else:
            print('[ZEUS] Wrong Step')
            return None

class Deliver_Normal:
    def step(self, step):
        if step == 'step_1':
            ans = {
                'frame'    : 'j',
                'position' : INIT_POSE,
                'speed'    : 15.0,
                'requires_ack' : True,
                'next_step': 'step_2'
            }
            return ans
            
        elif step == 'step_2':
            ans = {
                'frame'    : 'j',
                'position' : [-176.12,   52.47,  112.57,   97.36,   88.01,  -75.26],
                'speed'    : 30.0,
                'requires_ack' : True,
                'next_step': 'step_3'
            }
            return ans
        
        elif step == 'step_3':
            ans = {
                'camera_trigger' : True,
                'requires_ack' : True,
                'next_step': 'step_4'
            }
            return ans
        
        elif step == 'step_4':
            ans = {
                'camera_move' : True,
                'requires_ack' : True,
                'speed'       : 200.0,
                'next_step': 'step_offset'
            }
            return ans
            
        elif step == 'step_offset':
            ans = {
                'front_offset_move' : True,
                'requires_ack' : True,
                'next_step': 'step_5'
            }
            return ans
        
        elif step == 'step_5':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, 80.0, 0.0, 0.0, 0.0],
                'speed'    : 30.0,
                'requires_ack' : True,
                'next_step': 'step_6'
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
                'frame'    : 't',
                'position' : [0.0, 70.0, 0.0, 0.0, 0.0, 0.0],
                'speed'    : 80.0,
                'requires_ack' : True,
                'next_step': 'step_8'
            }
            return ans

        elif step == 'step_8':
            ans = {
                'deliver_offset_move'    : True,
                'speed'    : 80.0,
                'requires_ack' : True,
                'next_step': 'step_9'
            }
            return ans
        
        elif step == 'step_9':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, -170.0, 0.0, 0.0, 0.0],
                'speed'    : 300.0,
                'requires_ack' : True,
                'next_step': 'step_10'
            }
            return ans

        elif step == 'step_10':
            ans = {
                'frame'    : 'j',
                'position' : [-173.58,   38.03,   96.39,   -0.12,   45.97,    5.69],
                'speed'    : 17.5,
                'requires_ack' : True,
                'next_step': 'step_11'
            }
            return ans
        
        elif step == 'step_11':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, 150.0, 0.0, 0.0, 0.0],
                'speed'    : 75.0,
                'requires_ack' : True,
                'next_step': 'step_12'
            }
            return ans
        
        elif step == 'step_12':
            ans = {
                'gripper' : 'open',
                'requires_ack' : True,
                'next_step': 'step_13'
            }
            return ans
        
        elif step == 'step_13':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, -50.0, 0.0, 0.0, 0.0],
                'speed'    : 300.0,
                'requires_ack' : True,
                'next_step': 'step_14'
            }
            return ans
        
        elif step == 'step_14':
            ans = {
                'frame'    : 'j',
                'position' : INIT_POSE,
                'speed'    : 40.0,
                'requires_ack' : True,
                'next_step': 'None'
            }
            return ans

SWITCH_ON_INIT_POSE = [-129.18,   27.07,  120.32,  135.08,   65.49,  -67.26]     
class Start:
    def step(self, step):
        # 혹시 모를 초기화 위치
        if step == 'step_1':
            ans = {
                'frame'    : 'j',
                'position' : INIT_POSE,
                'speed'    : 5.0,
                'requires_ack' : True,
                'next_step': 'step_2'
            }
            return ans
        
        elif step == 'step_2':
            ans = {
                'gripper' : 'close',
                'requires_ack' : False,
                'next_step': 'step_3'
            }
            return ans
        
        elif step == 'step_3':
            ans = {
                'frame'    : 'j',
                'position' : SWITCH_ON_INIT_POSE,
                'speed'    : 40.0,
                'requires_ack' : True,
                'next_step': 'step_4'
            }
            return ans
        
        elif step == 'step_4':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, 25.0, 0.0, 0.0, 0.0],
                'speed'    : 45.0,
                'requires_ack' : True,
                'next_step': 'step_5'
            }
            return ans
        
        # elif step == 'step_4':
        #     ans = {
        #         'frame'    : 't',
        #         'position' : [0.0, 0.0, -100.0, 0.0, 0.0, 0.0],
        #         'speed'    : 50.0,
        #         'requires_ack' : True,
        #         'next_step': 'step_5'
        #     }
        #     return ans
        
        elif step == 'step_5':
            ans = {
                'frame'    : 'j',
                'position' : INIT_POSE,
                'speed'    : 30.0,
                'requires_ack' : True,
                'next_step': 'step_6'
            }
            return ans
        
        elif step == 'step_6':
            ans = {
                'gripper' : 'open',
                'requires_ack' : True,
                'next_step': 'None'
            }
            return ans

SWITCH_OFF_INIT_POSE = [-126.35,   25.52,  122.68,  138.23,   65.04,  -69.08]        
class Finish:
    def step(self, step):
        # 혹시 모를 초기화 위치
        if step == 'step_1':
            ans = {
                'frame'    : 'j',
                'position' : INIT_POSE,
                'speed'    : 5.0,
                'requires_ack' : True,
                'next_step': 'step_2'
            }
            return ans
        
        elif step == 'step_2':
            ans = {
                'gripper' : 'close',
                'requires_ack' : False,
                'next_step': 'step_3'
            }
            return ans
        
        elif step == 'step_3':
            ans = {
                'frame'    : 'j',
                'position' : SWITCH_OFF_INIT_POSE,
                'speed'    : 40.0,
                'requires_ack' : True,
                'next_step': 'step_4'
            }
            return ans
        
        elif step == 'step_4':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, 25.0, 0.0, 0.0, 0.0],
                'speed'    : 45.0,
                'requires_ack' : True,
                'next_step': 'step_5'
            }
            return ans
        
        # elif step == 'step_4':
        #     ans = {
        #         'frame'    : 't',
        #         'position' : [0.0, 0.0, -100.0, 0.0, 0.0, 0.0],
        #         'speed'    : 50.0,
        #         'requires_ack' : True,
        #         'next_step': 'step_5'
        #     }
        #     return ans
        
        elif step == 'step_5':
            ans = {
                'frame'    : 'j',
                'position' : INIT_POSE,
                'speed'    : 30.0,
                'requires_ack' : True,
                'next_step': 'step_6'
            }
            return ans
        
        elif step == 'step_6':
            ans = {
                'gripper' : 'open',
                'requires_ack' : True,
                'next_step': 'None'
            }
            return ans
        
BOX_DETECT_POSE = [-155.06,   31.34,   95.49,   -0.05,   53.74,   23.72]
class Deliver_Box:
    def step(self, step):
        if step == 'step_1':
            ans = {
                'frame'    : 'j',
                'position' : INIT_POSE,
                'speed'    : 10.0,
                'requires_ack' : True,
                'next_step': 'step_2'
            }
            return ans
        # 열기 위치로 이동    
        elif step == 'step_2':
            ans = {
                'frame'    : 'j',
                'position' : [-144.80,   42.52,   93.16,    0.09,   44.89,   33.89],
                'speed'    : 20.0,
                'requires_ack' : True,
                'next_step': 'step_3'
            }
            return ans
        # 손잡이 위치로 이동
        elif step == 'step_3':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, 100.0, 0.0, 0.0, 0.0],
                'speed'    : 70.0,
                'requires_ack' : True,
                'next_step': 'step_4'
            }
            return ans
        # 손잡이 잡기
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
                'frame'    : 't',
                'position' : [0.0, -120.0, 0.0, 0.0, 0.0, 0.0],
                'speed'    : 50.0,
                'requires_ack' : True,
                'next_step': 'step_6'
            }
            return ans
        # 손잡이 놓기
        elif step == 'step_6':
            ans = {
                'gripper' : 'open',
                'requires_ack' : True,
                'next_step': 'step_7'
            }
            return ans
        # 약간의 상승
        elif step == 'step_7':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, -50.0, 0.0, 0.0, 0.0],
                'speed'    : 120.0,
                'requires_ack' : True,
                'next_step': 'step_8'
            }
            return ans
        # Detect 위치로 이동
        elif step == 'step_8':
            ans = {
                'frame'    : 'j',
                'position' : BOX_DETECT_POSE,
                'speed'    : 15.0,
                'requires_ack' : True,
                'next_step': 'step_10'
            }
            return ans
        
        # 박스 디텍을 위한 이동
        elif step == 'step_10':
            ans = {
                'camera_trigger' : True,
                'requires_ack' : True,
                'next_step': 'step_11'
            }
            return ans
        
        # 디텍 한걸로 이동
        elif step == 'step_11':
            ans = {
                'box_camera_move' : True,
                'requires_ack' : True,
                'speed'       : 70.0,
                'next_step': 'step_12'
            }
            return ans
    
        # 박스 잡기를 위한 이동
        elif step == 'step_12':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, 85.0, 0.0, 0.0, 0.0],
                'speed'    : 50.0,
                'requires_ack' : True,
                'next_step': 'step_13'
            }
            return ans

        elif step == 'step_13':
            ans = {
                'gripper' : 'close',
                'requires_ack' : False,
                'next_step': 'step_14'
            }
            return ans
        
        elif step == 'step_14':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, -105.0, 0.0, 0.0, 0.0],
                'speed'    : 100.0,
                'requires_ack' : True,
                'next_step': 'step_15'
            }
            return ans
    
        # 파라미터 초기화
        elif step == 'step_15':
            ans = {
                'clear' : True,
                'requires_ack' : False,
                'next_step': 'step_16'
            }
            return ans
        
        elif step == 'step_16':
            ans = {
                'frame'    : 'j',
                'position' : [-181.37,   36.53,   95.64,    0.56,   47.73,   -0.78],
                'speed'    : 20.0,
                'requires_ack' : True,
                'next_step': 'step_17'
            }
            return ans
        
        # 도구 디텍 위치
        elif step == 'step_17':
            ans = {
                'target_trigger' : True,
                'requires_ack' : True,
                'next_step': 'step_18'
            }
            return ans
        
        elif step == 'step_18':
            ans = {
                'target_move' : True,
                'requires_ack' : True,
                'speed'       : 150.0,
                'next_step': 'step_19'
            }
            return ans
        
        elif step == 'step_19':
            ans = {
                'chat_trigger' : True,
                'requires_ack' : True,
                'next_step': 'step_20'
            }
            return ans
        
        elif step == 'step_20':
            ans = {
                'boxbox' : True,
                'requires_ack' : True,
                'speed'       : 200.0,
                'next_step': 'step_21'
            }
            return ans
            
        elif step == 'step_21':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, 145.0, 0.0, 0.0, 0.0],
                'speed'    : 65.0,
                'requires_ack' : True,
                'next_step': 'step_22'
            }
            return ans
    
        elif step == 'step_22':
            ans = {
                'gripper' : 'open',
                'requires_ack' : True,
                'next_step': 'step_23'
            }
            return ans
        
        elif step == 'step_23':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, -110.0, 0.0, 0.0, 0.0],
                'speed'    : 150.0,
                'requires_ack' : True,
                'next_step': 'step_24'
            }
            return ans
        
        # 서랍 닫기 전 init
        elif step == 'step_24':
            ans = {
                'frame'    : 'j',
                'position' : [-155.77,   36.56,  106.69,   -0.08,   37.32,   23.05],
                'speed'    : 20.0,
                'requires_ack' : True,
                'next_step': 'step_25'
            }
            return ans 
        
        # 열린 서랍 손잡이 위치로 이동
        elif step == 'step_25':
            ans = {
                'frame'    : 'j',
                'position' : [-155.78,   47.05,  108.43,   -0.11,   25.09,   23.07],
                'speed'    : 7.0,
                'requires_ack' : True,
                'next_step': 'step_26'
            }
            return ans 
        # 손잡이 잡기
        elif step == 'step_26':
            ans = {
                'gripper' : 'close',
                'requires_ack' : True,
                'next_step': 'step_27'
            }
            return ans
        # 닫기
        elif step == 'step_27':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 120.0, 0.0, 0.0, 0.0, 0.0],
                'speed'    : 50.0,
                'requires_ack' : True,
                'next_step': 'step_28'
            }
            return ans
        
        # 손잡이 놓기
        elif step == 'step_28':
            ans = {
                'gripper' : 'open',
                'requires_ack' : True,
                'next_step': 'step_29'
            }
            return ans
        # 약간의 상승
        elif step == 'step_29':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, -60.0, 0.0, 0.0, 0.0],
                'speed'    : 120.0,
                'requires_ack' : True,
                'next_step': 'step_30'
            }
            return ans
        # 초기화
        if step == 'step_30':
            ans = {
                'frame'    : 'j',
                'position' : INIT_POSE,
                'speed'    : 20.0,
                'requires_ack' : True,
                'next_step': 'None'
            }
            return ans
        
GRIP_INIT_POSE = [-162.44,    2.35,  104.30, -225.75,   22.97,  313.07]
class Down:
    def step(self, step):
        
        # 초기화
        if step == 'step_1':
            ans = {
                'frame'    : 'j',
                'position' : INIT_POSE,
                'speed'    : 10.0,
                'requires_ack' : True,
                'next_step': 'step_2'
            }
            return ans
        
        # 잡기 전 자세로 진입
        elif step == 'step_2':
            ans = {
                'frame'    : 'j',
                'position' : GRIP_INIT_POSE,
                'speed'    : 30.0,
                'requires_ack' : True,
                'next_step': 'step_3'
            }
            return ans
        
        # 잡으러 진입
        elif step == 'step_3':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, 200.0, 0.0, 0.0, 0.0],
                'speed'    : 120.0,
                'requires_ack' : True,
                'next_step': 'step_4'
            }
            return ans
        
        # 잡기
        elif step == 'step_4':
            ans = {
                'gripper' : 'close',
                'requires_ack' : True,
                'next_step': 'step_5'
            }
            return ans
        
        # 잡고 하강
        elif step == 'step_5':
            ans = {
                'frame'    : 't',
                'position' : [0.0, -50.0, 0.0, 0.0, 0.0, 0.0],
                'speed'    : 20.0,
                'requires_ack' : True,
                'next_step': 'step_6'
            }
            return ans
        
        # 놓기
        elif step == 'step_6':
            ans = {
                'gripper' : 'open',
                'requires_ack' : True,
                'next_step': 'step_7'
            }
            return ans
        
        elif step == 'step_7':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 50.0, 0.0, 0.0, 0.0, 0.0],
                'speed'    : 80.0,
                'requires_ack' : True,
                'next_step': 'step_8'
            }
            return ans
        
        # 퇴장
        elif step == 'step_8':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, -200.0, 0.0, 0.0, 0.0],
                'speed'    : 150.0,
                'requires_ack' : True,
                'next_step': 'step_9'
            }
            return ans
        
        elif step == 'step_9':
            ans = {
                'frame'    : 'j',
                'position' : INIT_POSE,
                'speed'    : 30.0,
                'requires_ack' : True,
                'next_step': 'None'
            }
            return ans

LOOK_MY_BOARD = [-166.14,   49.02,   72.38,   -0.00,   58.60,   17.62]
MEASURE_INIT = [-150.44,   51.97,  101.52, -180.93,   63.37,  270.33]
MEASURE_GRIP = [-150.14,   65.44,   80.89, -180.64,   56.20,  270.28]
MEASURE_TIP_TOP = [-150.09,   58.01,   83.34, -180.61,   51.22,  270.30]
SOUND = [-142.91,   58.44,   90.01, -172.08,   58.54,  265.65]
NONE_SOUND = [-143.19,   57.11,   94.08, -172.70,   61.03,  266.25]
class Measure:
    def step(self, step):
        # 초기화
        if step == 'step_1':
            ans = {
                'frame'    : 'j',
                'position' : LOOK_MY_BOARD,
                'speed'    : 20.0,
                'requires_ack' : True,
                'next_step': 'step_2'
            }
            return ans
        
        # 잡기 init 좌표로 이동
        elif step == 'waiting':
            ans = {
                'wait_a_sec'    : 3,
                'requires_ack' : True,
                'next_step': 'step_2'
            }
            return ans 
        
        # 잡기 init 좌표로 이동
        elif step == 'step_2':
            ans = {
                'frame'    : 'j',
                'position' : MEASURE_INIT,
                'speed'    : 20.0,
                'requires_ack' : True,
                'next_step': 'step_3'
            }
            return ans
        
        # 잡는 좌표로 이동
        elif step == 'step_3':
            ans = {
                'frame'    : 'j',
                'position' : MEASURE_GRIP,
                'speed'    : 15.0,
                'requires_ack' : True,
                'next_step': 'step_4'
            }
            return ans
        
        # 잡기
        elif step == 'step_4':
            ans = {
                'gripper' : 'close',
                'requires_ack' : True,
                'next_step': 'step_5'
            }
            return ans
        
        # 잡고 상승
        elif step == 'step_5':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 80.0, 0.0, 0.0, 0.0, 0.0],
                'speed'    : 80.0,
                'requires_ack' : True,
                'next_step': 'step_6'
            }
            return ans
        
        # 소리 나는 좌표 위로 이동
        elif step == 'step_6':
            ans = {
                'frame'    : 'j',
                'position' : SOUND,
                'speed'    : 10.0,
                'requires_ack' : True,
                'next_step': 'step_7'
            }
            return ans
        
        # 하강
        elif step == 'step_7':
            ans = {
                'frame'    : 't',
                'position' : [0.0, -50.0, 0.0, 0.0, 0.0, 0.0],
                'speed'    : 12.0,
                'requires_ack' : True,
                'next_step': 'step_8'
            }
            return ans
        
        # 검침 결과 음성으로 발송
        elif step == 'step_8':
            ans = {
                'talking_measure_result' : True,
                'result' : 'sound',
                'requires_ack' : False,
                'next_step': 'step_9'
            }
            return ans
        
        # 상승
        elif step == 'step_9':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 50.0, 0.0, 0.0, 0.0, 0.0],
                'speed'    : 50.0,
                'requires_ack' : True,
                'next_step': 'step_10'
            }
            return ans
        
        # 소리 안나는 좌표로 이동
        elif step == 'step_10':
            ans = {
                'frame'    : 'j',
                'position' : NONE_SOUND,
                'speed'    : 10.0,
                'requires_ack' : True,
                'next_step': 'step_11'
            }
            return ans
        
        # 하강
        elif step == 'step_11':
            ans = {
                'frame'    : 't',
                'position' : [0.0, -50.0, 0.0, 0.0, 0.0, 0.0],
                'speed'    : 12.0,
                'requires_ack' : True,
                'next_step': 'step_12'
            }
            return ans
        
        # 검침 결과 음성으로 발송
        elif step == 'step_12':
            ans = {
                'talking_measure_result' : True,
                'result' : 'no_sound',
                'requires_ack' : False,
                'next_step': 'step_13'
            }
            return ans
        
        # 상승
        elif step == 'step_13':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 50.0, 0.0, 0.0, 0.0, 0.0],
                'speed'    : 50.0,
                'requires_ack' : True,
                'next_step': 'step_14'
            }
            return ans
        
        # 팁 반납하는 위로 이동
        elif step == 'step_14':
            ans = {
                'frame'    : 'j',
                'position' : MEASURE_TIP_TOP,
                'speed'    : 15.0,
                'requires_ack' : True,
                'next_step': 'step_15'
            }
            return ans
        
        # 팁 두러 하강
        elif step == 'step_15':
            ans = {
                'frame'    : 't',
                'position' : [0.0, -60.0, 0.0, 0.0, 0.0, 0.0],
                'speed'    : 20.0,
                'requires_ack' : True,
                'next_step': 'step_16'
            }
            return ans
        
        elif step == 'step_16':
            ans = {
                'gripper' : 'open',
                'requires_ack' : True,
                'next_step': 'step_17'
            }
            return ans
        
         # 잡기 init 좌표로 이동
        elif step == 'step_17':
            ans = {
                'frame'    : 'j',
                'position' : MEASURE_INIT,
                'speed'    : 15.0,
                'requires_ack' : True,
                'next_step': 'step_18'
            }
            return ans
        
        elif step == 'step_18':
            ans = {
                'frame'    : 'j',
                'position' : INIT_POSE,
                'speed'    : 30.0,
                'requires_ack' : True,
                'next_step': 'None'
            }
            return ans
    
TOOL_AND_BOX_DETECT = [-184.02,   24.04,   93.32,   -0.17,   63.00,   -4.75]
class Return_All:
    def step(self, step):
        # 일반 도구가 있는 경우라고 무조건 가정
        if step == 'step_1':
            ans = {
                'frame'    : 'j',
                'position' : TOOL_AND_BOX_DETECT,
                'speed'    : 35.0,
                'requires_ack' : True,
                'next_step': 'step_2'
            }
            return ans
        
        # 필요한 값을 카메라 노드에 정확히 요청
        elif step == 'step_2':
            ans = {
                'all_return_camera_trigger' : True,
                'requires_ack' : True,
                'next_step': 'step_3'
            }
            return ans
        
        # 도구 중앙으로 이동
        elif step == 'step_3':
            ans = {
                'all_return_camera_center' : True,
                'requires_ack' : True,
                'speed'       : 200.0,
                'next_step': 'all_return_offset_step'
            }
            return ans
        
        elif step == 'all_return_offset_step':
            ans = {
                'all_return_offset_move' : True,
                'requires_ack' : True,
                'next_step': 'step_4'
            }
            return ans
        
        # 집이러 내려가기
        elif step == 'step_4':
            ans = {
                'all_return_camera_move' : True,
                'requires_ack' : True,
                'speed'       : 40.0,
                'next_step': 'step_5'
            }
            return ans
        
        
        # 잡기
        elif step == 'step_5':
            ans = {
                'gripper' : 'close',
                'requires_ack' : True,
                'next_step': 'step_6'
            }
            return ans
        
        # 상승
        elif step == 'step_6':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, -80.0, 0.0, 0.0, 0.0],
                'speed'    : 80.0,
                'requires_ack' : True,
                'next_step': 'step_7'
            }
            return ans
        
        # 도구 두러 가는 위치
        elif step == 'step_7':
            ans = {
                'all_return_tool_offset' : True,
                'speed'    : 20.0,
                'requires_ack' : True,
                'next_step': 'step_8'
            }
            return ans
        
        elif step == 'step_8':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, 105.0, 0.0, 0.0, 0.0],
                'speed'    : 80.0,
                'requires_ack' : True,
                'next_step': 'step_9'
            }
            return ans
        
        # 여기서 dict 에 존재하는 첫번째 도구 정보 삭제
        elif step == 'step_9':
            ans = {
                'gripper' : 'open',
                'all_return_pop_dict' : True,
                'requires_ack' : True,
                'next_step': 'step_10'
            }
            return ans
        
        elif step == 'step_10':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, -100.0, 0.0, 0.0, 0.0],
                'speed'    : 150.0,
                'requires_ack' : True,
                'next_step': 'check'
            }
            return ans

        ### =============== 여기까지가 박스 반환 ==========================
        
        # 일단 디텍 위치로 이동
        elif step == 'M3_step_1':
            ans = {
                'frame'    : 'j',
                'position' : TOOL_AND_BOX_DETECT,
                'speed'    : 35.0,
                'requires_ack' : True,
                'next_step': 'M3_step_2'
            }
            return ans
        
        # 열기 위치로 이동    
        elif step == 'M3_step_2':
            ans = {
                'frame'    : 'j',
                'position' : [-144.80,   42.52,   93.16,    0.09,   44.89,   33.89],
                'speed'    : 25.0,
                'requires_ack' : True,
                'next_step': 'M3_step_3'
            }
            return ans
        
        # 손잡이 위치로 이동
        elif step == 'M3_step_3':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, 100.0, 0.0, 0.0, 0.0],
                'speed'    : 80.0,
                'requires_ack' : True,
                'next_step': 'M3_step_4'
            }
            return ans
        # 손잡이 잡기
        elif step == 'M3_step_4':
            ans = {
                'gripper' : 'close',
                'requires_ack' : True,
                'next_step': 'M3_step_5'
            }
            return ans
        # 서랍 열기
        elif step == 'M3_step_5':
            ans = {
                'frame'    : 't',
                'position' : [0.0, -120.0, 0.0, 0.0, 0.0, 0.0],
                'speed'    : 50.0,
                'requires_ack' : True,
                'next_step': 'M3_step_6'
            }
            return ans
        # 손잡이 놓기
        elif step == 'M3_step_6':
            ans = {
                'gripper' : 'open',
                'requires_ack' : True,
                'next_step': 'M3_step_7'
            }
            return ans
        # 약간의 상승
        elif step == 'M3_step_7':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, -50.0, 0.0, 0.0, 0.0],
                'speed'    : 150.0,
                'requires_ack' : True,
                'next_step': 'M3_step_8'
            }
            return ans
        
        # 여기서 이제 박스를 디텍하러 가야함
        # 종합 디텍 위치로 이동
        elif step == 'M3_step_8':
            ans = {
                'frame'    : 'j',
                'position' : TOOL_AND_BOX_DETECT,
                'speed'    : 20.0,
                'requires_ack' : True,
                'next_step': 'M3_step_9'
            }
            return ans
        
        # 박스 좌표 요청
        elif step == 'M3_step_9':
            ans = {
                'all_return_m3_camera_trigger' : True,
                'requires_ack' : True,
                'next_step': 'M3_step_10'
            }
            return ans
        
        # 박스 좌표로 이동
        elif step == 'M3_step_10':
            ans = {
                'all_return_m3_box_camera_move' : True,
                'requires_ack' : True,
                'speed'       : 150.0,
                'next_step': 'M3_step_11'
            }
            return ans
    
        # 박스 잡기를 위한 이동
        elif step == 'M3_step_11':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, 95.0, 0.0, 0.0, 0.0],
                'speed'    : 60.0,
                'requires_ack' : True,
                'next_step': 'M3_step_12'
            }
            return ans

        elif step == 'M3_step_12':
            ans = {
                'gripper' : 'close',
                'requires_ack' : True,
                'next_step': 'M3_step_13'
            }
            return ans
        # 박스를 집고 상승
        elif step == 'M3_step_13':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, -100.0, 0.0, 0.0, 0.0],
                'speed'    : 120.0,
                'requires_ack' : True,
                'next_step': 'M3_step_14'
            }
            return ans
        
        # 상자 드랍 상부 위치로 이동
        elif step == 'M3_step_14':
            ans = {
                'all_return_m3_offset' : True,
                'speed'    : 12.0,
                'requires_ack' : True,
                'next_step': 'M3_step_15'
            }
            return ans 
        
        # 상자 두러 하강
        elif step == 'M3_step_15':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, 100.0, 0.0, 0.0, 0.0],
                'speed'    : 60.0,
                'requires_ack' : True,
                'next_step': 'M3_step_16'
            }
            return ans 
        
        # 상자 두기
        elif step == 'M3_step_16':
            ans = {
                'gripper' : 'open',
                'requires_ack' : True,
                'next_step': 'M3_step_17'
            }
            return ans
        
        # 상자 두고 상승
        elif step == 'M3_step_17':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, -100.0, 0.0, 0.0, 0.0],
                'speed'    : 150.0,
                'requires_ack' : True,
                'next_step': 'M3_step_18'
            }
            return ans 
        
        # 서랍 손잡이 진입 전
        elif step == 'M3_step_18':
            ans = {
                'frame'    : 'j',
                'position' : [-155.77,   36.56,  106.69,   -0.08,   37.32,   23.05],
                'speed'    : 15.0,
                'requires_ack' : True,
                'next_step': 'M3_step_19'
            }
            return ans 
        
        # 열린 서랍 손잡이 위치로 이동
        elif step == 'M3_step_19':
            ans = {
                'frame'    : 'j',
                'position' : [-155.78,   47.05,  108.43,   -0.11,   25.09,   23.07],
                'speed'    : 5.0,
                'requires_ack' : True,
                'next_step': 'M3_step_20'
            }
            return ans 
        # 손잡이 잡기
        elif step == 'M3_step_20':
            ans = {
                'gripper' : 'close',
                'requires_ack' : True,
                'next_step': 'M3_step_21'
            }
            return ans
        # 닫기
        elif step == 'M3_step_21':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 120.0, 0.0, 0.0, 0.0, 0.0],
                'speed'    : 50.0,
                'requires_ack' : True,
                'next_step': 'M3_step_22'
            }
            return ans
        
        # 손잡이 놓기
        elif step == 'M3_step_22':
            ans = {
                'gripper' : 'open',
                'requires_ack' : True,
                'next_step': 'M3_step_23'
            }
            return ans
        # 약간의 상승
        elif step == 'M3_step_23':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, -100.0, 0.0, 0.0, 0.0],
                'speed'    : 120.0,
                'all_return_pop_dict' : True,
                'requires_ack' : True,
                'next_step': 'final'
            }
            return ans
        
        ### =============== 여기까지가 박스 반환 ==========================

        # 최종 종료
        elif step == 'final':
            ans = {
                'frame'    : 'j',
                'position' : INIT_POSE,
                'speed'    : 30.0,
                'requires_ack' : True,
                'next_step': 'None'
            }
            return ans