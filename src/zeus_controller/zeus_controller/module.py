# Zeus Common Mission Module

# CAM INIT Pose
INIT_POSE = [-101.97, -17.57, -65.79, 0.18, -96.80, -101.69]
        
class Block:
    def step(self, step):
        # # 50cm 초기 카메라 Detect 위치로 이동
        # if step == 'step_0':
        #     ans = {
        #         'frame'    : 'j',
        #         'position' : INIT_POSE,
        #         'speed'    : 20.0,
        #         'requires_ack' : True,
        #         'next_step': 'step_1'
        #     }
        #     return ans
        
        # 50cm 초기 카메라 Detect 위치로 이동
        if step == 'step_1':
            ans = {
                'frame'    : 'j',
                'position' : INIT_POSE,
                'speed'    : 10.0,
                'requires_ack' : True,
                'next_step': 'step_2'
            }
            return ans
        
        # 카메라에 토픽 발생하여 좌표 수신
        elif step == 'step_2':
            ans = {
                'block_order'    : True,
                'detect_str' : 'block1',
                'requires_ack' : True,
                'next_step': 'step_3'
            }
            return ans
        
        # 1차 이동
        elif step == 'step_3':
            ans = {
                'block_pick_move'    : True,
                'pick_str' : 'first',
                'speed' : 600.0,
                'requires_ack' : True,
                'next_step': 'step_4'
            }
            return ans
        
        # 카메라에 2차 토픽 발생하여 좌표 수신
        elif step == 'step_4':
            ans = {
                'block_order'    : True,
                'detect_str' : 'block2',
                'requires_ack' : True,
                'next_step': 'step_5'
            }
            return ans
        
         # 2차 이동 전 yaw만 회전
        elif step == 'step_5':
            ans = {
                'block_pick_move'    : True,
                'pick_str' : 'second',
                'speed' : 80.0,
                'requires_ack' : True,
                'next_step': 'step_6'
            }
            return ans
        
        # 그리퍼 석션 시작 + Block 좌표 요청
        elif step == 'step_6':
            ans = {
                'gripper'    : True,
                'gripper_str' : 's',
                'drop_coor_order' : True,
                'requires_ack' : False,
                'next_step': 'step_7'
            }
            return ans
        
        # 블록 집으러 진입
        elif step == 'step_7':
            ans = {
                'block_pick_move'    : True,
                'pick_str' : 'third',
                'speed' : 200.0,
                'requires_ack' : True,
                'next_step': 'step_8'
            }
            return ans
        
        # # 코앞에서 진입
        # elif step == 'step_8':
        #     ans = {
        #         'frame'    : 't',
        #         'position' : [0.0, 0.0, 10.0, 0.0, 0.0, 0.0],
        #         'speed'    : 30.0,
        #         'requires_ack' : True,
        #         'next_step': 'step_9'
        #     }
        #     return ans
        
        # 석션기가 블록을 집기까지 대기
        elif step == 'step_8':
            ans = {
                'wait_a_sec' : True,
                'time' : 0.5, # sec 단위
                'requires_ack' : False,
                'next_step': 'step_10'
            }
            return ans
        
        # 상승
        elif step == 'step_10':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, -150.0, 0.0, 0.0, 0.0],
                'speed'    : 400.0,
                'requires_ack' : True,
                'next_step': 'step_12'
            }
            return ans
        
        # 떨어뜨리는 초기 단계로 이동 ## 수정해야 해용
        # elif step == 'step_11':
        #     ans = {
        #         'frame'    : 'j',
        #         'position' : [-15.75, -27.47, -95.23, 0.20, -57.54, -102.09],
        #         'speed'    : 25.0,
        #         'requires_ack' : True,
        #         'next_step': 'step_12'
        #     }
        #     return ans
        
        # 떨어뜨리는 Offset 위치로 이동
        elif step == 'step_12':
            ans = {
                'block_drop_move' : True,
                'speed'    : 15.0,
                # 'gripper'    : True,
                # 'gripper_str' : 'e',
                'requires_ack' : True,
                'next_step': 'step_13'
            }
            return ans
        
        # 석션기 종료와 동시에 하강
        elif step == 'step_13':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, 95.0, 0.0, 0.0, 0.0],
                'speed'    : 200.0,
                'gripper'    : True,
                'gripper_str' : 'e',
                'requires_ack' : True,
                'next_step': 'step_14'
            }
            return ans
        
        elif step == 'step_14':
            ans = {
                'wait_a_sec' : True,
                'time' : 1.0, # sec 단위
                'requires_ack' : False,
                'next_step': 'step_15'
            }
            return ans
        
        # 다시 상승 -> 이후 바로 step_1으로 이동할 수 있게
        elif step == 'step_15':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, -80.0, 0.0, 0.0, 0.0],
                'speed'    : 150.0,
                'gripper'    : True,
                'gripper_str' : 'e',
                'requires_ack' : True,
                'next_step': 'step_16'
            }
            return ans
        
        elif step == 'step_16':
            ans = {
                'frame'    : 'j',
                'position' : INIT_POSE,
                'speed'    : 30.0,
                'requires_ack' : True,
                'next_step': 'step_1'
            }
            return ans