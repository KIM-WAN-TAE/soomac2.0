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
                'speed'    : 99.0,
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
                'speed' : 900.0,
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
                'next_step': 'step_7'
            }
            return ans
        
        # #이거 지금 생략되어있음
        # # 2차 이동 전 yaw만 회전
        # elif step == 'step_5':
        #     ans = {
        #         'block_pick_move'    : True,
        #         'pick_str' : 'second',
        #         'speed' : 90.0,
        #         'requires_ack' : True,
        #         'next_step': 'step_6'
        #     }
        #     return ans
        
        # # 그리퍼 석션 시작 + Block 좌표 요청
        # elif step == 'step_6':
        #     ans = {
        #         'gripper'    : True,
        #         'gripper_str' : 's',
        #         'drop_coor_order' : True,
        #         'requires_ack' : False,
        #         'next_step': 'step_7'
        #     }
        #     return ans
        
        # 블록 집으러 진입
        elif step == 'step_7':
            ans = {
                'gripper'    : True,
                'gripper_str' : 's',
                'drop_coor_order' : True,
                'block_pick_move'    : True,
                'pick_str' : 'third',
                'speed' : 650.0,
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
                'time' : 0.1, # sec 단위
                'requires_ack' : False,
                'next_step': 'step_10'
            }
            return ans
        
        # 상승
        elif step == 'step_10':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, -85.0, 0.0, 0.0, 0.0],
                'speed'    : 1000.0,
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
                'speed'    : 99.0,
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
                'position' : [0.0, 0.0, 150.0, 0.0, 0.0, 0.0],
                'speed'    : 750.0,
                # 'gripper'    : True,
                # 'gripper_str' : 'e',
                'requires_ack' : True,
                'next_step': 'step_14'
            }
            return ans
        
        elif step == 'step_14':
            ans = {
                'gripper'    : True,
                'gripper_str' : 'e',
                'wait_a_sec' : True,
                'time' : 0.1, # sec 단위
                'requires_ack' : False,
                'next_step': 'step_15'
            }
            return ans
        
        # 다시 상승 -> 이후 바로 step_1으로 이동할 수 있게
        elif step == 'step_15':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, -80.0, 0.0, 0.0, 0.0],
                'speed'    : 1000.0,
                'gripper'    : True,
                'gripper_str' : 'e',
                'requires_ack' : True,
                'next_step': 'step_1'
            }
            return ans
        
        # elif step == 'step_16':
        #     ans = {
        #         'frame'    : 'j',
        #         'position' : INIT_POSE,
        #         'speed'    : 90.0,
        #         'requires_ack' : True,
        #         'next_step': 'step_1'
        #     }
        #     return ans
        
        elif step == 'emer_1':
            ans = {
                'frame'    : 'j',
                'position' : [-131.07,  -23.71, -118.91,    0.13,  -37.60, -130.91],
                'speed'    : 60.0,
                'requires_ack' : True,
                'next_step': 'emer_2'
            }
            return ans
        
        elif step == 'emer_2':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, 42.0, 0.0, 0.0, 0.0],
                'speed'    : 300.0,
                'gripper'    : True,
                'gripper_str' : 's',
                'requires_ack' : True,
                'next_step': 'emer_3'
            }
            return ans
        
        elif step == 'emer_3':
            ans = {
                'frame'    : 'j',
                'position' : [-131.06,  -27.25, -120.54,    0.15,  -32.44, -130.93],
                'speed'    : 50.0,
                'requires_ack' : True,
                'next_step': 'emer_4'
            }
            return ans
        
        elif step == 'emer_4':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, 0.0, -30.0, 0.0, 0.0],
                'speed'    : 10.0,
                'requires_ack' : True,
                'next_step': 'emer_5'
            }
            return ans
        
        elif step == 'emer_5':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 400.0, 0.0, 0.0, 0.0, 0.0],
                'speed'    : 800.0,
                'requires_ack' : True,
                'next_step': 'emer_6'
            }
            return ans
        
        elif step == 'emer_6':
            ans = {
                'frame'    : 'j',
                'position' : [-131.06,  -27.25, -120.54,    0.15,  -32.44, -130.93],
                'speed'    : 80.0,
                'requires_ack' : True,
                'next_step': 'emer_7'
            }
            return ans
        
        elif step == 'emer_7':
            ans = {
                'gripper'    : True,
                'gripper_str' : 'e',
                'requires_ack' : False,
                'next_step': 'step_1'
            }
            return ans
        
        