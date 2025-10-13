# Zeus Common Mission Module

# CAM INIT Pose
INIT_POSE = [-101.97, -17.57, -65.79, 0.18, -96.80, -101.69]
        
class Block:
    def step(self, step):
        
        # 50cm 초기 카메라 Detect 위치로 이동
        if step == 'step_1':
            ans = {
                'frame'    : 'j',
                'position' : INIT_POSE,
                'speed'    : 5.0,
                'requires_ack' : True,
                'next_step': 'None'
            }
            return ans
        
        # 카메라에 토픽 발생하여 좌표 수신
        elif step == 'step_2':
            ans = {
                'block_order'    : True,
                'detect_str' : 'block1',
                'requires_ack' : True,
                'next_step': 'None'
            }
            return ans
        
        # 1차 이동
        elif step == 'step_3':
            ans = {
                'block_pick_move'    : True,
                'pick_str' : 'first',
                'speed' : 10.0,
                'requires_ack' : True,
                'next_step': 'None'
            }
            return ans
        
        # 카메라에 2차 토픽 발생하여 좌표 수신
        elif step == 'step_2':
            ans = {
                'block_order'    : True,
                'detect_str' : 'block2',
                'requires_ack' : True,
                'next_step': 'None'
            }
            return ans
        
         # 2차 이동 전 yaw만 회전
        elif step == 'step_3':
            ans = {
                'block_pick_move'    : True,
                'pick_str' : 'second',
                'speed' : 10.0,
                'requires_ack' : True,
                'next_step': 'None'
            }
            return ans
        
        # 그리퍼 석션 시작 + Block 좌표 요청
        elif step == 'step_3':
            ans = {
                'gripper'    : True,
                'gripper_str' : 's',
                'drop_coor_order' : True,
                'requires_ack' : False,
                'next_step': 'None'
            }
            return ans
        
        # 블록 집으러 진입
        elif step == 'step_3':
            ans = {
                'block_pick_move'    : True,
                'pick_str' : 'third',
                'speed' : 10.0,
                'requires_ack' : True,
                'next_step': 'None'
            }
            return ans
        
        # 석션기가 블록을 집기까지 대기
        elif step == 'step_4':
            ans = {
                'wait_a_sec' : True,
                'time' : 2.0, # sec 단위
                'requires_ack' : False,
                'next_step': 'None'
            }
            return ans
        # 상승
        elif step == 'step_1':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, -80.0, 0.0, 0.0, 0.0],
                'speed'    : 5.0,
                'requires_ack' : True,
                'next_step': 'None'
            }
            return ans
        
        # 떨어뜨리는 초기 단계로 이동 ## 수정해야 해용
        elif step == 'step_1':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, -80.0, 0.0, 0.0, 0.0],
                'speed'    : 5.0,
                'requires_ack' : True,
                'next_step': 'None'
            }
            return ans
        
        # 떨어뜨리는 Offset 위치로 이동
        elif step == 'step_1':
            ans = {
                'block_drop_move' : True,
                'speed'    : 5.0,
                'requires_ack' : True,
                'next_step': 'None'
            }
            return ans
        
        # 석션기 종료와 동시에 하강
        elif step == 'step_1':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, 50.0, 0.0, 0.0, 0.0],
                'speed'    : 5.0,
                'gripper'    : True,
                'gripper_str' : 'e',
                'requires_ack' : True,
                'next_step': 'None'
            }
            return ans
        
        # 다시 상승 -> 이후 바로 step_1으로 이동할 수 있게
        elif step == 'step_1':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, -80.0, 0.0, 0.0, 0.0],
                'speed'    : 5.0,
                'gripper'    : True,
                'gripper_str' : 'e',
                'requires_ack' : True,
                'next_step': 'step_1'
            }
            return ans