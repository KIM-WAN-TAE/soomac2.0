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
        
        # 그리퍼 석션 시작
        elif step == 'step_3':
            ans = {
                'gripper'    : True,
                'gripper_str' : 's',
                'requires_ack' : False,
                'next_step': 'None'
            }
            return ans
    
        # 블록 Drop 위치 요청 // 기다리지는 않음
        elif step == 'step_4':
            ans = {
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
        
        # 블록 Drop 위치 요청 // 기다리지는 않음
        elif step == 'step_4':
            ans = {
                'drop_coor_order' : True,
                'requires_ack' : False,
                'next_step': 'None'
            }
            return ans