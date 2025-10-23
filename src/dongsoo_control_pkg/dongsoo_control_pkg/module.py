
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