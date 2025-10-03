"""
코드 사용방법
handler = Start()
pos1 = handler.step('step_1')  # [0.1, 0.2, 0.3]
pos2 = handler.step('step_2')  # [0.3, 0.2, 0.1]
pos3 = handler.step('step_99') # None (정의되지 않은 step)
"""

class Start:
    def step(self, step):
        if step == 'step_1':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, -30.0, 0.0, 0.0, 0.0],
                'speed'    : 60.0,
                'requires_ack' : True,
                'next_step': 'step_2'
            }
            return ans
            
        elif step == 'step_2':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, 100.0, 0.0, 0.0, 0.0],
                'speed'    : 60.0,
                'requires_ack' : True,
                'next_step': 'step_3'
            }
            return ans
        
        elif step == 'step_3':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, -100.0, 0.0, 0.0, 0.0],
                'speed'    : 60.0,
                'requires_ack' : True,
                'next_step': 'step_4'
            }
            return ans
            
        elif step == 'step_4':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 0.0, 30.0, 0.0, 0.0, 0.0],
                'speed'    : 60.0,
                'requires_ack' : True,
                'next_step': 'None'
            }
            return ans
        
        else:
            print('[ZEUS] Wrong Step')
            return None
            
class Finish:
    def step(self, step):
        if step == 'step_1':
            ans = {
                'frame'    : 't',
                'position' : [50.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                'speed'    : 60.0,
                'requires_ack' : True,
                'next_step': 'step_2'
            }
            return ans
            
        elif step == 'step_2':
            ans = {
                'frame'    : 't',
                'position' : [-50.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                'speed'    : 60.0,
                'requires_ack' : True,
                'next_step': 'step_3'
            }
            return ans
        
        elif step == 'step_3':
            ans = {
                'frame'    : 't',
                'position' : [0.0, 50.0, 0.0, 0.0, 0.0, 0.0],
                'speed'    : 60.0,
                'requires_ack' : True,
                'next_step': 'step_4'
            }
            return ans
            
        elif step == 'step_4':
            ans = {
                'frame'    : 't',
                'position' : [0.0, -50.0, 0.0, 0.0, 0.0, 0.0],
                'speed'    : 60.0,
                'requires_ack' : True,
                'next_step': 'None'
            }
            return ans
        
        else:
            print('[ZEUS] Wrong Step')
            return None