#!/usr/bin/env python3

class FreeMissionState():
    def __init__(self, tool, mode, direction):
        self.tool = tool
        self.mode = mode
        self.dir  = direction
        
        self.state = 0
    
    def find_dongsoo_mission(self):
        # 니퍼
        if self.tool == 'nipper':
            print('Get Nipper Plz')
            
            if self.state == 0:
                
            elif self.state == 1:
        
        
        # 버니어 캘리퍼
        elif self.tool == 'vernier_calipers':
            pass
        
        
        # 와이어 커터
        elif self.tool == 'wire_cutter':
            pass
            
            
        # 와이어 스트리퍼
        elif self.tool == 'wire_stripper':
            pass