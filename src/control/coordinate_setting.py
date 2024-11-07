import numpy as np
# 최종적으로, 모든 좌표는 cartesian 혹은 ik 기준 각도로 변환되어야 한다.
class coordinate_setting:
    def __init__(self):
        # cartesian 좌표계
        self.camera_coord = [0, 210, 370 , 0]
        
        # ik 기준 각도
        self.parking_degree = [0, 180, -130, -100, 0]
        self.define_degree = [0, 210, 370 , 0]

        # dynamixel 기준 각도
        self.parking_above_degree = [270, 143.7, 150.73, 110, 180]
        self.parking_degree = [270, 115.6, 160.6, 87.5, 180]

        self.gripper_open = 3666
        self.gripper_close = 2236
        
        self.gripper_open_mm = 72 #72
        self.gripper_close_mm = 19 #15.9
        self.change_dynamixel_degree_to_ik_degree()
        # 322 196.5
    def change_dynamixel_degree_to_ik_degree(self):
        change_param = [180, 90, 180, 180, 180] 
        for i in range(5):
            self.parking_above_degree[i] -= change_param[i]
            self.parking_degree[i] -= change_param[i]            