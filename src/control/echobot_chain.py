from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import numpy as np
import math

def dtr(dgree):
   return dgree*(np.pi/180)

l = [50, 65.95, 332.5, 270.1, 81.75, 17.25+132.47-6]
# l = [50, 65.95, 332.5, 270.2, 81.75, 17.25+132.47-6]

d = [l[0], l[1], 0, 0, 0]
a = [0, 0, l[2], l[3], l[4]+l[5]]
al = [0, dtr(90), 0, 0, dtr(90)]

class Echobot:
    def __init__(self):
        self.make_chain()

    def Make_URDF(self, link_name, d, a, al, th=0):
        return URDFLink(
            name = link_name,
            origin_translation=[a, 0, d], 
            origin_orientation=[al, 0, th],
            rotation=[0, 0, 1],
        )
    # 4DOF robot arm define
    def make_chain(self): 
            self.arm = Chain(name='arm', links=[
            OriginLink(), # base
            self.Make_URDF('link1', d[0], a[0], al[0], th=dtr(90)),
            self.Make_URDF('link2', d[1], a[1], al[1]),
            self.Make_URDF('link3', d[2], a[2], al[2]),
            self.Make_URDF('link4', d[3], a[3], al[3]),
            self.Make_URDF('link5', d[4], a[4], al[4])],
            active_links_mask=[False, True, True, True, True, False] )
 
    # IK : position -> 1st~4th angle / atan -> 5th angle 
    def IK(self, target_coord):
        angle = self.arm.inverse_kinematics(target_coord[:3], target_orientation=[0, 0, -1], orientation_mode="X") #, target_orientation=[0, 0, -1], orientation_mode="X")
        # orientation mode 를 "X"로 설정하기. EE의 green axis가 x축 이므로
        self.angles = np.round(np.rad2deg(angle), 3)
        self.angles = self.angles[1:5] #[0,n,n,n,n,0]

        if target_coord[0] > 0:
            wrist_angle_radians = math.atan(target_coord[1]/(target_coord[0]))
            wrist_angle_degrees = - (90 -math.degrees(wrist_angle_radians)) - target_coord[3] #atan로 구한 wrist의 각도를 빼서 0으로 맞춰주고, 목표 각도를 다시 더해주기
            # print(wrist_angle_degrees)
        elif target_coord[0] < 0:
            wrist_angle_radians = math.atan(target_coord[1]/(target_coord[0]))
            wrist_angle_degrees = - (90 -math.degrees(wrist_angle_radians)) - target_coord[3] + 180 #atan로 구한 wrist의 각도를 빼서 0으로 맞춰주고, 목표 각도를 다시 더해주기
            # print(wrist_angle_degrees)
        else:
            wrist_angle_degrees = target_coord[3]

        self.angles = np.r_[self.angles, wrist_angle_degrees]
        return self.angles
    
    def IK_non_twist(self, target_coord): # (x,y,z) -> theta x 4
        angle = self.arm.inverse_kinematics(target_coord, target_orientation=[0, 0, -1], orientation_mode="X") #, target_orientation=[0, 0, -1], orientation_mode="X")
        # orientation mode 를 "X"로 설정하기. EE의 green axis가 x축 이므로
        
        angle = np.round(np.rad2deg(angle), 3)
        angle = angle[1:5] #[0,n,n,n,n,0]    
        return angle
     
    def FK(self, degree):
        degree_for_FK = np.zeros(6)
        degree_for_FK[1:5] = degree[:4]
        degree_for_FK = np.deg2rad(degree_for_FK)
        transformation_matrix = self.arm.forward_kinematics(degree_for_FK)
        coord = transformation_matrix[:3, 3]
        return coord 