import rospy
import numpy as np
import math
import time
from std_msgs.msg import Float32MultiArray as fl
from std_msgs.msg import Bool, Float32
from std_msgs.msg import String
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink


def dtr(dgree):
   return dgree*(np.pi/180)

l = [50, 65.95, 332.5, 270.1, 81.75, 17.25+132.47-6]
# l = [50, 65.95, 332.5, 270.2, 81.75, 17.25+132.47-6]

d = [l[0], l[1], 0, 0, 0]
a = [0, 0, l[2], l[3], l[4]+l[5]]
al = [0, dtr(90), 0, 0, dtr(90)]

class MakeChain:
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
    def IK(self, target_pose):
        angle = self.arm.inverse_kinematics(target_pose[:3], target_orientation=[0, 0, -1], orientation_mode="X") #, target_orientation=[0, 0, -1], orientation_mode="X")
        # orientation mode 를 "X"로 설정하기. EE의 green axis가 x축 이므로
        self.angles = np.round(np.rad2deg(angle), 3)
        self.angles = self.angles[1:5] #[0,n,n,n,n,0]

        if target_pose[0] > 0:
            wrist_angle_radians = math.atan(target_pose[1]/(target_pose[0]))
            wrist_angle_degrees = - (90 -math.degrees(wrist_angle_radians)) - target_pose[3] #atan로 구한 wrist의 각도를 빼서 0으로 맞춰주고, 목표 각도를 다시 더해주기
            print(wrist_angle_degrees)
        elif target_pose[0] < 0:
            wrist_angle_radians = math.atan(target_pose[1]/(target_pose[0]))
            wrist_angle_degrees = - (90 -math.degrees(wrist_angle_radians)) - target_pose[3] + 180 #atan로 구한 wrist의 각도를 빼서 0으로 맞춰주고, 목표 각도를 다시 더해주기
            print(wrist_angle_degrees)
        else:
            wrist_angle_degrees = 0

        self.angles = np.r_[self.angles, wrist_angle_degrees]
        return self.angles
   


def main():
    rospy.init_node('Direct_control', anonymous=True)
    pub_direct = rospy.Publisher('/Direct', fl, queue_size=10)
    rate = rospy.Rate(15) # 10hz
    arm = MakeChain()

    while not rospy.is_shutdown():
        # position = input("Enter position(x, y, z, twist): ") # 숫자 입력 받음
        x, y, z, twist = map(float, input("Enter position(x, y, z, twist): ").split())


        position = [x, y, z, twist]
        angle = arm.IK(position)
        msg = fl()
        msg.data = angle
        print(angle)
        pub_direct.publish(msg)        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
