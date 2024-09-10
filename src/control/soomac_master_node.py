import rospy
import numpy as np
import math
import time
import camera_transformation as ct
from std_msgs.msg import Float32MultiArray as fl
from std_msgs.msg import Bool, Float32
from std_msgs.msg import String
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

#####################################################################################################################################################################################
# Robot configuration

def dtr(dgree):
   return dgree*(np.pi/180)

l = [50, 65.95, 332.5, 270.1, 81.75, 17.25+132.47-6]
# l = [50, 65.95, 332.5, 270.2, 81.75, 17.25+132.47-6]

d = [l[0], l[1], 0, 0, 0]
a = [0, 0, l[2], l[3], l[4]+l[5]]
al = [0, dtr(90), 0, 0, dtr(90)]

#####################################################################################################################################################################################
# Ikpy

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
   
#####################################################################################################################################################################################   
# FSM Class

class FSM:
    def __init__(self):
        # initial setting
        self.state_done = False
        self.task_done = False
        self.grip_open = 72
        self.grip_seperation = self.grip_open
        self.current_state = "define_pose"
        self.last_state = "define_pose"
        self.arm = MakeChain()
        # fixed pose(degree)
        self.parking = np.array([0, 180, -130, -100, 0])# parking 자세 설계팀과 상의 필요 # 각도값 조절 필요, 일단 카메라 포즈랑 동일하게 해둠
        self.define_pose = np.array([0, 170, -120, -90, 0])# task 정의 자세 # GUI에서 실행 버튼 및 초기 위치 버튼 누르면 여기로 이동함
        self.camera_pose = np.array([0, 210, 370 , 0])
        # self.camera_pose = np.array([-20, 90, -70, 0, 0])

        # offset parameter
        self.above_offset = np.array([0, 0, 100, 0])
        self.grip_offset = np.array([0, 0, 20, 0])
        self.lift_offset = np.array([0, 0, 150, 0])
        self.object_size = None


        # 변수 사전 선언 // [0,0,0]으로 좌표 설정 시 해당 위치로 이동하라는 신호가 오면 base로 endpoint가 위치하려 하는 상황이 생기므로 초기값은 안전하게 설정함.
        self.pick_pose = [150, 0, 100, 0]
        self.place_pose = [150, 0, 100, 0]
        self.action_setting()

        # action 정의
        self.action_list =["define_pose" , "camera_pose",                         # 고정된 위치 동작
                           "pick_above" , "pick_grip" , "grip_on" , "pick_lift",  # pick zone 동작 
                           "place_lift", "place_grip", "grip_off", "place_above"] # place zone 동작
        # action 정의
        # self.action_list =["define_pose" , "camera_pose",                         # 고정된 위치 동작
        #                    "pick_grip" , "grip_on" , "middle_lift", 
        #                    "place_grip", "grip_off"] # place zone 동작
                            
        self.action_num = len(self.action_list)

        # ROS publish
        # master to motor_control
        self.pub_goal_pose = rospy.Publisher('/goal_pose', fl, queue_size=10)                  # pose 제어
        self.pub_grip_seperation = rospy.Publisher('/grip_seperation', Float32, queue_size=10) # grip 제어 
        self.pub_task_motor = rospy.Publisher('/task_to_motor_control', String, queue_size=10) # GUI 명령 하달
        self.pub_start = rospy.Publisher('/start', fl, queue_size=10)                        # GUI 명령 하달
        
        # master to vision
        self.camera_ready = rospy.Publisher('/camera_ready', Bool, queue_size=10)

        # master to GUI
        self.define_ready = rospy.Publisher('/define_ready', Bool, queue_size=10)         


    ######################################################################################################
    def start(self): # init pose로 direct 이동, GUI에서 초기 위치 누르면 해당 메서드 동작 ********************************************
        print('\n---------------- Started! Moving to define_pose pose ----------------')
        msg = fl()
        msg.data = self.define_pose
        self.pub_start.publish(msg)
        self.current_state = "define_pose"

    def move_to_camera(self): # camera pose로 direct 이동, GUI에서 camera 위치 누르면 해당 메서드 동작
        print('\nMoving to camera pose')
        msg = String()
        msg.data = "camera"
        self.pub_task_motor.publish(msg)

        # self.current_state = "camera_pose"
    def move_to_define(self): # camera pose로 direct 이동, GUI에서 camera 위치 누르면 해당 메서드 동작
        print('\nMoving to camera pose')
        msg = String()
        msg.data = "define"
        self.pub_task_motor.publish(msg)
        # self.current_state = "camera_pose"        

    def action_setting(self, print_op=False): # vision으로 부터 받은 정보를 바탕으로 way point의 좌표 정보 제작
        # pick zone
        self.pick_above = self.pick_pose + self.above_offset 
        self.pick_grip = self.pick_pose + self.grip_offset
        self.pick_lift = self.pick_pose + self.lift_offset

        # place zone
        self.place_above = self.place_pose + self.above_offset
        self.place_grip = self.place_pose + self.grip_offset
        self.place_lift = self.place_pose + self.lift_offset

        self.middle_lift = (self.pick_lift + self.place_lift)/2
        if print_op == True:
            print('\nSpecific position data ---------------')
            print(f'-Pick Above:  [{self.pick_above[0]:.1f}, {self.pick_above[1]:.1f}, {self.pick_above[2]:.1f}, {self.pick_above[3]:.1f}]')
            print(f'-Pick Grip:   [{self.pick_grip[0]:.1f}, {self.pick_grip[1]:.1f}, {self.pick_grip[2]:.1f}, {self.pick_grip[3]:.1f}]')
            print(f'-Pick Lift:   [{self.pick_lift[0]:.1f}, {self.pick_lift[1]:.1f}, {self.pick_lift[2]:.1f}, {self.pick_lift[3]:.1f}]')
            print(f'-Place Above: [{self.place_above[0]:.1f}, {self.place_above[1]:.1f}, {self.place_above[2]:.1f}, {self.place_above[3]:.1f}]')
            print(f'-Place Grip:  [{self.place_grip[0]:.1f}, {self.place_grip[1]:.1f}, {self.place_grip[2]:.1f}, {self.place_grip[3]:.1f}]')
            print(f'-Place Lift:  [{self.place_lift[0]:.1f}, {self.place_lift[1]:.1f}, {self.place_lift[2]:.1f}, {self.place_lift[3]:.1f}]')
            #print('pick_above:',self.pick_above, '\npick_grip:', self.pick_grip, '\npick_lift:', self.pick_lift)
            #print('place_above', self.place_above, '\nplace_grip', self.place_grip, '\nplace_lift', self.place_lift)    

       
    # 비전으로터 받는 데이터 형식 {"pick": (x, y, z, theta, grip size), "place": (x, y, z, heading)}
    def get_data_from_vision(self,data): # vision으로 부터 받은 데이터를 가공해줌 -> 이후 action_setting 진행 -> 이후 new_state로 넘어가서 새로운 동작 진행
        vision_data = data
        self.object_size = np.array(vision_data[4])
        self.pick_pose = np.array(list(vision_data[:4]))
        self.place_pose = np.array(list(vision_data[5:]))
        #self.pick_pose = ct.transformation_init(self.pick_pose)
        #elf.place_pose = ct.transformation_init(self.place_pose)
        self.action_setting(print_op=True) # vision data 기반으로 way point 설정, # print option은 vision data로 가공된 정보 확인하기 위함
        self.new_state() # init에서 vision 정보 받았으니 새로운 state로 이동 시 물체 위로 이동

    ######################################################################################################

    def new_state(self): # state_done 시 방금 도착한 자세가 init이 아닌 경우 새로운 state로 변경해줌. 이후에는 바로 동작 update 진행

        current_state_index = self.action_list.index(self.current_state)
        print('\nCurrent state:', self.current_state, '(index:', current_state_index, end=')')

        if current_state_index == self.action_num-1: # place_offset(마지막)인 경우 init_pos로 이동 // init_pos로 이동 시 state_done topic이 와도 다음 동작으로 넘어가지 않음. 다시 vision topic 올때 까지 기다림
            
            self.last_state = self.current_state
            self.current_state = self.action_list[1] # camera_pose로 이동
            print(' --> Next state:', self.current_state)
            self.update()

            print("Pick and place 완료")


        else:
            self.last_state = self.current_state
            self.current_state = self.action_list[current_state_index+1]
            print(' --> Next state:', self.current_state)
            self.update()

    def update(self): #new_state로부터 갱신된 동작을 진행
        # camera
        if self.current_state == "define_pose":
            self.pub_pose(self.camera_pose, 1) # option 1
            print('updated to define_pose')            

        elif self.current_state == "camera_pose":            
            self.pub_pose(self.camera_pose, 3) # option 3
            print('updated to camera_pose')
            
        # pick 
        elif self.current_state == "pick_above":
            self.pub_pose(self.pick_above)
            print('updated to pick_above')            

        elif self.current_state == "pick_grip":
            self.pub_pose(self.pick_grip)
            print('updated to pick_grip')

        elif self.current_state == "grip_on":
            self.grip_seperation = self.object_size
            self.pub_grip(self.grip_seperation)
            print("updated to grip_on")

        elif self.current_state == "pick_lift":
            self.pub_pose(self.pick_lift)
            print("updated to pick_lift")
            
        # place
        elif self.current_state == "place_above":
            self.pub_pose(self.place_above)
            print("updated to place_above")

        elif self.current_state == "place_grip":
            self.pub_pose(self.place_grip)
            print("updated to place_grip")

        elif self.current_state == "grip_off":
            self.grip_seperation = self.grip_open
            self.pub_grip(self.grip_open)
            print("updated to grip_off")

        elif self.current_state == "place_lift":
            self.pub_pose(self.place_lift)
            print("updated to place_lift")

        elif self.current_state == "middle_lift":
            self.pub_pose(self.middle_lift)
            print("updated to middle_lift")

        print('------------------------------------------------------------------')

    
    def pub_pose(self, goal_pose = [150, 0, 100, 0], option = 0): # publish 좌표 + 손목 각도[,4] -> 5축 각도[,5] # goal_pose가 없는 init, parking일 경우에는 goal_pose는 임의의 값으로 설정(어차피 안씀)

        if option == 0 : #IK 계산
            self.goal_degree = self.arm.IK(goal_pose)
            print('-- goal pose (%.1f %.1f %.1f %.1f)' % (goal_pose[0], goal_pose[1], goal_pose[2], goal_pose[3]))

        elif option == 1 : # define Pose(init_pose)
            self.goal_degree = self.define_pose
                
        elif option == 2 : #거치대 Pose
            self.goal_degree = self.parking

        elif option == 3 : # twist 각도 보존 (camera pose용)
            self.goal_degree = self.arm.IK(goal_pose)
            print('-- goal pose (%.1f %.1f %.1f %.1f)' % (goal_pose[0], goal_pose[1], goal_pose[2], goal_pose[3]))
            self.goal_degree[4] = goal_pose[3]

        goal_msg = fl() 
        goal_msg.data = self.goal_degree
        self.pub_goal_pose.publish(goal_msg)
        print('-- goal axis (%.1f %.1f %.1f %.1f %.1f)' % (self.goal_degree[0], self.goal_degree[1], self.goal_degree[2], self.goal_degree[3], self.goal_degree[4]))


    def pub_grip(self, grip_seperation):
        grip_msg = Float32()
        grip_msg.data = grip_seperation
        self.pub_grip_seperation.publish(grip_msg)
        print('grip seperation:', grip_seperation)



#####################################################################################################################################################################################   
# Callback Class

class Callback:
    def __init__(self):
        self.soomac_fsm = FSM()
        self.camera_pose_grip_state = True
        self.ros_sub()
        

    def ros_sub(self): # main_loop

        # vision to master
        rospy.Subscriber('/vision', fl, self.vision)         
        rospy.Subscriber('/camera_pose', Bool, self.move_to_camera_for_vision) # vision -> control

        # GUI to master
        rospy.Subscriber('/task_type', String, self.task_type)

        # motor_control to master
        rospy.Subscriber('/state_done', Bool, self.state_done)
        rospy.Subscriber('/task_done', Bool, self.task_done)
        rospy.spin()

    def vision(self, data): # camera -> 동작
        if self.soomac_fsm.current_state == "camera_pose":
            print('\nSubscribed vision topic\n: object initial pos({:.1f} {:.1f} {:.1f}) ori({:.1f}°), object size({:.1f}), goal pos({:.1f} {:.1f} {:.1f}) ori({:.1f}°)'.format(*data.data))
            self.soomac_fsm.get_data_from_vision(data.data)
        else:
            print("잘못된 자세입니다.")

    def move_to_camera_for_vision(self, data):
        self.soomac_fsm.current_state = "camera_pose"
        self.soomac_fsm.update()


    def task_type(self, data): # gui로 부터 task받을 시 동작 메서드
        #rospy.loginfo(f'Subscribed {data.data} topic')

        if data.data == "start" : # 주차(or 어디든) -> init # 초가 주차 상태일 때 누르면 init으로 옴
            self.soomac_fsm.start()

        elif data.data == "camera_pose" :
            print('******* camera_pose')
            self.soomac_fsm.move_to_camera() # gui pause 해제용
            self.soomac_fsm.current_state = "camera_pose"
            self.soomac_fsm.update()

        elif data.data == "define_pose" : ##############***
            print('******* define_pose')
            self.soomac_fsm.move_to_define() # gui pause 해제용
            self.soomac_fsm.current_state = "define_pose"
            self.soomac_fsm.update()

        elif data.data == "stop" or data.data == "pause" :
            msg = String()
            msg.data = "stop"
            self.soomac_fsm.pub_task_motor.publish(msg)
            print(msg.data)
        
        elif data.data == "previous" :
            print('******* previous')
            if self.soomac_fsm.current_state != self.soomac_fsm.last_state: 
                print('moving to previous state\n')
                print('이동 지점 : ', self.soomac_fsm.current_state)
                print('출발 지점(previous 지점) : ', self.soomac_fsm.last_state)
                
                self.soomac_fsm.current_state = self.soomac_fsm.last_state
                msg = String()
                msg.data = "previous"
                self.soomac_fsm.pub_task_motor.publish(msg)
                # rospy.sleep(0.1) ######### **********        
                self.soomac_fsm.update()
            else :
                print('already initial pose\n')

        elif data.data == "continue" :
            msg = String()
            msg.data = "continue"
            self.soomac_fsm.pub_task_motor.publish(msg)
            print('Continuing the operation after stop')

    def state_done(self, data): # motor_control로 부터 state_done 받을 시 동작 메서드
        # print('test')
        if self.soomac_fsm.current_state != "camera_pose" and self.soomac_fsm.current_state != "define_pose": # init_pose로 이동 완료된 상황에서는 new_state안하고 vision 정보 기다림
            print('state_done\n')
            self.soomac_fsm.new_state()
            self.camera_pose_grip_state = False

        elif self.soomac_fsm.current_state == "camera_pose": # camera pose로 이동 완료 시
            print('ready to camera')
            msg = Bool()
            msg.data = True
            if self.camera_pose_grip_state == False:
                self.soomac_fsm.pub_grip(self.soomac_fsm.grip_open)
                self.camera_pose_grip_state = True
            else:
                self.soomac_fsm.camera_ready.publish(msg) # to vision                

        elif self.soomac_fsm.current_state == "define_pose":
            print('ready to define')
            msg = Bool()
            msg.data = True
            self.soomac_fsm.define_ready.publish(msg) # to GUI
            if self.camera_pose_grip_state == False:
                self.soomac_fsm.pub_grip(self.soomac_fsm.grip_open)   
                self.camera_pose_grip_state = True         

    # 추후 코드 수정 필요
    def task_done(self): # motor_control에서 주어진 명령 수행 완료 시
        print('subscribed task_done topic')
        self.soomac_fsm.task_done = True
        

#####################################################################################################################################################################################

def main():
    rospy.init_node('soomac_master_node', anonymous=True)
    Callback()

if __name__ == '__main__':
    try:
        rospy.logwarn("Master Node is on")
        main()

    except rospy.ROSInterruptException:
        rospy.loginfo('Program is shut down')
