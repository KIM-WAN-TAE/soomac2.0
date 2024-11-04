## angle -> move motort / angle(0,0,0,0) -> end point will be located in (396, 0, 102.5)

import os, sys
sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import rospy
import time
import numpy as np
# 현재 스크립트의 디렉토리 경로를 가져옵니다.
#module_directory = os.path.join(os.path.dirname(__file__), "~/DynamixelSDK/ros/dynamixel_sdk")
module_directory = os.path.expanduser("~/DynamixelSDK/ros/dynamixel_sdk/src")
# module_directory = os.path.join(os.path.expanduser("~"), "DynamixelSDK/ros/dynamixel_sdk")

# sys.path 리스트에 모듈 디렉토리를 추가합니다.
sys.path.append(module_directory)

from dynamixel_sdk import *
from dynamixel_sdk.port_handler import PortHandler
from dynamixel_sdk.packet_handler import PacketHandler
from dynamixel_sdk.robotis_def import *

from std_msgs.msg import Float32MultiArray as fl
from std_msgs.msg import Float32, Bool, String
from soomac.msg import action_info

from functions import *
# import threading
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# import plotly.graph_objects as go
DEVICENAME = '/dev/ttyUSB0'


#**********************XM430-W350-R(PROTOCOL_VERSION 2.0)**************************#

# Control table address
XM_ADDR_TORQUE_ENABLE           = 64

XM_ADDR_VELOCITY_I_GAIN         = 76
XM_ADDR_VELOCITY_P_GAIN         = 78

XM_ADDR_POTISION_D_GAIN         = 80
XM_ADDR_POSITION_I_GAIN         = 82
XM_ADDR_POSITION_P_GAIN         = 84

XM_ADDR_FEEDFORWARD_2ND_GAIN    = 88
XM_ADDR_FEEDFORWARD_1ST_GAIN    = 90
XM_ADDR_PROFILE_ACCELERATION    = 108
XM_ADDR_PROFILE_VELOCITY        = 112
XM_ADDR_GOAL_POSITION           = 116
XM_ADDR_MOVING                  = 122
XM_ADDR_MOVING_STATUS           = 123
XM_ADDR_PRESENT_POSITION        = 132
XM_ADDR_VELOCITY_LIMIT          = 44

XM_CURRENT_ADDR = 126

XM_PROTOCOL_VERSION = 2.0

XM_DXL_ID = [0, 1, 2, 3, 4, 5]
gripper_DXL_ID = 5
XM_BAUDRATE = 3000000

XM_TORQUE_ENABLE = 1
XM_TORQUE_DISABLE = 0

#xm_packet_handler = PacketHandler(XM_PROTOCOL_VERSION)

# 각 모터에 대한 ID 및 레지스터 주소 설정
#XM_DXL_ADDR_PRESENT_POSITION = 126  # XM 모터의 현재 위치 값을 읽기 위한 레지스터 주소


################################################################################################################################################
# parameter
N = 50 # link motor 경로 분활
N_grip = 2 # gripper motor 경로 분활
# timer = time.time()
# repeat_time = 0.05
# impact_num = 13

class Impact: # 작업 완료
    def __init__(self):
        self.last_torgue = np.array([])
        self.diff_torques = np.array([]).reshape(0,5)
        self.diff_1st = rospy.Publisher('/diff_1st', Float32, queue_size=10)
        self.diff_2nd = rospy.Publisher('/diff_2nd', Float32, queue_size=10)
        self.impact_to_gui = rospy.Publisher('/impact_to_gui', Bool, queue_size=10)

    def diff(self, current_torque): # 입력 : 현재 토크 값, 출력은 없으며 전역 변수 diff_torques에 토크 변화량이 저장됨
        # print("#######미분#######")
        current_torque = np.array(current_torque)
        diff_torque = np.zeros(5)
        if len(self.last_torgue)!=0: # last_torque 존재 시  
            diff_torque = self.last_torgue - current_torque
            self.last_torgue = current_torque
            self.diff_torques = np.append(self.diff_torques,[np.abs(diff_torque)], axis = 0)
        else:
            self.last_torgue = current_torque

        if self.diff_torques.shape[0]>=10: # 10개 이상의 토크 미분값이 저장된 경우
            self.diff_torques[0] = self.diff_torques[8]
            self.diff_torques[1] = self.diff_torques[9]
            self.diff_torques = self.diff_torques[:2]
        self.diff_1st.publish(diff_torque[1])

    def impact_check(self):
        #print("#######충격 확인#######")
        diff_torques_num = self.diff_torques.shape[0] # 저장된 변화량의 개수
        # dof = self.diff_torques.shape[1] # 모터의 개수(5)
        diff_2nd = np.zeros(5) # 토크 변화량의 기울기가 급변할 때를 파악하기 위함.

        result = 0 # 충돌 결과
        if self.diff_torques.shape[0] >= 2: # 1차 미분값이 2개 이상 모이면
            for i in range(5):
                diff_2nd[i] = self.diff_torques[diff_torques_num-1][i]-self.diff_torques[diff_torques_num-2][i]
            diff_2nd = np.abs(diff_2nd)
            
            if diff_2nd[0] >= 100: # 수평 방향 충돌 감지
                result = 1
                rospy.logerr('########## impact 발생(수평) ')

            # if diff_2nd[1] >= 150: # XM_1 방향 충돌 감지
            #     result = 1
            #     rospy.logerr('########## impact 발생(XM_1) ')

            # if diff_2nd[2] >= 80: # 수직 방향 충돌 감지
            #     result = 1
            #     rospy.logerr('########## impact 발생(수직)')

            # if diff_2nd[0]/80 + diff_2nd[2]/80 >= 1.5: # 수직 방향 충돌 감지
            #     result = 1
            #     rospy.logerr('########## impact 발생(대각선)')
            
        self.diff_2nd.publish(diff_2nd[1])                

        return result
            
    def print_info(self, test_data):
        print("input data : ",test_data)
        print("last_torque : ",self.last_torgue)
        print("diff_torques : ",self.diff_torques)
        # print("last_diff_2nd : ",self.last_diff_2nd)

class DynamixelNode:
    def __init__(self): # 파라미터 설정
        # setting
        self.impact = Impact()
        self.dynamixel_setting()
        self.change_para = 1024/90

        ## ROS
        # for state done

        # for plot_torque
        self.pub_data_1 = rospy.Publisher('XM_0', Float32, queue_size=10)
        self.pub_data_2 = rospy.Publisher('XM_1', Float32, queue_size=10)
        self.pub_data_3 = rospy.Publisher('XM_2', Float32, queue_size=10)
        self.pub_data_4 = rospy.Publisher('XM_3', Float32, queue_size=10)
        self.pub_data_5 = rospy.Publisher('XM_4', Float32, queue_size=10)

        # self.data_array = np.zeros((20, 5))  # (20, 4) 형태의 데이터 배열 초기화

    def dynamixel_setting(self): # 다이나믹셀 세팅
        # XM 모터와 통신을 위한 포트 핸들러 및 패킷 핸들러 초기화
        self.port_handler_xm = PortHandler(DEVICENAME)
        self.packet_handler_xm = PacketHandler(XM_PROTOCOL_VERSION)

        # XM 모터 포트 열기
        if self.port_handler_xm.openPort():
            rospy.loginfo("Successfully opened the XM port.")
        else:
            rospy.logerr("Failed to open the XM port.")
            rospy.signal_shutdown("Failed to open the XM port.")
            return

        # XM 모터 통신 속도 설정
        if self.port_handler_xm.setBaudRate(XM_BAUDRATE):
            rospy.loginfo("Successfully set the XM baudrate.")
        else:
            rospy.logerr("Failed to set the XM baudrate.")
            rospy.signal_shutdown("Failed to set the XM baudrate.")
            return
        
        # link 모터의 토크 활성화
        for dxl_id in XM_DXL_ID:
            dxl_comm_result, dxl_error = self.packet_handler_xm.write1ByteTxRx(self.port_handler_xm, dxl_id, XM_ADDR_TORQUE_ENABLE, XM_TORQUE_ENABLE)
            dxl_comm_result, dxl_error = self.packet_handler_xm.write1ByteTxRx(self.port_handler_xm, dxl_id, XM_ADDR_PROFILE_ACCELERATION, 5)

            if dxl_comm_result != COMM_SUCCESS:
                rospy.logerr("Failed to enable torque for XM Motor ID: {}".format(dxl_id))
                rospy.signal_shutdown("Failed to enable torque for XM Motor ID: {}".format(dxl_id))
                return
            else:
                rospy.loginfo("Torque enabled for XM Motor ID: {}".format(dxl_id))
        
        # gripper 모터의 토크 활성화
        dxl_comm_result, dxl_error = self.packet_handler_xm.write1ByteTxRx(self.port_handler_xm, gripper_DXL_ID, XM_ADDR_TORQUE_ENABLE, XM_TORQUE_ENABLE)
        dxl_comm_result, dxl_error = self.packet_handler_xm.write1ByteTxRx(self.port_handler_xm, gripper_DXL_ID, XM_ADDR_PROFILE_ACCELERATION, 5)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("Failed to enable torque for XM Motor ID: {}".format(gripper_DXL_ID))
            rospy.signal_shutdown("Failed to enable torque for XM Motor ID: {}".format(gripper_DXL_ID))
            return
        else:
            rospy.loginfo("Torque enabled for XM Motor ID: {}".format(gripper_DXL_ID))        
        
        self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, 0, XM_ADDR_POSITION_P_GAIN, 800) 
        self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, 1, XM_ADDR_POSITION_P_GAIN, 300)
        self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, 2, XM_ADDR_POSITION_P_GAIN, 800)
        self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, 3, XM_ADDR_POSITION_P_GAIN, 800) 
        self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, 4, XM_ADDR_POSITION_P_GAIN, 800)
        self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, 5, XM_ADDR_POSITION_P_GAIN, 800)         
        
        self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, 0, XM_ADDR_PROFILE_VELOCITY, 180) 
        self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, 1, XM_ADDR_PROFILE_VELOCITY, 180)
        self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, 2, XM_ADDR_PROFILE_VELOCITY, 180)
        self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, 3, XM_ADDR_PROFILE_VELOCITY, 180) 
        self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, 4, XM_ADDR_PROFILE_VELOCITY, 180)
        self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, 5, XM_ADDR_PROFILE_VELOCITY, 180) 

        
        # p gain 설정
        for i in range(0,5):
            self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, i, XM_ADDR_POTISION_D_GAIN, 10) #0
            self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, i, XM_ADDR_FEEDFORWARD_1ST_GAIN, 10) #0

    def read_motor_position(self, port_handler, packet_handler, dxl_id, addr_present_position): # 현재 모터 value 도출해주는 메서드
        # 모터의 현재 위치 읽기
        dxl_present_position, _, _ = packet_handler.read2ByteTxRx(port_handler, dxl_id, addr_present_position)
        return dxl_present_position
    
    def return_current_values(self):
        present_position = np.zeros(6)        
        for dxl_id in range(0,6):
            present_position[dxl_id] = self.read_motor_position(self.port_handler_xm, self.packet_handler_xm, dxl_id, XM_ADDR_PRESENT_POSITION)        
        return present_position

    def read_motor_current(self, dxl_id): # 주어진 id의 모터에 대한 전류 값 반환
        # 모터의 현재 전류 값 읽기
        dxl_present_current, dxl_comm_result, dxl_error = self.packet_handler_xm.read2ByteTxRx(self.port_handler_xm, dxl_id, XM_CURRENT_ADDR)
        
        if dxl_comm_result != COMM_SUCCESS:
            # rospy.logerr("Failed to read current for XM Motor ID: {}".format(dxl_id))
            return None
        
        # 전류 값을 부호 있는 정수로 변환
        if dxl_present_current >= 32768:  # 음수 값 처리를 위한 부호 확장
            dxl_present_current -= 65536
        
        return dxl_present_current
        
    def monitor_current(self): # 모터 전류값 모니터링하는 메서드. 모니터링 후 충격 확인까지 진행
        global t
        data_t = np.zeros(5)
        result = 0
        
        for dxl_id in XM_DXL_ID[:5]:
            torque_value = self.read_motor_current(dxl_id)
            if torque_value is not None:
                # rospy.loginfo("XM Motor ID: {}, Current Torque: {:.2f}, Direction: {}".format(dxl_id, torque, direction))  
                data_t[dxl_id] = torque_value
        
        self.plot_torque(data_t)
        self.impact.diff(data_t)
        result = self.impact.impact_check()
        return result
        
    def plot_torque(self, data_t): # 모터 전류값 Plot 해주는 메서드
        # print(data_t)
        msg_1 = Float32(data=data_t[0])
        msg_2 = Float32(data=data_t[1])
        msg_3 = Float32(data=data_t[2])
        msg_4 = Float32(data=data_t[3])
        msg_5 = Float32(data=data_t[4])
        
        self.pub_data_1.publish(msg_1)
        self.pub_data_2.publish(msg_2)
        self.pub_data_3.publish(msg_3)
        self.pub_data_4.publish(msg_4)
        self.pub_data_4.publish(msg_5)

    def shutdown(self): # 종료 시 포트 닫기
        # 노드 종료 시 AX, XM 시리얼 포트 닫기
        self.port_handler_xm.closePort()    
        rospy.loginfo("Shutdown Dynamixel node.")

    def pub_pose(self, dynamixel_value): # input : (,6) // 5축 degree + gripper_value 
        
        for dxl_id in XM_DXL_ID:
            dv = int(dynamixel_value[dxl_id]) 
            self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, dxl_id, XM_ADDR_GOAL_POSITION, dv)
class Pose:
    def __init__(self, start_value, action_msg):
        self.goal_sub = rospy.Subscriber('/action_req', action_info, self.callback_action)  # from IK solver
        self.state_done_topic = rospy.Publisher('/state_done', Bool, queue_size=10) # to master
        self.state_done_okay = False
        
        self.stop_state = False
        # 초기 정보 및 첫 동작 정보 설정
        self.last_value = start_value
        self.action = action_msg.action
        self.degree = action_msg.degree
        self.grip_mm = action_msg.grip_size
        self.N = action_msg.N                
        self.trajectory_value = np.zeros(6) # 대기열       
        self.move_with_grip()

    def callback_action(self, action_msg):
        self.stop_state = False

        self.action = action_msg.action
        self.degree = action_msg.degree
        self.grip_mm = action_msg.grip_size   
        self.N = action_msg.N
        
        print(action_msg)
        if self.action == "move_with_grip":
            self.move_with_grip()
        elif self.action == "move":
            self.move()
        elif self.action == "line":
            self.line()
        elif self.action == "grip_close":
            self.grip_close()
        elif self.action == "grip_open":                            
            self.grip_open()
            
    def move_with_grip(self):
        self.state_done_okay = False
        goal_value = degree_to_dynamixel_value(self.degree, self.grip_mm)
        self.trajectory_value = cubic_trajectory(self.last_value, goal_value)

    def move(self):
        self.state_done_okay = False
        current_grip_value = self.last_value[5]
        goal_value = degree_to_dynamixel_value(self.degree, current_grip_value, grip_option="value")
        if self.N != 0:
            self.trajectory_value = cubic_trajectory(self.last_value, goal_value, N_req=self.N)
        else:
            self.trajectory_value = cubic_trajectory(self.last_value, goal_value)


    def line():
        pass

    def grip_close(self):
        self.state_done_okay = False
        goal_value = self.last_value
        goal_value[5] = grip_mm_to_value(self.grip_mm)
        self.trajectory_value = gripper_trajectory(self.last_value, goal_value)

    def grip_open(self):
        self.state_done_okay = False
        goal_value = self.last_value
        goal_value[5] = grip_mm_to_value(self.grip_mm)
        self.trajectory_value = gripper_trajectory(self.last_value, goal_value)

    def state_done(self): # link, gripper 제어 완료 시 state_done 토픽 발행해주는 메서드
        if self.state_done_okay == False:
            state_msg = Bool()
            state_msg.data = True
            self.state_done_topic.publish(state_msg)
        self.state_done_okay = True

    def pose_update(self):
        if len(self.trajectory_value) > 1: # trajectory 대기열에 2개 이상 존재 시 가장 앞 값을 last_value에 넣고 해당 값을 삭제.
            self.last_value = self.trajectory_value[0]
            self.trajectory_value = np.delete(self.trajectory_value,0,0) # 대기열 첫번째 값 삭제
        
        elif len(self.trajectory_value) == 1: # 값이 1개라면 traj 모두 진행 후 목표 값에 도달해있다는 뜻임. 따라서 state_done 발행
            self.last_value = self.trajectory_value[0] # traj의 마지막 인자로 last_value 설정
            self.state_done() # state_done 토픽 발행 시, moster_node로 부터 goal_pose가 들어오고, trajectory 배열이 추가로 생성됨
            # print("state_done\n\n")
            
        elif len(self.trajectory_value) == 0: # 값이 0이라면, 아직 한번도 trajectory를 진행하지 않은 상태임. 해당 경우는 초기 실행 상태임으로 pose update 없이 pass
            pass

def main(data):
    rate = rospy.Rate(15)
    print('motor_control_node is started')
    global dynamixel
    dynamixel = DynamixelNode()
    current_value = dynamixel.return_current_values()

    pose = Pose(current_value, data)
    impact = Impact()


    while not rospy.is_shutdown():
        
        impact_state = dynamixel.monitor_current()
        if impact_state == 1:
            pose.stop_state = True 
            impact.impact_to_gui.publish(True)

        if pose.stop_state == False: # stop이 아니면 pose update
            pose.pose_update()
            print(len(pose.trajectory_value))
        # print(pose.last_value)
        dynamixel.pub_pose(pose.last_value) # 계속 last_value로 모터 작동

        # if pose.end:
        #     dynamixel.end()

        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('motor_control', anonymous=True)
        print("waiting for start")
        start_sub = rospy.Subscriber('/start', action_info, main)  # from IK solver
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
