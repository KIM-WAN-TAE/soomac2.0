import os, sys
sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import rospy
import numpy as np
import math
import time
import camera_transformation as ct
from std_msgs.msg import Float32MultiArray as fl
from std_msgs.msg import Bool, Float32
from std_msgs.msg import String
from soomac.msg import action_info
from echobot_chain import Echobot
from functions import *
from copy import deepcopy
from coordinate_setting import *



class robot_function: # degree : 모터 5개의 각도, grip_size : mm
    def __init__(self):
        self.chain = Echobot()
        self.pub_action = rospy.Publisher('/action_req', action_info, queue_size=10)

    def move(self, coord, N = 0):
        if len(coord) == 4: # (x,y,z,rx) -> (th1, th2, th3, th4, th5)
            degree = self.chain.IK(coord)
        else:
            degree = coord
        
        if N != 0:
            self.action_pub(action = "move", degree = degree, N=N)
        else:
            self.action_pub(action = "move", degree = degree)

    def move_with_grip(self, coord, grip_mm):
        if len(coord) == 4: # (x,y,z,rx) -> (th1, th2, th3, th4, th5)
            degree = self.chain.IK(coord)
        else:
            degree = coord
    
        self.action_pub(action = "move_with_grip", degree = degree, grip_size=grip_mm)        

    def line(self, coord, twist):
        if len(coord) == 4: # (x,y,z,rx) -> (th1, th2, th3, th4, th5)
            degree = self.chain.IK(coord)
        else:
            degree = coord
        self.action_pub(action = "line", degree = degree, twist=twist)

    def grip_close(self, grip_size):
        self.action_pub(action = "grip_close", grip_size=grip_size)

    def grip_open(self, grip_size):
        self.action_pub(action = "grip_open", grip_size=grip_size)

    def continue_(self):
        self.action_pub(action = "continue")

    def previous(self):
        self.action_pub(action = "previous")                

    def stop(self):
        self.action_pub(action = "stop")                

    def action_pub(self, action = "x", degree = [0], grip_size = 0, N = 0, twist = 0):
        action_msg = action_info()
        action_msg.action = action
        action_msg.degree = degree
        action_msg.grip_size = grip_size   
        action_msg.N = N
        action_msg.twist = twist         
        self.pub_action.publish(action_msg)


class Control:
    def __init__(self):
        # 고정 좌표
        coord = coordinate_setting()
        self.parking_degree = deepcopy(coord.parking_degree)
        self.define_degree = deepcopy(coord.define_degree)
        self.camera_coord = deepcopy(coord.camera_coord)
        self.parking_above_degree = deepcopy(coord.parking_above_degree)

        self.gripper_open_mm = deepcopy(coord.gripper_open_mm)
        self.gripper_close_mm = deepcopy(coord.gripper_close_mm)


        self.rb = robot_function()
        self.object_size = self.gripper_open_mm
        self.pick_coord = self.define_degree
        self.place_coord = None
        self.action_state = 0
        self.mode = None

        # offset 설정
        self.pick_offset = 100
        self.place_offset = 200

        # ros topic
        self.pub_start = rospy.Publisher('/start', action_info, queue_size=10)
        self.camera_ready = rospy.Publisher('/camera_ready', Bool, queue_size=10)
    def modes(self):
        if self.mode == 'start':
            self.mode_start()

        elif self.mode == 'pnp':
            self.mode_pnp()
            
        elif self.mode == 'camera_pose':
            self.mode_camera_pose()

        elif self.mode == 'define_pose':
            self.mode_define_pose()   

        elif self.mode == 'end':
            self.mode_end()              

        elif self.mode == 'push':
            self.mode_push()              


    def mode_pnp(self):
        if self.action_state == 0:
            pass
        
        elif self.action_state == 1:
            print('##### [pnp_action] step 1 : pick_above')
            pick_above = deepcopy(self.pick_coord)
            pick_above[2] += self.pick_offset
            self.rb.move(pick_above)
            self.action_state += 1

        elif self.action_state == 2:
            print('##### [Mode : init_pos] step_2 : pick')
            self.rb.move(self.pick_coord)
            self.action_state += 1

        elif self.action_state == 3:
            print('##### [Mode : init_pos] step_2 : grip')
            self.rb.grip_close(self.object_size)
            self.action_state += 1

        elif self.action_state == 4:
            print('##### [Mode : init_pos] step_2 : pick_lift')
            pick_above = deepcopy(self.pick_coord)
            pick_above[2] += self.pick_offset
            self.rb.move(pick_above)
            self.action_state += 1

        elif self.action_state == 5:
            print('##### [Mode : init_pos] step_2 : place_above')
            place_above = deepcopy(self.place_coord)
            place_above[2] += self.place_offset
            self.rb.move(place_above)
            self.action_state += 1

        elif self.action_state == 6:
            print('##### [Mode : init_pos] step_2 : place')
            self.rb.move(self.place_coord)
            self.action_state += 1

        elif self.action_state == 7:
            print('##### [Mode : init_pos] step_2 : grip_open')
            self.rb.grip_open(self.gripper_open_mm)
            self.action_state += 1

        elif self.action_state == 8:
            print('##### [Mode : init_pos] step_2 : place')
            place_above = deepcopy(self.place_coord)
            place_above[2] += self.place_offset
            self.rb.move(place_above)            
            self.action_state += 1            

        elif self.action_state == 9:
            print('##### [Mode : init_pos] step_2 : camera_pos')
            self.rb.move(self.camera_coord)
            self.action_state += 1

        elif self.action_state == 10:
            msg = Bool()
            msg.data = True
            self.camera_ready.publish(msg)
            print("camera_ready topic")
        else:
            pass
    
    def mode_start(self):
        if self.action_state == 0:
            pass
        
        elif self.action_state == 1:
            print('\n---------------- Started! Moving to define_pose pose ----------------')
            print('##### [mode_start] step 1 : parking_above_degree')
            action_msg = action_info()

            action_msg.action = "start"
            action_msg.degree = self.parking_above_degree
            action_msg.grip_size = self.gripper_open_mm      

            self.pub_start.publish(action_msg)
            self.action_state += 1

        elif self.action_state == 2:
            print('##### [mode_start] step 2 : define_degree')
            self.rb.move(self.define_degree)
            self.action_state += 1


    def mode_camera_pose(self):
        if self.action_state == 0:
            pass
        
        elif self.action_state == 1:
            print('##### [Mode : init_pos] step_1 : camera_pos')
            self.rb.move(self.camera_coord) 
            self.action_state += 1

        elif self.action_state == 2:
            print('##### [Mode : init_pos] step_2 : camera_ready')
            msg = Bool()
            msg.data = True
            self.camera_ready.publish(msg)
            print("camera_ready topic")
            self.action_state += 1



    def mode_define_pose(self):
        if self.action_state == 0:
            pass
        
        elif self.action_state == 1:
            print('##### [start_action] step 1 : define_above')
            self.rb.move(self.define_degree)
            self.action_state += 1

    def mode_end(self):
        if self.action_state == 0:
            pass
        
        elif self.action_state == 1:
            print('##### [end] step 1 : parking_above_degree')
            self.rb.move(self.parking_above_degree)
            self.action_state += 1

        elif self.action_state == 2:
            print('##### [end] step 1 : parking')
            self.rb.move(self.parking_degree, N = 30)
            self.action_state += 1            

    def mode_push(self):
        if self.action_state == 0:
            pass
        
        elif self.action_state == 1:
            print('##### [Mode : init_pos] step_2 : grip_open')
            self.rb.grip_open(self.gripper_open_mm)
            self.action_state += 1

        elif self.action_state == 2:
            print('##### [push] step 1 : pick_above')
            pick_above = deepcopy(self.pick_coord)
            pick_above[2] += self.pick_offset
            self.rb.move(pick_above)
            self.action_state += 1

        elif self.action_state == 3:
            print('##### [Mode : init_pos] step_2 : pick')
            self.rb.move(self.pick_coord)
            self.action_state += 1

        elif self.action_state == 4: # line으로 바꾸기
            print('##### [Mode : init_pos] step_2 : push')
            self.rb.line(self.place_coord, twist=self.place_coord[3])
            self.action_state += 1

        elif self.action_state == 5:
            print('##### [Mode : init_pos] step_2 : place')
            self.rb.move(self.place_coord)
            self.action_state += 1

        elif self.action_state == 6:
            print('##### [Mode : init_pos] step_2 : place_above')
            place_above = deepcopy(self.place_coord)
            place_above[2] += self.place_offset
            self.rb.move(place_above)            
            self.action_state += 1  

        elif self.action_state == 7:
            print('##### [Mode : init_pos] step_2 : camera_pos')
            self.rb.move(self.camera_coord)
            self.action_state += 1

        elif self.action_state == 8:
            msg = Bool()
            msg.data = True
            self.camera_ready.publish(msg)
            print("camera_ready topic")

        else:
            pass


    ################## 조작 ################## 
    def stop(self):
        self.rb.stop()

    def previous(self): # 나중에 다시 보기
        self.rb.previous()

    def continue_(self):
        self.rb.continue_()
        self.action_state -= 1 # 기존 명령 다시 보내기
        self.modes()


class Callback:
    def __init__(self):
        self.control = Control()
        # vision to master
        rospy.Subscriber('/vision', fl, self.vision)         
        rospy.Subscriber('/camera_pose', Bool, self.camera_pose) # vision -> control

        # GUI to master
        rospy.Subscriber('/task_type', String, self.task_type)

        # motor_control to master
        rospy.Subscriber('/state_done', Bool, self.state_done)
        rospy.Subscriber('/impact', Bool, self.impact)
        rospy.spin()
        
    def vision(self, data): # camera -> 동작
        print('####### vision topic ####### ')
        print(data)
        vision_data = data.data
        self.control.object_size = vision_data[4]
        self.control.pick_coord = np.array(list(vision_data[:4]))
        self.control.place_coord = np.array(list(vision_data[5:]))
        self.control.action_state = 1

        if self.control.object_size == -1: # push 해야함
            self.control.mode = 'push'
        else:
            self.control.mode = 'pnp'
        
        self.control.modes()

    def camera_pose(self, msg):
        self.control.mode = 'camera_pose'
        self.control.action_state = 1            
        self.control.modes()        

    def task_type(self, data):
        print('####### vision topic ####### ')
        print('Task type is ', data)            
        task = data.data

        if task == "start":
            self.control.mode = 'start'
            self.control.action_state = 1            
            self.control.modes()

        elif task == "camera_pose":
            self.control.mode = 'camera_pose'
            self.control.action_state = 1            
            self.control.modes()

        elif task == "define_pose":
            self.control.mode = 'define_pose'
            self.control.action_state = 1            
            self.control.modes()

        elif task == "end":
            self.control.mode = 'end'
            self.control.action_state = 1            
            self.control.modes()     

        elif task == "stop" or task == "pause": 
            self.control.stop()

        elif task == "previous":
            self.control.previous()

        elif task == "continue":
            self.control.continue_()



    def impact(self, data):
        pass


    def state_done(self, data): # motor_control로 부터 state_done 받을 시 동작 메서드
        print("### state_done")
        self.control.modes()


def main():
    rospy.init_node('ControlNode', anonymous=True)
    Callback()
    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.logwarn("Control_master_node is on")
        main()

    except rospy.ROSInterruptException:
        rospy.loginfo('Program is shut down')    