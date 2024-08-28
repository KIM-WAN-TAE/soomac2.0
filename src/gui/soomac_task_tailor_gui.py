#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
from std_msgs.msg import Float32MultiArray as fl
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image as Im
from soomac.srv import DefineTask, DefineTaskResponse

import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from pathlib import Path
from PIL import Image, ImageTk, ImageEnhance
Image.ANTIALIAS = Image.LANCZOS

import time
import pygame  # pygame 라이브러리 추가
import customtkinter as ctk
import cv2
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import matplotlib
from matplotlib import pyplot as plt
matplotlib.use("TkAgg")

from vision.realsense.realsense_depth import DepthCamera
from vision.realsense.utilities import compute_xyz, save_as_npy

# Pygame 초기화 및 사운드 로드
pygame.mixer.init()
# click_sound = pygame.mixer.Sound("/home/hyunwoo20/catkin_ws/src/soomac/src/gui/click_sound.mp3")  # 경로를 실제 파일 경로로 변경
# click_sound = pygame.mixer.Sound("/home/choiyoonji/catkin_ws/src/soomac/src/gui/click_sound.mp3")  # 경로를 실제 파일 경로로 변경
click_sound = pygame.mixer.Sound("/home/seojin/catkin_ws/src/soomac/src/gui/click_sound.mp3")  # 경로를 실제 파일 경로로 변경

# image_path = "/home/choiyoonji/catkin_ws/src/soomac/src/gui/start_image2.jpg"
image_path = "/home/seojin/catkin_ws/src/soomac/src/gui/start_image2.jpg"

task_name = None #task_name 토픽 발행을 위한 전역 변수 설정
rgb_frame = None
depth_frame = None

class Robot_control:
    def __init__(self):
        self.pub_vision = rospy.Publisher('/vision', fl, queue_size=10) 
        self.pub_gui = rospy.Publisher('/task_type', String, queue_size=10)
        self.goal_pose_test = rospy.Publisher('/goal_pose', fl, queue_size=10)
        self.camera_pose = rospy.Publisher('/camera_pose', Bool, queue_size=10)

        rospy.Subscriber('/impact_to_gui', Bool, self.impact_cb)
        rospy.Subscriber('/define_ready', Bool, self.define_ready_test)
        rospy.Subscriber('/camera_ready', Bool, self.camera_ready_test)
        rospy.Subscriber('/rgb_frame', Im, self.rgb_callback)
        rospy.Subscriber('/depth_frame', Im, self.depth_callback)
        
        # gui msg type 정의
        self.gui_msg = String()
        self.gui_msg.data = None
        # vision test용 msg 
        self.vision_msg = fl()
        self.vision_msg.data = [250, 250, 0, 0, 15, # pick : (x, y, z, theta, grip_size) 
                                -250, 250, 0, 30 ] # place : (x, y, z, theta)

        self.impact_screen_exis = False

    def rgb_callback(self, image):
        global rgb_frame
        image.encoding = 'rgb8'
        bridge = CvBridge()
        # Convert ROS Image message to OpenCV image
        rgb_frame = bridge.imgmsg_to_cv2(image, "bgr8")
        
    def depth_callback(self, image):
        global depth_frame
        bridge = CvBridge()
        # Convert ROS Image message to OpenCV image
        depth_frame = bridge.imgmsg_to_cv2(image, "16UC1")

    def tailor(self, task_name):
        rospy.wait_for_service('task_name')
        try:
            print("tailor topic")
            task_definition = rospy.ServiceProxy('task_name', DefineTask)
            done = task_definition(task_name)
            return done
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
 
    def start(self):
        self.gui_msg.data = "start"
        self.pub_gui.publish(self.gui_msg)
        print('gui - start')

    def pause(self): # okay
        self.gui_msg.data = "pause"
        self.pub_gui.publish(self.gui_msg)
        print('gui - pause')
        self.open_pause_window()

    def impact_cb(self,data): # okay 
        impact = data.data
        if impact == True: 
            if self.impact_screen_exis == False:
                self.impact_screen()
                self.impact_screen_exis = True

    def info(self):
        self.gui_msg.data = "gui_info"
        self.pub_gui.publish(self.gui_msg)
        print("gui - info")

################### screen for robot control ################### 
    def impact_screen(self):
        impact_window = ctk.CTkToplevel()
        impact_window.title("Impact")
        impact_window.geometry(f"{int(300*1.4)}x{int(200*1.4)}")

        ctk.CTkLabel(impact_window, text="충돌 감지", font=ctk.CTkFont(size=int(16*1.4))).pack(pady=int(20*1.4))

        button_frame = ctk.CTkFrame(impact_window)
        button_frame.pack(pady=int(10*1.4))

        def Continue():
            print('continue')
            self.gui_msg.data = "continue"
            self.pub_gui.publish(self.gui_msg)
            print('gui - continue')
            impact_window.destroy()     
            self.impact_screen_exis = False    

        def Previous():
            print('previous')
            self.gui_msg.data = "previous"
            self.pub_gui.publish(self.gui_msg)
            print('gui - previous')
            impact_window.destroy()
            self.impact_screen_exis = False

        def Init_pose():
            print('camera_pose')
            self.gui_msg.data = "camera_pose"
            self.pub_gui.publish(self.gui_msg)
            print('gui - init_pose')
            impact_window.destroy()
            self.impact_screen_exis = False

        continue_button = ctk.CTkButton(button_frame, text="계속하기", 
                                        command=with_sound(Continue), width=int(80*1.4))
        continue_button.grid(row=0, column=0, padx=int(10*1.4))

        previous_action_button = ctk.CTkButton(button_frame, text="이전 동작", 
                                               command=with_sound(Previous), width=int(80*1.4))
        previous_action_button.grid(row=0, column=1, padx=int(10*1.4))

        restart_button = ctk.CTkButton(button_frame, text="처음으로", 
                                       command=with_sound(Init_pose), width=int(80*1.4))
        restart_button.grid(row=0, column=2, padx=int(10*1.4))

    def open_pause_window(self):
        pause_window = ctk.CTkToplevel()
        pause_window.title("Paused")
        pause_window.geometry(f"{int(300*1.4)}x{int(200*1.4)}")

        ctk.CTkLabel(pause_window, text="일시 정지 상태입니다", font=ctk.CTkFont(size=int(16*1.4))).pack(pady=int(20*1.4))

        button_frame = ctk.CTkFrame(pause_window)
        button_frame.pack(pady=int(10*1.4))

        def Continue():
            print('continue')
            self.gui_msg.data = "continue"
            self.pub_gui.publish(self.gui_msg)
            print('gui - continue')
            pause_window.destroy()         

        def Previous():
            print('previous')
            self.gui_msg.data = "previous"
            self.pub_gui.publish(self.gui_msg)
            print('gui - previous')
            pause_window.destroy()

        def Init_pose():
            print('init_pose')
            self.gui_msg.data = "camera_pose"
            self.pub_gui.publish(self.gui_msg)
            print('gui - init_pose')
            pause_window.destroy()
            
        continue_button = ctk.CTkButton(button_frame, text="계속하기", 
                                        command=with_sound(Continue), width=int(80*1.4))
        continue_button.grid(row=0, column=0, padx=int(10*1.4))

        previous_action_button = ctk.CTkButton(button_frame, text="이전 동작", 
                                               command=with_sound(Previous), width=int(80*1.4))
        previous_action_button.grid(row=0, column=1, padx=int(10*1.4))

        restart_button = ctk.CTkButton(button_frame, text="처음으로", 
                                       command=with_sound(Init_pose), width=int(80*1.4))
        restart_button.grid(row=0, column=2, padx=int(10*1.4))

###################   for test   ###################
    def define_ready_test(self, data): #
        print("define ready")

    def camera_ready_test(self, data):
        print("camera ready")

    def vision_test(self): 
        self.pub_vision.publish(self.vision_msg)
        print('vision topic')

    def camera_pose_move_test(self):
        msg = Bool()
        msg.data = True
        self.camera_pose.publish(msg)
###############################################################

image_count = 0
robot_arm = Robot_control()

# 사운드 재생 함수
def play_click_sound():
    click_sound.play()

# GUI에서 버튼의 기능과 사운드를 결합하는 함수
def with_sound(func):
    def wrapper(*args, **kwargs):
        play_click_sound()
        return func(*args, **kwargs)
    return wrapper

def show_image_animation(root, on_complete):
    global image_path
    try:
        image = Image.open(image_path)
        original_width, original_height = image.size

        window_width = 558
        window_height = int((original_height / original_width) * window_width)

        screen_width = root.winfo_screenwidth()
        screen_height = root.winfo_screenheight()
        position_top = int(screen_height/2 - window_height/2)
        position_right = int(screen_width/2 - window_width/2)

        root.geometry(f'{window_width}x{window_height}+{position_right}+{position_top}')

        label = ctk.CTkLabel(root, text="", font=ctk.CTkFont(size=int(20*1.4), weight="bold"))
        label.place(relx=0.5, rely=0.5, anchor=ctk.CENTER)

        screen_width = window_width
        screen_height = window_height
        ratio = min(screen_width/original_width, screen_height/original_height)
        new_size = (int(original_width * ratio), int(original_height * ratio))
        resized_image = image.resize(new_size, Image.ANTIALIAS)
        photo = ImageTk.PhotoImage(resized_image)

        label.image = photo
        label.configure(image=photo)

        def fade_in():
            alpha = 0
            while alpha < 1.0:
                label.image = ImageTk.PhotoImage(resized_image)
                label.configure(image=label.image)
                label.update()
                time.sleep(0.01)
                alpha += 0.05

            show_start_button(root, on_complete)  # 페이드 인이 완료되면 시작 버튼 표시

        root.after(0, fade_in)

    except FileNotFoundError:
        print(f"Error: Image file not found at {image_path}")

def show_start_button(root, on_complete):
    ctk.set_appearance_mode("dark")
    ctk.set_default_color_theme("blue")
    start_button = ctk.CTkButton(root, text="실행", font=ctk.CTkFont(size=int(20*1.4)), 
                                 command=with_sound(on_complete), width=200, height=50)
    start_button.place(relx=0.5, rely=0.8, anchor=ctk.CENTER)

def on_start_button_click(root):
    for widget in root.winfo_children():
        widget.destroy()
    main_gui(root)

def main_gui(root):
    global image_path

    ctk.set_appearance_mode("dark")
    ctk.set_default_color_theme("blue")

    root.title("Soomac Taylor")

    image = Image.open(image_path)
    original_width, original_height = image.size

    window_width = 558
    window_height = int((original_height / original_width) * window_width)

    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    position_top = int(screen_height/2 - window_height/2)
    position_right = int(screen_width/2 - window_width/2)

    root.geometry(f'{window_width}x{window_height}+{position_right}+{position_top}')


    title_label = ctk.CTkLabel(root, text="Soomac Task Taylor", font=ctk.CTkFont(size=int(20*1.4), weight="bold"))
    title_label.grid(row=0, column=0, columnspan=2, pady=int(20*1.4))

    def confirm_exit():
        exit_window = ctk.CTkToplevel(root)
        exit_window.title("확인")
        exit_window.geometry(f"{int(300*1.4)}x{int(150*1.4)}")

        label = ctk.CTkLabel(exit_window, text="종료 시 로봇도 함께 종료됩니다", font=ctk.CTkFont(size=int(20)))
        label.pack(pady=int(30))

        def exit_program():
            root.destroy()

        def close_exit_window():
            exit_window.destroy()

        yes_button = ctk.CTkButton(exit_window, text="예", font=ctk.CTkFont(size=int(20)),
                                    command=with_sound(exit_program), height=int(40), width=int(110))
        yes_button.pack(side=ctk.LEFT, padx=int(10*1.4), pady=int(10*1.4))

        no_button = ctk.CTkButton(exit_window, text="아니오", font=ctk.CTkFont(size=int(20)), 
                                  command=with_sound(close_exit_window), height=int(40), width=int(110))
        no_button.pack(side=ctk.RIGHT, padx=int(10*1.4), pady=int(10*1.4))

    def open_task_loader():
        global task_name
        task_loader_window = ctk.CTkToplevel(root)
        task_loader_window.title("Task 불러오기")
        task_loader_window.geometry(f"{int(400*1.4)}x{int(350*1.4)}x300x300")

        selected_task = ctk.StringVar()

        task_list_frame = ctk.CTkScrollableFrame(task_loader_window, width=int(380*1.4), height=int(250*1.4))
        task_list_frame.pack(pady=int(20*1.4), padx=int(10*1.4), fill=ctk.BOTH, expand=True)

        task_folder = Path.home() / "catkin_ws/src/soomac/src/gui/Task"
        task_folder.mkdir(parents=True, exist_ok=True)

        task_dirs = [d.name for d in task_folder.iterdir() if d.is_dir()]
        for task in task_dirs:
            task_radio = ctk.CTkRadioButton(task_list_frame, text=task, variable=selected_task, value=task, command=play_click_sound, font=ctk.CTkFont(size=int(20)))
            task_radio.pack(anchor=ctk.W, pady=int(5*1.4), padx=int(10*1.4))

        def load_selected_task():
            global task_name
            task_name = selected_task.get()
            if task_name:
                processing()  # 선택한 Task 이름을 사용하여 processing 함수 호출
                print(selected_task.get())
            else:
                print("Task가 선택되지 않았습니다")

        button_frame = ctk.CTkFrame(task_loader_window)
        button_frame.pack(pady=int(10*1.4))

        load_button = ctk.CTkButton(button_frame, text="불러오기", font=ctk.CTkFont(size=int(20)), command=lambda:[with_sound(load_selected_task)(), task_loader_window.destroy()], width=int(120*1.4))
        load_button.pack(side=ctk.LEFT, padx=int(10*1.4))

        back_button = ctk.CTkButton(button_frame, text="뒤로가기", font=ctk.CTkFont(size=int(20)), command=with_sound(task_loader_window.destroy), width=int(120*1.4))
        back_button.pack(side=ctk.RIGHT, padx=int(10*1.4))

    buttons = [
        # ("실행", robot_arm.start),
        ("새 Task 정의하기", open_task_definition),
        ("Task 불러오기", open_task_loader),
        ("camera 자세", robot_arm.camera_pose_move_test),
        ("종료", confirm_exit),
        # ("Vision Data (Dev Info)", robot_arm.vision_test),
        ("Vision Data (Dev Info)", dev_info)
    ]

    positions = [
        (1, 0), 
        (2, 0), 
        (2, 1), 
        (4, 1), 
        (4, 0), 
        (3, 1)
    ]

    for i, (text, command) in enumerate(buttons):
        if i == 0:  
            row, col = positions[i]
            button = ctk.CTkButton(root, text=text, command=with_sound(command), width=int(300*1.4), height=int(40*1.4), font=ctk.CTkFont(size=int(20)))
            button.grid(row=row, column=col, padx=0, pady=int(10*1.4), columnspan=2)
        else:  
            row, col = positions[i]
            button = ctk.CTkButton(root, text=text, command=with_sound(command), width=int(180*1.4), height=int(40*1.4), font=ctk.CTkFont(size=int(20)))
            button.grid(row=row, column=col, padx=int(10*1.4), pady=int(16*1.4))

    # label_text = "새 Task 정의하기 버튼을 통해서 나만의 Task를 만들어 작업을 수행해보세요!"
    # label = ctk.CTkLabel(root, text=label_text, font=ctk.CTkFont(size=int(14*1.4)))
    # label.grid(row=row, column=col, padx=int(10*1.4), pady=int(5*1.4))

    info = ctk.CTkLabel(root, text="새 Task 정의하기 버튼을 눌러 나만의 Task를 만들어 작업을 수행해보세요!", font=ctk.CTkFont(size=int(15), weight="bold"))
    info.grid(row=5, column=0, columnspan=2, pady=int(10))

    info2 = ctk.CTkLabel(root, text="Task 불러오기 버튼을 눌러 저장해 두었던 Task를 수행해보세요!", font=ctk.CTkFont(size=int(15), weight="bold"))
    info2.grid(row=6, column=0, columnspan=2, pady=int(10))

    info3 = ctk.CTkLabel(root, text="카메라 자세 버튼을 눌러 Task를 만들어 수행해보세요!", font=ctk.CTkFont(size=int(15), weight="bold"))
    info3.grid(row=7, column=0, columnspan=2, pady=int(10))


def main_screen():
    root = ctk.CTk()
    root.title("Soomac Task Tailor")

    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    window_width = 558
    window_height = 800
    position_top = int(screen_height/2 - window_height/2)
    position_right = int(screen_width/2 - window_width/2)

    root.geometry(f'{window_width}x{window_height}+{position_right}+{position_top}')

    show_image_animation(root, on_complete=lambda: on_start_button_click(root))

    root.mainloop()

def dev_info():
    root = ctk.CTk()
    root.title("Developer Information")

    root.geometry('300x400')
    info1 = ctk.CTkLabel(root, text="[Vision]\n 최윤지 \n오희민", font=ctk.CTkFont(size=int(20), weight="bold"))
    info1.grid(row=0, column=0, columnspan=2, padx=int(20), pady=int(20))

    info2 = ctk.CTkLabel(root, text="[Control]\n 노현우 \n 조준현 \n 마태은 ", font=ctk.CTkFont(size=int(20), weight="bold"))
    info2.grid(row=1, column=0, columnspan=2, padx=int(20), pady=int(20))

    info3 = ctk.CTkLabel(root, text="[Design]\n 최유진 \n 김도윤 \n정서진", font=ctk.CTkFont(size=int(20), weight="bold"))
    info3.grid(row=2, column=0, columnspan=2, padx=int(20), pady=int(20))

def open_task_definition():
    global task_name
    print("새 Task 정의하기 윈도우 열림")

    task_window = ctk.CTkToplevel()
    task_window.title("새 Task 정의하기")
    task_window.geometry(f"{int(650)}x{int(300)}")

    ctk.CTkLabel(task_window, text="Task 이름:", font=ctk.CTkFont(size=int(14*1.4))).grid(row=0, column=0, pady=int(10*1.4), padx=int(10*1.4), sticky=ctk.W)
    task_name_entry = ctk.CTkEntry(task_window, width=int(200*1.4))
    task_name_entry.grid(row=0, column=1, pady=int(10*1.4), padx=int(10*1.4), sticky=ctk.W)

    def on_task_name_change(*args):
        current_text = task_name_var.get()
        task_name_var.set(current_text.replace(' ', '_'))

    task_name_var = ctk.StringVar()
    task_name_var.trace("w", on_task_name_change)
    task_name_entry.configure(textvariable=task_name_var)

    ctk.CTkLabel(task_window, text="반복 방식:", font=ctk.CTkFont(size=int(14*1.4))).grid(row=1, column=0, pady=int(10*1.4), padx=int(10*1.4), sticky=ctk.W)
    repeat_mode_var = ctk.StringVar(value="개수 기반")
    
    repeat_mode_frame = ctk.CTkFrame(task_window)
    repeat_mode_frame.grid(row=1, column=1, pady=int(10*1.4), padx=int(10*1.4), sticky=ctk.W)
    
    repeat_mode_count = ctk.CTkRadioButton(repeat_mode_frame, text="개수 기반", variable=repeat_mode_var, value="개수 기반", font=ctk.CTkFont(size=int(14*1.4)), command=play_click_sound)
    repeat_mode_count.grid(row=0, column=0, padx=int(10*1.4))
    
    repeat_mode_distribution = ctk.CTkRadioButton(repeat_mode_frame, text="분포 기반", variable=repeat_mode_var, value="분포 기반", font=ctk.CTkFont(size=int(14*1.4)), command=play_click_sound)
    repeat_mode_distribution.grid(row=0, column=1, padx=int(10*1.4))

    ctk.CTkLabel(task_window, text="그리퍼 종류:", font=ctk.CTkFont(size=int(14*1.4))).grid(row=2, column=0, pady=int(10*1.4), padx=int(10*1.4), sticky=ctk.W)
    gripper_type_var = ctk.StringVar(value="기계식 그리퍼")

    gripper_type_frame = ctk.CTkFrame(task_window)
    gripper_type_frame.grid(row=2, column=1, pady=int(10*1.4), padx=int(10*1.4), sticky=ctk.W)

    gripper_mechanical = ctk.CTkRadioButton(gripper_type_frame, text="기계식 그리퍼", 
                                            variable=gripper_type_var, value="기계식 그리퍼", font=ctk.CTkFont(size=int(14*1.4)), command=play_click_sound)
    gripper_mechanical.grid(row=0, column=0, padx=int(10*1.4))
    
    gripper_vacuum = ctk.CTkRadioButton(gripper_type_frame, text="진공 그리퍼", 
                                        variable=gripper_type_var, value="진공 그리퍼", font=ctk.CTkFont(size=int(14*1.4)), command=play_click_sound)
    gripper_vacuum.grid(row=0, column=1, padx=int(10*1.4))
    
    gripper_soft = ctk.CTkRadioButton(gripper_type_frame, text="소프트 그리퍼", 
                                      variable=gripper_type_var, value="소프트 그리퍼", font=ctk.CTkFont(size=int(14*1.4)), command=play_click_sound)
    gripper_soft.grid(row=0, column=2, padx=int(10*1.4))

    info3 = ctk.CTkLabel(task_window, text="* 모든 입력란을 채운 후에 저장 및 촬영 버튼을 눌러주세요!", font=ctk.CTkFont(size=int(15), weight="bold"))
    info3.grid(row=4, column=0, columnspan=2, pady=int(5))

    def save_and_capture():
        global image_count
        global task_name
        task_name = task_name_var.get()
        repeat_mode = repeat_mode_var.get()
        gripper_type = gripper_type_var.get()

        save_path = Path.home() / "catkin_ws/src/soomac/src/gui/Task" / task_name

        if save_path.exists():
            warning_window = ctk.CTkToplevel(task_window)
            warning_window.title("경고")
            warning_window.geometry("450x170")

            warning_label = ctk.CTkLabel(warning_window, text="이름이 이미 존재합니다. 다른 이름으로 Task를 정의해주세요",
                                          font=ctk.CTkFont(size=16), text_color="#FFFFFF")
            warning_label.pack(pady=20)

            ok_button = ctk.CTkButton(warning_window, text="확인",fg_color="#FF0000", hover_color="#CC0000",
                                       command=warning_window.destroy, width=100, font=ctk.CTkFont(size=20))
            ok_button.pack(pady=10)
            return

        save_path.mkdir(parents=True, exist_ok=True)

        if task_name and repeat_mode and gripper_type:
            task_window.destroy()
            image_count = 0
            open_camera_window(save_path, task_name)
        else:
            print("모든 입력란을 작성해주세요")

    button_frame = ctk.CTkFrame(task_window)
    button_frame.grid(row=3, column=0, columnspan=2, pady=int(20*1.4), padx=int(20*1.4), sticky=ctk.EW)

    save_button = ctk.CTkButton(button_frame, text="저장 후 촬영",font=ctk.CTkFont(size=int(20)), command=lambda:[play_click_sound(), save_and_capture()], width=int(120*1.4))
    save_button.grid(row=0, column=0, padx=int(10*1.4), pady=int(5), sticky=ctk.W)

    back_button = ctk.CTkButton(button_frame, text="뒤로가기",font=ctk.CTkFont(size=int(20)), command=lambda:[play_click_sound(), task_window.destroy()], width=int(120*1.4))
    back_button.grid(row=0, column=1, padx=int(10*1.4), pady=int(5), sticky=ctk.E)

def open_camera_window(save_path, task_name):
    camera_window = ctk.CTkToplevel()
    camera_window.title("카메라 뷰")
    camera_window.geometry(f"{int(800)}x{int(600)}")

    print("카메라 윈도우 열림")
    # resolution_width, resolution_height = (640, 480)
    # clip_distance_max = 10.00
    # Realsensed435Cam = DepthCamera(resolution_width, resolution_height)
    # depth_scale = Realsensed435Cam.get_depth_scale()

    last_image_path = None  

    video_label = ctk.CTkLabel(camera_window, text="")
    video_label.pack()

    def update_frame():
        # ret, depth_raw_frame, color_raw_frame = Realsensed435Cam.get_raw_frame()
        # if not ret:
        #     print("Unable to get a frame")

        # color_frame = np.asanyarray(color_raw_frame.get_data())
        # depth_frame = np.asanyarray(depth_raw_frame.get_data())

        rgb = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2RGBA)
        img = Image.fromarray(rgb)
        imgtk = ImageTk.PhotoImage(image=img)
        video_label.imgtk = imgtk
        video_label.configure(image=imgtk)
        video_label.after(10, update_frame)

    def capture_image():
        global image_count
        color_path = f"{save_path}/{task_name}_color_{image_count}.png"
        depth_path = f"{save_path}/{task_name}_depth_{image_count}.png"
        npy_path = f"{save_path}/{task_name}_npy_{image_count}.npy"

        # ret, depth_raw_frame, color_raw_frame = Realsensed435Cam.get_raw_frame()
        # if not ret:
        #     print("Unable to get a frame")

        # color_frame = np.asanyarray(color_raw_frame.get_data())
        # depth_frame = np.asanyarray(depth_raw_frame.get_data())
        
        cv2.imwrite(color_path, rgb_frame)
        plt.imsave(depth_path, depth_frame)

        depth_scale = 0.0010000000474974513
        intrinsics = {'fx': 387.5052185058594, 'fy': 387.5052185058594, 'x_offset': 324.73431396484375, 'y_offset': 238.08770751953125, 'img_height': 480, 'img_width': 640}

        rgb = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2RGB)
        xyz = compute_xyz(depth_frame * depth_scale, intrinsics)
        data = save_as_npy(rgb, xyz)
        np.save(npy_path, data)

        print(f"{image_count} Image saved at {color_path}")

        image_count += 1

    def retake_image():
        global image_count
        image_count -= 1
        print(f"{image_count} 사진이 삭제되었습니다")

    def reset_task_images():
        global image_count
        for file in os.listdir(save_path):
            if file.startswith(task_name):
                os.remove(os.path.join(save_path, file))
        print("모든 사진이 삭제되었습니다")

        image_count = 0

        confirm_button = ctk.CTkButton(ask_to_execute, text="확인", command=with_sound(ask_to_execute), width=int(80*1.6))
        confirm_button.pack(pady=int(10*1.6))

    update_frame()

    button_frame = ctk.CTkFrame(camera_window)
    button_frame.pack(side=ctk.BOTTOM, pady=int(20*1.4))

    capture_button = ctk.CTkButton(button_frame, text="촬영", font=ctk.CTkFont(size=int(20)), command=with_sound(capture_image), width=int(100*1.4))
    capture_button.grid(row=0, column=0, padx=int(10*1.4))

    retake_button = ctk.CTkButton(button_frame, text="재촬영", font=ctk.CTkFont(size=int(20)), command=with_sound(retake_image), width=int(100*1.4))
    retake_button.grid(row=0, column=1, padx=int(10*1.4))

    reset_button = ctk.CTkButton(button_frame, text="초기화", font=ctk.CTkFont(size=int(20)), command=with_sound(reset_task_images), width=int(100*1.4))
    reset_button.grid(row=0, column=2, padx=int(10*1.4))

    complete_button = ctk.CTkButton(button_frame, text="완료", font=ctk.CTkFont(size=int(20)), command=lambda:[with_sound(ask_to_execute)(), camera_window.destroy()], width=int(100*1.4))
    complete_button.grid(row=0, column=3, padx=int(10*1.4))

    def on_closing():
        camera_window.destroy()
        # Realsensed435Cam.release()

    camera_window.protocol("WM_DELETE_WINDOW", on_closing)

def processing():
    global task_name
    print("Task 수행 중 윈도우 열림")
    task_name_pub = rospy.Publisher('task_name', String, queue_size=10)

    rospy.sleep(1)
    
    tase_name_msg = String()
    tase_name_msg.data = task_name
    task_name_pub.publish(tase_name_msg)

    execute_window = ctk.CTkToplevel()
    execute_window.title("Task 수행 중")
    execute_window.geometry(f"{int(300*1.4)}x{int(150*1.4)}")

    ctk.CTkLabel(execute_window, text="Task 수행 중 ", font=ctk.CTkFont(size=int(20))).pack(pady=int(20*1.4))

    def close_all_windows():
        for window in execute_window.winfo_children():
            if isinstance(window, ctk.CTkToplevel):
                window.destroy()
        execute_window.destroy()

    stop_button = ctk.CTkButton(execute_window, text="그만하기", font=ctk.CTkFont(size=int(20)), command=with_sound(close_all_windows), width=int(80*1.4))
    stop_button.pack(side=ctk.LEFT, padx=int(10*1.4), pady=int(10*1.4))

    pause_button = ctk.CTkButton(execute_window, text="일시정지", font=ctk.CTkFont(size=int(20)), command=with_sound(robot_arm.pause), width=int(80*1.4))
    pause_button.pack(side=ctk.RIGHT, padx=int(10*1.4), pady=int(10*1.4))

def ask_to_execute():
    print("Task 실행 여부 윈도우 열림")
    task_complete_pub = rospy.Publisher('define_task', String, queue_size=10)
    
    complete_msg = String()
    complete_msg.data = "task complete"
    task_complete_pub.publish(complete_msg)

    execute_window = ctk.CTkToplevel()
    execute_window.title("확인")
    execute_window.geometry(f"{int(300*1.4)}x{int(150*1.4)}")

    ctk.CTkLabel(execute_window, text="Task 정의가 완료되었습니다 \n Task를 바로 실행하시겠습니까?", font=ctk.CTkFont(size=int(20))).pack(pady=int(20*1.4))

    def close_all_windows():
        for window in execute_window.winfo_children():
            if isinstance(window, ctk.CTkToplevel):
                window.destroy()
        execute_window.destroy()

    def with_kill(func): #창을 닫는 함수와 다른 함수를 함께 실행시켜줄 수 있는 코드
        def wrapper(*args, **kwargs):
            close_all_windows()
            return func(*args, **kwargs)
        return wrapper

    yes_button = ctk.CTkButton(execute_window, text="예", font=ctk.CTkFont(size=int(20)), command=with_kill(processing), width=int(80*1.4))
    yes_button.pack(side=ctk.LEFT, padx=int(10*1.4), pady=int(10*1.4)) # processing과 함께 창을 닫기

    no_button = ctk.CTkButton(execute_window, text="아니오", font=ctk.CTkFont(size=int(20)), command=with_sound(close_all_windows), width=int(80*1.4))
    no_button.pack(side=ctk.RIGHT, padx=int(10*1.4), pady=int(10*1.4))

def impact_screen():
    exit_window = ctk.CTkToplevel()
    exit_window.title("충돌 감지")
    exit_window.geometry(f"{int(400*1.4)}x{int(150*1.4)}")

    ctk.CTkLabel(exit_window, text="충돌이 감지되었으니 확인 후 진행해주세요",
                                          font=ctk.CTkFont(size=int(20))).pack(pady=int(20*1.4))
    
    def exit_program():
        exit_window.destroy()

    def close_exit_window():
        pass

    yes_button = ctk.CTkButton(exit_window, text="계속하기", font=ctk.CTkFont(size=int(20)), command=with_sound(exit_program), width=int(100*1.4))
    yes_button.pack(side=ctk.LEFT, padx=(int(50*1.4), int(10*1.4)), pady=int(10*1.4))

    no_button = ctk.CTkButton(exit_window, text="나가기", font=ctk.CTkFont(size=int(20)), command=with_sound(close_exit_window), width=int(100*1.4))
    no_button.pack(side=ctk.RIGHT, padx=(int(10*1.4), int(50*1.4)), pady=int(10*1.4))

if __name__ == "__main__":
    rospy.init_node('soomac_task_tailor_gui', anonymous=True)
    main_screen()