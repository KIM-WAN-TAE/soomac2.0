#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
from std_msgs.msg import Float32MultiArray as fl
from std_msgs.msg import Bool, String
from soomac.srv import DefineTask, DefineTaskResponse

import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from tkinter import *
from tkinter import filedialog
import threading
from pathlib import Path

import cv2
from PIL import Image, ImageTk
import pyrealsense2 as rs
import numpy as np
from matplotlib import pyplot as plt

from vision.realsense.realsense_depth import DepthCamera
from vision.realsense.utilities import compute_xyz, save_as_npy

class Robot_control:
    def __init__(self):
        self.pub_vision = rospy.Publisher('vision', fl, queue_size=10) 
        self.pub_gui = rospy.Publisher('task_type', String, queue_size=10)
        # gui msg type 정의
        self.gui_msg = String()
        self.gui_msg.data = None
        # vision test용 msg 
        self.vision_msg = fl()
        self.vision_msg.data = [250, 0, 10, 30, 20, # pick : (x, y, z, theta, grip_size) 
                                400, 0, 10, 60 ] # place : (x, y, z, theta)
        
    def Tailor(self, task_name):
        rospy.wait_for_service('task_name')
        try:
            print("tailor topic")
            task_definition = rospy.ServiceProxy('task_name', DefineTask)
            done = task_definition(task_name)
            return done
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def vision(self):
        self.pub_vision.publish(self.vision_msg)
        print('vision topic')

    def start(self):
        self.gui_msg.data = "gui_start"
        self.pub_gui.publish(self.gui_msg)
        print('gui - start')

    def init_pos(self):
        self.gui_msg.data = "gui_init_pose"
        self.pub_gui.publish(self.gui_msg)
        print('gui - init_pos')

    def stop(self):
        self.gui_msg.data = "gui_stop"
        self.pub_gui.publish(self.gui_msg)
        print('gui - stop')
        
    def pause(self):
        self.gui_msg.data = "gui_pause"
        self.pub_gui.publish(self.gui_msg)
        print('gui - pause')

    def info(self):
        self.gui_msg.data = "gui_info"
        self.pub_gui.publish(self.gui_msg)
        print("gui - info")

class ros_subscribe:
    def __init__(self) -> None:
        pass

    def ros_sub(self):
        print('subscribing')
        rospy.Subscriber('impact_to_gui', Bool, self.callback)
        rospy.spin()  # ROS 이벤트 루프 실행    

    def callback(self, data):
        impact = data.data
        if impact == True:
            impact_screen()


resolution_width, resolution_height = (640, 480)

clip_distance_max = 10.00

Realsensed435Cam = DepthCamera(resolution_width, resolution_height)
depth_scale = Realsensed435Cam.get_depth_scale()

image_count = 0

robot_arm = Robot_control()


# 메인 화면 설정
def main_screen():
    root = Tk()
    root.title("Soomac Taylor")
    root.geometry("420x520")
    root.configure(bg="#e0f7da")  # 연한 초록색 배경
      # 로봇 제어 클래스

    # 타이틀 레이블
    title_label = Label(root, text="Soomac Task Taylor", font=("Helvetica", 20, "bold"), bg="#a5d6a7", fg="#1b5e20")
    title_label.grid(row=0, column=0, columnspan=2, pady=20)

    # 종료 버튼을 누를 때 확인 창을 띄우는 함수
    def confirm_exit():
        exit_window = Toplevel(root)
        exit_window.title("확인")
        exit_window.geometry("300x150")
        exit_window.configure(bg="#e0f7da")

        Label(exit_window, text="로봇의 전원 또한 종료됩니다.", bg="#e0f7da", fg="#1b5e20", font=("Helvetica", 14)).pack(pady=20)

        def exit_program():
            root.destroy()

        def close_exit_window():
            exit_window.destroy()

        yes_button = Button(exit_window, text="예", command=exit_program, bg="#66bb6a", fg="white", activebackground="#388e3c", activeforeground="white", width=10, height=2)
        yes_button.pack(side=LEFT, padx=10, pady=10)

        no_button = Button(exit_window, text="아니오", command=close_exit_window, bg="#66bb6a", fg="white", activebackground="#388e3c", activeforeground="white", width=10, height=2)
        no_button.pack(side=RIGHT, padx=10, pady=10)

    # Task 불러오기 창 열기   
    def open_task_loader():
        task_loader_window = Toplevel(root)
        task_loader_window.title("Task 불러오기")
        task_loader_window.geometry("400x350")
        task_loader_window.configure(bg="#e0f7da")

        selected_task = StringVar()  # 선택된 Task를 저장하는 변수

        # 스크롤 가능한 리스트박스
        task_frame = Frame(task_loader_window)
        task_frame.pack(fill=BOTH, expand=True, padx=10, pady=10)

        scrollbar = Scrollbar(task_frame)
        scrollbar.pack(side=RIGHT, fill=Y)

        listbox_canvas = Canvas(task_frame, width=350, height=200)  # 고정된 크기로 설정
        listbox_canvas.pack(side=LEFT, fill=BOTH, expand=True)

        task_frame_inner = Frame(listbox_canvas, bg="#e0f7da")
        listbox_canvas.create_window((0, 0), window=task_frame_inner, anchor="nw")

        # Task 폴더 경로 ----> 사용할 떄 각자의 경로에 맞게 수정 Path.home()은 home 디렉토리를 의미
        task_folder = Path.home() / "catkin_ws/src/soomac/src/gui/Task"

        # Task 폴더가 존재하지 않으면 생성
        task_folder.mkdir(parents=True, exist_ok=True)

        # Task 폴더에서 Task 폴더 이름을 로드
        task_dirs = [d.name for d in task_folder.iterdir() if d.is_dir()]
        for task in task_dirs:
            Radiobutton(task_frame_inner, text=task, variable=selected_task, value=task, bg="#e0f7da", fg="#1b5e20").pack(anchor=W)

        scrollbar.config(command=listbox_canvas.yview)
        listbox_canvas.config(yscrollcommand=scrollbar.set)

        def load_selected_task():
            task_name = selected_task.get()
            if task_name:
                task_path = task_folder / task_name
                print(f"Selected Task Path: {task_path}") #불러오기 버튼을 클릭했을 때 사용자가 선택한 Task의 폴더 경로를 출력해주는 함수 
            else:
                print("No Task selected")

        # 버튼 프레임 추가
        button_frame = Frame(task_loader_window, bg="#e0f7da")
        button_frame.pack(pady=10)

        load_button = Button(button_frame, text="불러오기", command=load_selected_task, bg="#66bb6a", fg="white", activebackground="#388e3c", activeforeground="white", width=15, height=2)
        load_button.pack(side=LEFT, padx=10)

        back_button = Button(button_frame, text="뒤로 가기", command=task_loader_window.destroy, bg="#66bb6a", fg="white", activebackground="#388e3c", activeforeground="white", width=15, height=2)
        back_button.pack(side=RIGHT, padx=10)

    # 메인 화면 버튼들 배치
    buttons = [
        ("실 행", robot_arm.start),
        ("새 Task 정의", open_task_definition),
        ("Task 불러오기", open_task_loader),
        ("종료", confirm_exit),
        ("초기 위치", robot_arm.init_pos),
        ("일시 정지", robot_arm.pause),
        ("로봇 정보", robot_arm.info),
        ("vision_data(개발자 정보)", robot_arm.vision),  # 테스트용
        ("긴급 정지", robot_arm.stop),
    ]

    positions = [
        (1, 0),  # 실행
        (2, 0),  # 새 Task 정의
        (2, 1),  # Task 불러오기
        (5, 1),  # 종료
        (3, 0),  # 초기 위치
        (3, 1),  # 일시 정지
        (4, 0),  # 로봇 정보
        (4, 1),  # 개발자 정보
        (5, 0),  # 긴급 정지
        (5, 1)   # 종료
    ]

    for i, (text, command) in enumerate(buttons):
        if i == 0:  # 실행 버튼
            row, col = positions[i]
            button = Button(root, text=text, command=command, width=46, height=3, bg="#66bb6a", fg="white", activebackground="#388e3c", activeforeground="white")
            button.grid(row=row, column=col, padx=0, pady=10, columnspan=2)  # columnspan=2 추가
        else:  # 나머지 버튼
            row, col = positions[i]
            button = Button(root, text=text, command=command, width=20, height=3, bg="#66bb6a", fg="white", activebackground="#388e3c", activeforeground="white")
            button.grid(row=row, column=col, padx=10, pady=10)

    root.mainloop()

# Task 정의 창 열기
def open_task_definition():
    task_window = Toplevel()
    task_window.title("새 Task 정의")
    task_window.geometry("530x200")
    task_window.configure(bg="#e0f7da")

    # Task 이름 입력란
    Label(task_window, text="Task 이름:", bg="#e0f7da", fg="#1b5e20").grid(row=0, column=0, pady=10, padx=10, sticky=W)
    task_name_entry = Entry(task_window, width=30)
    task_name_entry.grid(row=0, column=1, pady=10, padx=10)

    # 반복 방식 입력란
    Label(task_window, text="반복 방식:", bg="#e0f7da", fg="#1b5e20").grid(row=1, column=0, pady=10, padx=10, sticky=W)
    repeat_mode_entry = Entry(task_window, width=30)
    repeat_mode_entry.grid(row=1, column=1, pady=10, padx=10)

    # Task 정의 창의 버튼들
    def save_and_capture():
        global image_count
        task_name = task_name_entry.get() #사용자가 지정한 task 이름
        repeat_mode = repeat_mode_entry.get() #사용자가 지정한 반복 방식

        # 자동으로 경로 설정: catkin_ws/src/soomac/src/gui/Task/Task이름
        save_path = Path.home() / "catkin_ws/src/soomac/src/gui/Task" / task_name
        save_path.mkdir(parents=True, exist_ok=True)

        if task_name and repeat_mode:
            task_window.destroy()
            image_count = 0
            open_camera_window(save_path, task_name)
        else:
            print("모든 입력란을 채워주세요.")

    save_button = Button(task_window, text="저장 및 촬영", command=save_and_capture, bg="#66bb6a", fg="white", activebackground="#388e3c", activeforeground="white", width=15, height=2)
    save_button.grid(row=2, column=0, pady=20, padx=10, sticky=W)

    back_button = Button(task_window, text="뒤로 가기", command=task_window.destroy, bg="#66bb6a", fg="white", activebackground="#388e3c", activeforeground="white", width=15, height=2)
    back_button.grid(row=2, column=1, pady=20, padx=10, sticky=E)

# 웹캠 창 열기
def open_camera_window(save_path, task_name):
    camera_window = Toplevel()
    camera_window.title("촬영 화면")
    camera_window.geometry("1400x850")
    camera_window.configure(bg="#a5d6a7")  # 초록색 배경

    last_image_path = None  # 마지막으로 촬영한 이미지 경로를 저장

    # 웹캠 영상 표시 레이블
    video_label = Label(camera_window, bg="#a5d6a7")
    video_label.pack()

    def update_frame():
        ret, depth_raw_frame, color_raw_frame = Realsensed435Cam.get_raw_frame()
        if not ret:
            print("Unable to get a frame")

        color_frame = np.asanyarray(color_raw_frame.get_data())
        depth_frame = np.asanyarray(depth_raw_frame.get_data())

        rgb = cv2.cvtColor(color_frame, cv2.COLOR_BGR2RGBA)
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

        ret, depth_raw_frame, color_raw_frame = Realsensed435Cam.get_raw_frame()
        if not ret:
            print("Unable to get a frame")

        color_frame = np.asanyarray(color_raw_frame.get_data())
        depth_frame = np.asanyarray(depth_raw_frame.get_data())
        
        cv2.imwrite(color_path, color_frame)
        plt.imsave(depth_path, depth_frame)

        rgb = cv2.cvtColor(color_frame, cv2.COLOR_BGR2RGB)
        xyz = compute_xyz(depth_frame * depth_scale, Realsensed435Cam.get_camera_intrinsics())
        data = save_as_npy(rgb, xyz)
        np.save(npy_path, data)

        print(f"{image_count} Image saved at {color_path}")

        image_count += 1

    def retake_image():
        global image_count
        image_count -= 1
        print(f"{image_count} Image deleted.")

    def reset_task_images():
        for file in os.listdir(save_path):
            if file.startswith(task_name):
                os.remove(os.path.join(save_path, file))
        print("모든 이미지를 삭제했습니다.")
        image_count = 0

    def complete_task():
        Realsensed435Cam.release()
        camera_window.destroy()
        Robot_control.Tailor(task_name)
        ask_to_execute()

    update_frame()

    # 촬영 및 추가 버튼들 중앙 배치
    button_frame = Frame(camera_window, bg="#a5d6a7")
    button_frame.pack(side=BOTTOM, pady=20)

    capture_button = Button(button_frame, text="촬영", command=capture_image, bg="#66bb6a", fg="white", activebackground="#388e3c", activeforeground="white", width=10, height=2)
    capture_button.grid(row=0, column=0, padx=10)

    retake_button = Button(button_frame, text="재촬영", command=retake_image, bg="#66bb6a", fg="white", activebackground="#388e3c", activeforeground="white", width=10, height=2)
    retake_button.grid(row=0, column=1, padx=10)

    reset_button = Button(button_frame, text="초기화", command=reset_task_images, bg="#66bb6a", fg="white", activebackground="#388e3c", activeforeground="white", width=10, height=2)
    reset_button.grid(row=0, column=2, padx=10)

    complete_button = Button(button_frame, text="완료", command=complete_task, bg="#66bb6a", fg="white", activebackground="#388e3c", activeforeground="white", width=10, height=2)
    complete_button.grid(row=0, column=3, padx=10)

    def on_closing():
        Realsensed435Cam.release()
        camera_window.destroy()

    camera_window.protocol("WM_DELETE_WINDOW", on_closing)

# "바로 실행하시겠습니까?" 창 열기
def ask_to_execute():
    execute_window = Toplevel()
    execute_window.title("확인")
    execute_window.geometry("300x150")
    execute_window.configure(bg="#e0f7da")

    Label(execute_window, text="바로 실행하시겠습니까?", bg="#e0f7da", fg="#1b5e20", font=("Helvetica", 14)).pack(pady=20)

    def close_all_windows():
        for window in execute_window.winfo_children():
            if isinstance(window, Toplevel):
                window.destroy()
        execute_window.destroy()

    def task_start():
        pass  # 이곳에 로봇을 실행시키는 코드를 추가합니다.

    yes_button = Button(execute_window, text="예", command=task_start, bg="#66bb6a", fg="white", activebackground="#388e3c", activeforeground="white", width=10, height=2)
    yes_button.pack(side=LEFT, padx=10, pady=10)

    no_button = Button(execute_window, text="아니오", command=close_all_windows, bg="#66bb6a", fg="white", activebackground="#388e3c", activeforeground="white", width=10, height=2)
    no_button.pack(side=RIGHT, padx=10, pady=10)

def impact_screen():
    exit_window = Toplevel()
    exit_window.title("충돌 감지")
    exit_window.geometry("400x150")
    exit_window.configure(bg="#e0f7da")

    Label(exit_window, text="충돌이 감지되었습니다, 조치 후 계속하십시오.", bg="#e0f7da", fg="#1b5e20", font=("Helvetica", 14)).pack(pady=20)

    def exit_program():
        exit_window.destroy()

    def close_exit_window():
        pass

    yes_button = Button(exit_window, text="계속", command=exit_program, bg="#66bb6a", fg="white", activebackground="#388e3c", activeforeground="white", width=10, height=2)
    yes_button.pack(side=LEFT, padx=(50, 10), pady=10)

    no_button = Button(exit_window, text="종료", command=close_exit_window, bg="#66bb6a", fg="white", activebackground="#388e3c", activeforeground="white", width=10, height=2)
    no_button.pack(side=RIGHT, padx=(10, 50), pady=10)

if __name__ == "__main__":
    rospy.init_node('GUI', anonymous=True)
    sub = ros_subscribe()
    ros_thread = threading.Thread(target=sub.ros_sub)
    ros_thread.daemon = True
    ros_thread.start()
    main_screen()
