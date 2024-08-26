#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
from std_msgs.msg import Float32MultiArray as fl
from std_msgs.msg import Bool, String
from soomac.srv import DefineTask, DefineTaskResponse

import sys, os
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import customtkinter as ctk  # Import customtkinter
import threading
from pathlib import Path
from PIL import Image, ImageTk
import time

import cv2
from PIL import Image, ImageTk
import pyrealsense2 as rs
import numpy as np
import matplotlib   
from matplotlib import pyplot as plt
matplotlib.use("TkAgg")

from vision.realsense.realsense_depth import DepthCamera
from vision.realsense.utilities import compute_xyz, save_as_npy

class Robot_control:
    def __init__(self):
        self.pub_vision = rospy.Publisher('vision', fl, queue_size=10) 
        self.pub_gui = rospy.Publisher('task_type', String, queue_size=10)
        self.gui_msg = String()
        self.gui_msg.data = None
        self.vision_msg = fl()
        self.vision_msg.data = [250, 0, 10, 30, 20, 
                                400, 0, 10, 60 ]
        
    def tailor(self, task_name):
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
        rospy.spin()    

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

def show_image_animation(root):
    # 지정된 이미지 경로
    image_path = "/home/seojin/catkin_ws/src/soomac/src/gui/start_image.jpg"
    try:
        image = Image.open(image_path)
        original_width, original_height = image.size

        # 이미지 라벨 생성 (기본 텍스트 제거)
        label = ctk.CTkLabel(root, text="")  # 초기 텍스트를 빈 문자열로 설정
        label.place(relx=0.5, rely=0.5, anchor=ctk.CENTER)

        # 이미지 중앙에 맞추기 위한 크기 조정 (비율 유지)
        screen_width = 800
        screen_height = 600
        ratio = min(screen_width/original_width, screen_height/original_height)
        new_size = (int(original_width * ratio), int(original_height * ratio))
        resized_image = image.resize(new_size, Image.ANTIALIAS)
        photo = ImageTk.PhotoImage(resized_image)

        # 페이드 인 애니메이션
        for alpha in range(0, 100, 5):
            label.update()
            image_with_alpha = resized_image.copy()
            image_with_alpha.putalpha(int(alpha * 2.55))
            label.image = ImageTk.PhotoImage(image_with_alpha)
            label.configure(image=label.image)
            time.sleep(0.05)

        # 이미지 표시 시간 (2초)
        time.sleep(2)

        # 페이드 아웃 애니메이션
        for alpha in range(100, 0, -5):
            label.update()
            image_with_alpha = resized_image.copy()
            image_with_alpha.putalpha(int(alpha * 2.55))
            label.image = ImageTk.PhotoImage(image_with_alpha)
            label.configure(image=label.image)
            time.sleep(0.05)

        label.place_forget()  # 애니메이션이 끝난 후 라벨 숨기기

    except FileNotFoundError:
        print(f"Error: Image file not found at {image_path}")

# Main screen setup
def main_screen():
    ctk.set_appearance_mode("dark")
    ctk.set_default_color_theme("green")

    root = ctk.CTk()
    root.title("Soomac Taylor")
    root.geometry("402x520")

    show_image_animation(root)

    # Title label
    title_label = ctk.CTkLabel(root, text="Soomac Task Taylor", font=ctk.CTkFont(size=20, weight="bold"))
    title_label.grid(row=0, column=0, columnspan=2, pady=20)

    # Confirm exit dialog
    def confirm_exit():
        exit_window = ctk.CTkToplevel(root)
        exit_window.title("Confirm")
        exit_window.geometry("300x150")

        label = ctk.CTkLabel(exit_window, text="Do you want to shut down the robot as well?", font=ctk.CTkFont(size=14))
        label.pack(pady=20)

        def exit_program():
            root.destroy()

        def close_exit_window():
            exit_window.destroy()

        yes_button = ctk.CTkButton(exit_window, text="Yes", command=exit_program, width=80)
        yes_button.pack(side=ctk.LEFT, padx=10, pady=10)

        no_button = ctk.CTkButton(exit_window, text="No", command=close_exit_window, width=80)
        no_button.pack(side=ctk.RIGHT, padx=10, pady=10)
    # Open task loader
    def open_task_loader():
        task_loader_window = ctk.CTkToplevel(root)
        task_loader_window.title("Load Task")
        task_loader_window.geometry("400x350")

        selected_task = ctk.StringVar()  

        # 스크롤 가능한 프레임 생성
        task_list_frame = ctk.CTkScrollableFrame(task_loader_window, width=380, height=250)
        task_list_frame.pack(pady=20, padx=10, fill=ctk.BOTH, expand=True)

        task_folder = Path.home() / "catkin_ws/src/soomac/src/gui/Task"
        task_folder.mkdir(parents=True, exist_ok=True)

        task_dirs = [d.name for d in task_folder.iterdir() if d.is_dir()]
        for task in task_dirs:
            task_radio = ctk.CTkRadioButton(task_list_frame, text=task, variable=selected_task, value=task)
            task_radio.pack(anchor=ctk.W, pady=5, padx=10)  # 라디오 버튼 간격 조정

        # Load and Back buttons
        button_frame = ctk.CTkFrame(task_loader_window)
        button_frame.pack(pady=10)

        load_button = ctk.CTkButton(button_frame, text="Load", command=lambda: load_selected_task(selected_task.get()), width=120)
        load_button.pack(side=ctk.LEFT, padx=10)

        back_button = ctk.CTkButton(button_frame, text="Back", command=task_loader_window.destroy, width=120)
        back_button.pack(side=ctk.RIGHT, padx=10)

    def load_selected_task(task_name):
        if task_name:
            task_path = Path.home() / "catkin_ws/src/soomac/src/gui/Task" / task_name
            print(f"Selected Task Path: {task_path}") 
        else:
            print("No Task selected")

    # Main screen buttons layout
    buttons = [
        ("Execute", robot_arm.start),
        ("Define New Task", open_task_definition),
        ("Load Task", open_task_loader),
        ("Exit", confirm_exit),
        ("Initial Position", robot_arm.init_pos),
        ("Pause", robot_arm.pause),
        ("Robot Info", robot_arm.info),
        ("Vision Data (Dev Info)", robot_arm.vision),  
        ("Emergency Stop", robot_arm.stop),
    ]

    positions = [
        (1, 0), 
        (2, 0), 
        (2, 1), 
        (5, 1), 
        (3, 0), 
        (3, 1), 
        (4, 0), 
        (4, 1), 
        (5, 0), 
        (5, 1)
    ]

    for i, (text, command) in enumerate(buttons):
        if i == 0:  
            row, col = positions[i]
            button = ctk.CTkButton(root, text=text, command=command, width=300, height=40)
            button.grid(row=row, column=col, padx=0, pady=10, columnspan=2)
        else:  
            row, col = positions[i]
            button = ctk.CTkButton(root, text=text, command=command, width=180, height=40)
            button.grid(row=row, column=col, padx=10, pady=10)

    root.mainloop()


# Task definition screen
def open_task_definition():
    task_window = ctk.CTkToplevel()
    task_window.title("Define New Task")
    task_window.geometry("600x400")  # 창의 크기를 더 크게 설정

    # Task 이름 입력란
    ctk.CTkLabel(task_window, text="Task Name:").grid(row=0, column=0, pady=10, padx=10, sticky=ctk.W)
    task_name_entry = ctk.CTkEntry(task_window, width=200)
    task_name_entry.grid(row=0, column=1, pady=10, padx=10, sticky=ctk.W)

    # 입력할 때 공백을 언더바로 변환하는 함수
    def on_task_name_change(*args):
        current_text = task_name_var.get()
        task_name_var.set(current_text.replace(' ', '_'))

    # StringVar로 관리하고, 변경 시 위 함수를 호출하도록 설정
    task_name_var = ctk.StringVar()
    task_name_var.trace("w", on_task_name_change)
    task_name_entry.configure(textvariable=task_name_var)  # configure로 수정

    # 반복 방식 선택란
    ctk.CTkLabel(task_window, text="Repeat Mode:").grid(row=1, column=0, pady=10, padx=10, sticky=ctk.W)
    repeat_mode_var = ctk.StringVar(value="Count Based")
    
    repeat_mode_frame = ctk.CTkFrame(task_window)
    repeat_mode_frame.grid(row=1, column=1, pady=10, padx=10, sticky=ctk.W)
    
    repeat_mode_count = ctk.CTkRadioButton(repeat_mode_frame, text="Count Based", variable=repeat_mode_var, value="Count Based")
    repeat_mode_count.grid(row=0, column=0, padx=10)
    
    repeat_mode_distribution = ctk.CTkRadioButton(repeat_mode_frame, text="Distribution Based", variable=repeat_mode_var, value="Distribution Based")
    repeat_mode_distribution.grid(row=0, column=1, padx=10)

    # 그리퍼 종류 선택란
    ctk.CTkLabel(task_window, text="Gripper Type:").grid(row=2, column=0, pady=10, padx=10, sticky=ctk.W)
    gripper_type_var = ctk.StringVar(value="Mechanical Gripper")

    gripper_type_frame = ctk.CTkFrame(task_window)
    gripper_type_frame.grid(row=2, column=1, pady=10, padx=10, sticky=ctk.W)

    gripper_mechanical = ctk.CTkRadioButton(gripper_type_frame, text="Mechanical Gripper", variable=gripper_type_var, value="Mechanical Gripper")
    gripper_mechanical.grid(row=0, column=0, padx=10)
    
    gripper_vacuum = ctk.CTkRadioButton(gripper_type_frame, text="Vacuum Gripper", variable=gripper_type_var, value="Vacuum Gripper")
    gripper_vacuum.grid(row=0, column=1, padx=10)
    
    gripper_soft = ctk.CTkRadioButton(gripper_type_frame, text="Soft Gripper", variable=gripper_type_var, value="Soft Gripper")
    gripper_soft.grid(row=0, column=2, padx=10)

    # Task 정의 창의 버튼들
    def save_and_capture():
        global image_count
        task_name = task_name_var.get()
        repeat_mode = repeat_mode_var.get()
        gripper_type = gripper_type_var.get()

        # 자동으로 경로 설정: catkin_ws/src/soomac/src/gui/Task/Task이름
        save_path = Path.home() / "catkin_ws/src/soomac/src/gui/Task" / task_name
        save_path.mkdir(parents=True, exist_ok=True)

        if task_name and repeat_mode and gripper_type:
            task_window.destroy()
            image_count = 0
            open_camera_window(save_path, task_name)
        else:
            print("Please fill all fields.")

    # 버튼을 포함할 프레임 생성
    button_frame = ctk.CTkFrame(task_window)
    button_frame.grid(row=3, column=0, columnspan=2, pady=20, padx=20, sticky=ctk.EW)  # 전체 열을 차지하도록 조정

    save_button = ctk.CTkButton(button_frame, text="Save and Capture", command=save_and_capture, width=120)
    save_button.grid(row=0, column=0, padx=10, pady=10, sticky=ctk.W)

    back_button = ctk.CTkButton(button_frame, text="Back", command=task_window.destroy, width=120)
    back_button.grid(row=0, column=1, padx=10, pady=10, sticky=ctk.E)


# Camera window
def open_camera_window(save_path, task_name):
    camera_window = ctk.CTkToplevel()
    camera_window.title("Camera View")
    camera_window.geometry("1400x850")

    last_image_path = None  

    video_label = ctk.CTkLabel(camera_window)
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
        print("All images deleted.")
        image_count = 0

    def complete_task():
        camera_window.destroy()
        robot_arm.tailor(task_name=task_name)
        ask_to_execute()

    update_frame()

    button_frame = ctk.CTkFrame(camera_window)
    button_frame.pack(side=ctk.BOTTOM, pady=20)

    capture_button = ctk.CTkButton(button_frame, text="Capture", command=capture_image, width=100)
    capture_button.grid(row=0, column=0, padx=10)

    retake_button = ctk.CTkButton(button_frame, text="Retake", command=retake_image, width=100)
    retake_button.grid(row=0, column=1, padx=10)

    reset_button = ctk.CTkButton(button_frame, text="Reset", command=reset_task_images, width=100)
    reset_button.grid(row=0, column=2, padx=10)

    complete_button = ctk.CTkButton(button_frame, text="Complete", command=complete_task, width=100)
    complete_button.grid(row=0, column=3, padx=10)

    def on_closing():
        camera_window.destroy()

    camera_window.protocol("WM_DELETE_WINDOW", on_closing)

# Ask to execute task
def ask_to_execute():
    execute_window = ctk.CTkToplevel()
    execute_window.title("Confirm")
    execute_window.geometry("300x150")

    ctk.CTkLabel(execute_window, text="Do you want to execute the task now?", font=ctk.CTkFont(size=14)).pack(pady=20)

    def close_all_windows():
        for window in execute_window.winfo_children():
            if isinstance(window, ctk.CTkToplevel):
                window.destroy()
        execute_window.destroy()

    def task_start():
        pass  

    yes_button = ctk.CTkButton(execute_window, text="Yes", command=task_start, width=80)
    yes_button.pack(side=ctk.LEFT, padx=10, pady=10)

    no_button = ctk.CTkButton(execute_window, text="No", command=close_all_windows, width=80)
    no_button.pack(side=ctk.RIGHT, padx=10, pady=10)

def impact_screen():
    exit_window = ctk.CTkToplevel()
    exit_window.title("Collision Detected")
    exit_window.geometry("400x150")

    ctk.CTkLabel(exit_window, text="Collision detected. Please check and continue.", font=ctk.CTkFont(size=14)).pack(pady=20)

    def exit_program():
        exit_window.destroy()

    def close_exit_window():
        pass

    yes_button = ctk.CTkButton(exit_window, text="Continue", command=exit_program, width=100)
    yes_button.pack(side=ctk.LEFT, padx=(50, 10), pady=10)

    no_button = ctk.CTkButton(exit_window, text="Exit", command=close_exit_window, width=100)
    no_button.pack(side=ctk.RIGHT, padx=(10, 50), pady=10)

if __name__ == "__main__":
    rospy.init_node('GUI', anonymous=True)
    sub = ros_subscribe()
    ros_thread = threading.Thread(target=sub.ros_sub)
    ros_thread.daemon = True
    ros_thread.start()
    main_screen()
