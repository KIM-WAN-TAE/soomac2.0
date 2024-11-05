from tkinter import *
from tkinter import filedialog
import cv2
import os
from PIL import Image, ImageTk
import rospy
from std_msgs.msg import Float32MultiArray as fl
from std_msgs.msg import Bool, String
import threading

class Robot_control:
    def __init__(self):
        self.pub_vision = rospy.Publisher('vision', fl, queue_size=10) 
        self.pub_gui = rospy.Publisher('task_type', String, queue_size=10)
        self.impact = rospy.Publisher('impact_feedback', Bool, queue_size=10)
        self.start_test = rospy.Publisher('start', Bool, queue_size=10)
        self.goal_pose_test = rospy.Publisher('goal_pose', fl, queue_size=10)        
        
        # gui msg type 정의
        self.gui_msg = String()
        self.gui_msg.data = None
        # vision test용 msg 
        self.vision_msg = fl()
        self.vision_msg.data = [125, 250, 3, 0, 30, # pick : (x, y, z, theta, grip_size) 
                                -125, 250, 3, 0 ] # place : (x, y, z, theta)

    def vision_test(self): 
        self.pub_vision.publish(self.vision_msg)
        print('vision topic')

    def start(self):
        self.gui_msg.data = "start"
        self.pub_gui.publish(self.gui_msg)
        print('gui - start')

    def init_pos(self):
        self.gui_msg.data = "init_pose"
        self.pub_gui.publish(self.gui_msg)
        print('gui - init_pose')
        
        # self.gui_msg.data = "gui_init_pose"
        # self.pub_gui.publish(self.gui_msg)
        # print('gui - init_pos')

    def stop(self):
        self.gui_msg.data = "stop"
        self.pub_gui.publish(self.gui_msg)
        print('gui - stop')
        
    def pause(self):
        self.gui_msg.data = "pause"
        self.pub_gui.publish(self.gui_msg)
        print('gui - pause')
        self.pause_screen()

    def impact_test(self):
        msg = Bool()
        msg.data = True
        self.impact.publish(msg)
        print('impact_feedback')

    def impact_cb(self,data):
        impact = data.data
        if impact == True:
            self.impact_screen()

    def end(self):
        self.gui_msg.data = "end"
        self.pub_gui.publish(self.gui_msg)
        print('gui - end')

    def Continue(self):
        print('continue')
        self.gui_msg.data = "continue"
        self.pub_gui.publish(self.gui_msg)
        print('gui - continue')
        

    def Previous(self):
        print('previous')
        self.gui_msg.data = "previous"
        self.pub_gui.publish(self.gui_msg)
        print('gui - previous')


    def Init_pose(self):
        print('init_pose')
        self.gui_msg.data = "init_pose"
        self.pub_gui.publish(self.gui_msg)
        print('gui - init_pose')


    def impact_screen(self):
        exit_window = Toplevel()
        exit_window.title("충돌 감지")
        exit_window.geometry("400x150")
        exit_window.configure(bg="#e0f7da")

        Label(exit_window, text="충돌이 감지되었습니다, 조치 후 계속하십시오.", bg="#e0f7da", fg="#1b5e20", font=("Helvetica", 14)).pack(pady=20)

        ############### 코드 추가해야할 부분 ###################
        # GUI를 종료하면서 로봇을 거치대로 이동하는 작업을 수행하기 위해서 exit_program 함수에 로봇이 거치대로 이동하는 코드를 추가해야함. 
        def exit_program():
            exit_window.destroy()
            self.Continue()

        def close_exit_window():
            pass

        yes_button = Button(exit_window, text="계속", command=exit_program, bg="#66bb6a", fg="white", activebackground="#388e3c", activeforeground="white", width=10, height=2)
        yes_button.pack(side=LEFT, padx=(50,10), pady=10)

        no_button = Button(exit_window, text="종료", command=close_exit_window, bg="#66bb6a", fg="white", activebackground="#388e3c", activeforeground="white", width=10, height=2)
        no_button.pack(side=RIGHT, padx=(10,50), pady=10)

    def pause_screen(self):
        pause_window = Toplevel()
        pause_window.title("Pause")
        pause_window.geometry("400x150")
        pause_window.configure(bg="#e0f7da")

        Label(pause_window, text="Robot Pause. Please choice next action", bg="#e0f7da", fg="#1b5e20", font=("Helvetica", 14)).pack(pady=20)

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
            self.gui_msg.data = "init_pose"
            self.pub_gui.publish(self.gui_msg)
            print('gui - init_pose')
            pause_window.destroy()


        continue_button = Button(pause_window, text="계속 하기", command=Continue, bg="#66bb6a", fg="white", activebackground="#388e3c", activeforeground="white", width=8, height=2)
        continue_button.pack(side=LEFT, padx=(30,30), pady=10)

        previous_button = Button(pause_window, text="이전", command=Previous, bg="#66bb6a", fg="white", activebackground="#388e3c", activeforeground="white", width=8, height=2)
        previous_button.pack(side=LEFT, padx=(0,30), pady=10)

        init_pose_button = Button(pause_window, text="초기화", command=Init_pose, bg="#66bb6a", fg="white", activebackground="#388e3c", activeforeground="white", width=8, height=2)
        init_pose_button.pack(side=LEFT, pady=10)



# 메인 화면 설정
def main_screen():
    root = Tk()
    root.title("Soomac Taylor")
    root.geometry("420x520")
    root.configure(bg="#e0f7da")  # 연한 초록색 배경
    robot_arm = Robot_control() # 로봇 제어 클래스

    # ros
    rospy.Subscriber('impact_to_gui', Bool, robot_arm.impact_cb)
    rospy.Subscriber('/vision_complete', Bool, robot_arm.vision_test)

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

        ############### 코드 추가해야할 부분 ###################
        # GUI를 종료하면서 로봇을 거치대로 이동하는 작업을 수행하기 위해서 exit_program 함수에 로봇이 거치대로 이동하는 코드를 추가해야함. 
        def exit_program():
            root.destroy()
            robot_arm.end()

        def close_exit_window():
            exit_window.destroy()

        yes_button = Button(exit_window, text="예", command=exit_program, bg="#66bb6a", fg="white", activebackground="#388e3c", activeforeground="white", width=10, height=2)
        yes_button.pack(side=LEFT, padx=10, pady=10)

        no_button = Button(exit_window, text="아니오", command=close_exit_window, bg="#66bb6a", fg="white", activebackground="#388e3c", activeforeground="white", width=10, height=2)
        no_button.pack(side=RIGHT, padx=10, pady=10)

    # 메인 화면 버튼들 배치
    buttons = [
        ("실 행", robot_arm.start), # okay
        ("새 Task 정의", open_task_definition),
        ("Task 불러오기", lambda: None),
        ("종료", confirm_exit),
        ("초기 위치(goal_pos)", robot_arm.init_pos), # okay
        ("일시 정지", robot_arm.pause), # okay
        ("impact_test(로봇 정보)", robot_arm.impact_test), # 테스트용
        ("vision_data(개발자 정보)", robot_arm.vision_test), # 테스트용
        ("긴급 정지", robot_arm.stop), # okay
        # ("종료", confirm_exit),
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
    task_window.geometry("530x260")
    task_window.configure(bg="#e0f7da")

    # Task 이름 입력란
    Label(task_window, text="Task 이름:", bg="#e0f7da", fg="#1b5e20").grid(row=0, column=0, pady=10, padx=10, sticky=W)
    task_name_entry = Entry(task_window, width=30)
    task_name_entry.grid(row=0, column=1, pady=10, padx=10)

    # 반복 방식 입력란
    Label(task_window, text="반복 방식:", bg="#e0f7da", fg="#1b5e20").grid(row=1, column=0, pady=10, padx=10, sticky=W)
    repeat_mode_entry = Entry(task_window, width=30)
    repeat_mode_entry.grid(row=1, column=1, pady=10, padx=10)

    # 저장 경로 입력란
    Label(task_window, text="저장 경로:", bg="#e0f7da", fg="#1b5e20").grid(row=2, column=0, pady=10, padx=10, sticky=W)
    save_path_entry = Entry(task_window, width=30)
    save_path_entry.grid(row=2, column=1, pady=10, padx=10)

    def browse_save_path():
        path = filedialog.askdirectory()
        save_path_entry.insert(0, path)

    browse_button = Button(task_window, text="Browse", command=browse_save_path, bg="#66bb6a", fg="white", activebackground="#388e3c", activeforeground="white", width=10, height=2)
    browse_button.grid(row=2, column=2, pady=10, padx=10)

    # Task 정의 창의 버튼들
    def save_and_capture():
        task_name = task_name_entry.get()
        repeat_mode = repeat_mode_entry.get()
        save_path = save_path_entry.get()
        if task_name and repeat_mode and save_path:
            task_window.destroy()
            open_camera_window(save_path, task_name)
        else:
            print("모든 입력란을 채워주세요.")

    save_button = Button(task_window, text="저장 및 촬영", command=save_and_capture, bg="#66bb6a", fg="white", activebackground="#388e3c", activeforeground="white", width=10, height=2)
    save_button.grid(row=3, column=0, pady=20, padx=10, sticky=W)

    back_button = Button(task_window, text="뒤로 가기", command=task_window.destroy, bg="#66bb6a", fg="white", activebackground="#388e3c", activeforeground="white", width=10, height=2)
    back_button.grid(row=3, column=1, pady=20, padx=10, sticky=E)

# 웹캠 창 열기
def open_camera_window(save_path, task_name):
    camera_window = Toplevel()
    camera_window.title("촬영 화면")
    camera_window.geometry("800x600")
    camera_window.configure(bg="#a5d6a7")  # 초록색 배경

    last_image_path = None  # 마지막으로 촬영한 이미지 경로를 저장

    # 웹캠 영상 표시 레이블
    video_label = Label(camera_window, bg="#a5d6a7")
    video_label.pack()

    # 카메라 캡처 객체 생성
    vid = cv2.VideoCapture(0)

    def update_frame():
        _, frame = vid.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
        img = Image.fromarray(frame)
        imgtk = ImageTk.PhotoImage(image=img)
        video_label.imgtk = imgtk
        video_label.configure(image=imgtk)
        video_label.after(10, update_frame)

    def capture_image():
        nonlocal last_image_path
        image_count = len([name for name in os.listdir(save_path) if name.startswith(task_name) and name.endswith(".png")])
        last_image_path = os.path.join(save_path, f"{task_name}{image_count}.png")
        _, frame = vid.read()
        cv2.imwrite(last_image_path, frame)
        print(f"Image saved at {last_image_path}")

    def retake_image():
        nonlocal last_image_path
        if last_image_path and os.path.exists(last_image_path):
            os.remove(last_image_path)
            print(f"Deleted last image: {last_image_path}")
            last_image_path = None

    def reset_task_images():
        for file in os.listdir(save_path):
            if file.startswith(task_name) and file.endswith(".png"):
                os.remove(os.path.join(save_path, file))
        print("모든 이미지를 삭제했습니다.")
        last_image_path = None

    def complete_task():
        vid.release()
        camera_window.destroy()
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
        vid.release()
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
        pass
        ####################### 코드 추가해야할 부분 ######################## 

    yes_button = Button(execute_window, text="예", command=task_start, bg="#66bb6a", fg="white", activebackground="#388e3c", activeforeground="white", width=10, height=2)
    yes_button.pack(side=LEFT, padx=10, pady=10)

    no_button = Button(execute_window, text="아니오", command=close_all_windows, bg="#66bb6a", fg="white", activebackground="#388e3c", activeforeground="white", width=10, height=2)
    no_button.pack(side=RIGHT, padx=10, pady=10)



if __name__ == "__main__":
    rospy.init_node('GUI', anonymous=True)

    # ros_thread = threading.Thread(target=sub.ros_sub)
    # ros_thread.daemon = True
    # ros_thread.start()
    main_screen()
