#!/usr/bin/env python
# -- coding: utf-8 --

import cv2
import pyrealsense2 as rs
import numpy as np
import json
import rospy
from geometry_msgs.msg import Point, Pose

from time import time

WIDTH = 640
HEIGHT = 480


class Depth_Camera():

    def __init__(self):
        self.possub = rospy.Subscriber('/robot_pos', Pose, self.callback_robot, queue_size=1)
        self.pospub = rospy.Publisher('/target_position', Point, queue_size=1)

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.align = None
        self.align_to = None
        self.detect = False
        self.position = Point()

        # robot pos
        self.robot_pos = Pose()
        self.translate_vector = np.array([0, 0.2, 0.3])
        self.rotate_degree = -90

        context = rs.context()
        connect_device = None
        if context.devices[0].get_info(rs.camera_info.name).lower() != 'platform camera':
            connect_device = context.devices[0].get_info(rs.camera_info.serial_number)

        print(" > Serial number : {}".format(connect_device))
        self.config.enable_device(connect_device)
        self.config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, 30)

    def callback_robot(self, msg):
        self.robot_pos = msg.data
        self.update_T()
        self.update_R()
    
    def update_T(self):
        self.translate_vector = np.array([self.robot_pos.position.x, self.robot_pos.position.y, self.robot_pos.position.z])

    def update_R(self):
        q = [self.robot_pos.orientation.x, self.robot_pos.orientation.y, self.robot_pos.orientation.z, self.robot_pos.orientation.w]
        self.rotation_matrix = self.quaternion_rotation_matrix(q)
    
    def quaternion_rotation_matrix(self, Q):
        q0 = Q[0]
        q1 = Q[1]
        q2 = Q[2]
        q3 = Q[3]
        
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        rot_matrix = np.array([[r00, r10, r20],
                            [r01, r11, r21],
                            [r02, r12, r22]])

        return rot_matrix

    def rotate_x(self,vector, degree):
        # Degree to Radian
        rad = np.radians(degree)
        
        # Rotation Matrix
        rotation_matrix = np.array([
            [1, 0, 0],
            [0, np.cos(rad), -np.sin(rad)],
            [0, np.sin(rad), np.cos(rad)]
        ])
        
        return np.dot(rotation_matrix, vector)
    
    def rotation(self, vector, rotation):
        return np.dot(rotation, vector)
    
    def translate(self,vector, translation):
        return vector + translation

    def execute(self):
        print('Collecting depth information...')
        
        try:
            self.pipeline.start(self.config)
        except:
            print("There is no signal sended from depth camera.")
            print("Check connection status of camera.")
            return
        
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        try:
            while True:

                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                depth_info = depth_frame.as_depth_frame()

                color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics # 내부 파라미터
                
                color_image = np.asanyarray(color_frame.get_data())
                color_image = cv2.resize(color_image, (WIDTH, HEIGHT)) # 원본으로 resize해야 world 좌표가 잘 나옴
                # 객체 인지도 1280 x 720이 더 잘 됨
                
                results = model(color_image, stream=True)

                class_ids = []
                confidences = []
                bboxes = []
                obj_centers = []

                for result in results:
                    boxes = result.boxes
                    for box in boxes:
                        confidence = box.conf
                        if confidence > 0.5 :
                            xyxy = box.xyxy.tolist()[0]
                            bboxes.append(xyxy)
                            confidences.append(float(confidence))
                            class_ids.append(box.cls.tolist())
                            cx = int((xyxy[2]+xyxy[0])//2)
                            cy = int((xyxy[3]+xyxy[1])//2)
                            obj_centers.append([cx,cy]) # 중심
                            

                result_boxes = cv2.dnn.NMSBoxes(bboxes, confidences, 0.25, 0.45, 0.5)
  
                font = cv2.FONT_HERSHEY_PLAIN
                
                for i in range(len(bboxes)):
                    label = str(CLASSES[int(class_ids[i][0])])

                    # if label == 'Screw Driver':
                    if i in result_boxes:
                        bbox = list(map(int, bboxes[i])) 
                        x, y, x2, y2 = bbox
                        cx, cy = obj_centers[i]
                        
                        print("Depth : ", round((depth_info.get_distance(cx, cy) * 100), 2), "cm")

                        depth = round((depth_info.get_distance(cx, cy) * 100), 2)

                        if depth <= 10:
                            continue
                        
                        wx, wy, wz = rs.rs2_deproject_pixel_to_point(color_intrinsics, [cx, cy], depth)
                        # wx, wy, wz가 real world coordinator
                        # 단위는 cm
                        
                        print(wx, wy, wz)

                        vector = np.array([wx, wz, -wy])

                        # x축을 중심으로 d도 회전
                        rotated_vector = self.rotate_x(vector, self.rotate_degree)

                        translated_vector = self.translate(rotated_vector, self.translate_vector)
                        
                    try:
                        color = colors[i]
                        color = (int(color[0]), int(color[1]), int(color[2]), int(color[3]), int(color[4]))
                    except:
                        print("Error")
                    if label != "bottle":
                        color = (0,255,0)
                        cv2.rectangle(color_image, (x, y), (x2, y2), color, 4)
                        cv2.putText(color_image, "{} cm".format(depth), (x + 5, y + 60), 0, 1.0, color, 2)
                        cv2.putText(color_image, label, (x, y + 30), font, 3, color, 3)
                    
                cv2.imshow('image', color_image)
                #cv2.imwrite('./valueup_detect.png', color_image)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            self.pipeline.stop()
        print("┌──────────────────────────────────────┐")
        print('│ Collecting of depth info is stopped. │')
        print("└──────────────────────────────────────┘")

if __name__ == "__main__":
    rospy.init_node('camera')
    depth_camera = Depth_Camera()
    depth_camera.execute()