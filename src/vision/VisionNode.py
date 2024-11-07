#!/usr/bin/env python
# -- coding: utf-8 --

import rospy

from std_msgs.msg import Bool, String
from std_msgs.msg import Float32MultiArray as fl
from sensor_msgs.msg import Image
from soomac.msg import PickPlace, image

import os
import sys
sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import glob

import cv2
import numpy as np
import matplotlib.pyplot as plt
import copy
import json
from cv_bridge import CvBridge
import math
from sklearn.cluster import DBSCAN
from sklearn.decomposition import PCA
import pyrealsense2 as rs2

from siamese_network.eval import Siamese

from vision.realsense.realsense_camera import DepthCamera
from control.camera_transformation import transformation_camera


folder_path = '/home/choiyoonji/catkin_ws/src/soomac/src/gui/Task/'
resolution_width, resolution_height = (640, 480)
clip_distance_max = 10.00


class Vision:
    def __init__(self) -> None:
        self.vision_pub = rospy.Publisher('/vision', fl)
        self.rgb_pub = rospy.Publisher('/rgb_frame', Image)
        self.depth_pub = rospy.Publisher('/depth_frame', Image)
        self.save_sub = rospy.Subscriber('/save_img', image, self.save_callback)

        self.siamese = Siamese()

        self.rs = DepthCamera(resolution_width, resolution_height)
        self.depth_scale = self.rs.get_depth_scale()

        self.task_name = None
        self.coords = []
        self.task = []
        self.step = -1
        self.len_step = -1
        self.place_coord = None
        self.objects_img = []

    def image_pub(self):
        bridge = CvBridge()
        ret, depth_raw_frame, color_raw_frame = self.rs.get_raw_frame()
        if not ret:
            print("Unable to get a frame")
            return

        color_frame = np.asanyarray(color_raw_frame.get_data())
        depth_frame = np.asanyarray(depth_raw_frame.get_data())

        rgb_image = bridge.cv2_to_imgmsg(color_frame)
        depth_image = bridge.cv2_to_imgmsg(depth_frame)
        self.rgb_pub.publish(rgb_image)
        self.depth_pub.publish(depth_image)

    def save_callback(self, msg):
        ret, depth_raw_frame, color_raw_frame = self.rs.get_raw_frame()
        color_frame = np.asanyarray(color_raw_frame.get_data())
        depth_frame = np.asanyarray(depth_raw_frame.get_data())
        cv2.imwrite(msg.color, color_frame)
        plt.imsave(msg.depth[:-3]+'png', depth_frame)
        np.save(msg.depth, depth_frame)
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_frame, alpha=0.15), cv2.COLORMAP_JET)
        
        # origin_images = np.vstack((color_frame, depth_colormap))
        # resized_images = cv2.resize(origin_images, (640, 480*2))

        # cv2.imshow('Camera and Yolo Detection', resized_images)

    def load_json(self, task_name):
        self.task_name = task_name
        self.objects_img = []

        rgb_png = sorted(glob.glob(os.path.join(folder_path+'/'+self.task_name+'/', 'object0_*.png')))    
        for idx, path in enumerate(rgb_png):
            self.objects_img.append(cv2.imread(path))

        with open(folder_path+'/'+task_name+'/'+task_name+'.json') as json_file:
            json_data = json.load(json_file)

            print(json_data)

            self.task = json_data["steps"]
            self.len_step = len(self.task)
            self.case_place_coord = self.task[0]['place']

    def calc_position_obj(self, object1):
        ret, depth_raw_frame, color_raw_frame = self.rs.get_raw_frame()
        color_frame = np.asanyarray(color_raw_frame.get_data())
        depth_frame = np.asanyarray(depth_raw_frame.get_data())

        def check_valid(y, w, h):
            if object1 != 0:
                return y < 360 and 40 < w < 350 and 40 < h < 300
            else:
                return y >= 360 and 300 < w < 350 and 200 < h < 300

        gray = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)
        blurred_image = cv2.GaussianBlur(gray, (3, 3), 0)

        edges = cv2.Canny(blurred_image, threshold1=50, threshold2=135)
        points = np.column_stack(np.nonzero(edges))

        dbscan = DBSCAN(eps=10, min_samples=10).fit(points)
        labels = dbscan.labels_
                
        bbox = []
        max_similarity = -1
        for label in set(labels):
            if label == -1:  # 노이즈는 제외
                continue
            
            cluster_points = points[labels == label]
            y, x, h, w = cv2.boundingRect(cluster_points)
            
            if check_valid(y, w, h):
                crop = color_frame[x:x+w, y:y+w]
                similarity = self.siamese.eval(self.objects_img[object1], crop)
                if similarity > max_similarity:
                    bbox = [x, y, w, h]
                    max_similarity = similarity

        pick_coord = [int(bbox[0]+bbox[2]/2), int(bbox[1]+bbox[3]/2)]
        pick_coord = rs2.rs2_deproject_pixel_to_point(self.rs.depth_intrinsics, pick_coord, depth_frame[pick_coord[0],pick_coord[1]])
        
        x, y, w, h = bbox
        e = edges[y:y+h,x:x+w]
        object_pixels = np.column_stack(np.where(e == 255))
        pca = PCA(n_components=2)
        pca.fit(object_pixels)  

        # 주성분 벡터
        principal_axes = pca.components_
        first_axis = principal_axes[1]
        angle_radians = np.arctan2(first_axis[1], first_axis[0])  # y축, x축
        angle_degrees = np.degrees(angle_radians)

        return pick_coord, angle_degrees

    def coord_pub(self):
        if self.step >= self.len_step:
            self.step = 0
        # self.step %= (self.len_step)
        print(self.step)
        current_step = self.task[self.step]
    
        object1 = current_step['pick']
        place_coord = current_step['place']
        pick, theta = self.calc_position_obj(object1)
        
        if object1 == 0:
            place = self.case_place_coord
        else:
            place = self.case_place_coord

        pick_position = np.array(pick, dtype=float)
        place_position = np.array(place, dtype=float)

        print(pick_position)
        print(place_position)

        pick_position = transformation_camera(pick_position)
        place_position = transformation_camera(place_position)

        print(pick_position)
        print(place_position)

        coord = fl()
        if object1 == 0:
            coord.data = [pick_position[0]+74, pick_position[1], 25,
                        0, float(-1),
                        place_position[0]+74, place_position[1], 25,
                        0]
            print('gripper',float(-1))
        else:
            coord.data = [pick_position[0], pick_position[1], 20,
                        theta, float(30),
                        place_position[0], place_position[1], 20,
                        theta]
            
        self.vision_pub.publish(coord)

    def reset(self):
        self.task_name = None
        self.coords = []
        self.task = []
        self.step = -1
        self.len_step = -1


class Info:
    def __init__(self) -> None:
        self.vision = Vision()

        name_sub = rospy.Subscriber('/task_name', String, self.name_callback)
        robot_sub = rospy.Subscriber('/camera_ready', Bool, self.robot_callback)
        task_sub = rospy.Subscriber('/task_complete', Bool, self.task_callback)

        self.robot_pub = rospy.Publisher('/camera_pose', Bool)

        self.task_name = None
        self.robot_ready = False
        self.task_stop = False

    def name_callback(self, msg):
        self.task_name = msg.data
        self.robot_pub.publish(True)
        self.vision.load_json(self.task_name)

    def robot_callback(self, msg):
        self.task_name = msg.data
        rospy.sleep(1.5)
        self.vision.step += 1
        self.vision.coord_pub()

    def task_callback(self, msg):
        self.task_name = msg.data
        self.vision.reset()

    def image_pub(self):
        self.vision.image_pub()



def main():
    rospy.init_node("vision_node")
    rate = rospy.Rate(30)
    info = Info()

    while not rospy.is_shutdown():
        info.image_pub()
        rate.sleep()

if __name__ == "__main__":
    main()