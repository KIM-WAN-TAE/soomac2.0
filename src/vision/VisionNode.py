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

import open3d as o3d
import open3d.core as o3c
import cv2
import numpy as np
import matplotlib.pyplot as plt
from sklearn.decomposition import PCA
import copy
import json
from cv_bridge import CvBridge
import math

from realsense.realsense_depth import DepthCamera
from realsense.utilities import compute_xyz, save_as_npy, compute_xyz_2

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

        self.rs = DepthCamera(resolution_width, resolution_height)
        self.depth_scale = self.rs.get_depth_scale()

        self.task_name = None
        self.coords = []
        self.task = []
        self.step = -1
        self.len_step = -1
        self.place_coord = None

    def save_callback(self, msg):
        ret, depth_raw_frame, color_raw_frame = self.rs.get_raw_frame()
        color_frame = np.asanyarray(color_raw_frame.get_data())
        depth_frame = np.asanyarray(depth_raw_frame.get_data())
        cv2.imwrite(msg.color, color_frame)
        plt.imsave(msg.depth, depth_frame)


        rgb = cv2.cvtColor(color_frame, cv2.COLOR_BGR2RGB)
        xyz = compute_xyz_2(depth_frame * self.depth_scale, self.rs.get_camera_intrinsics())

        data = save_as_npy(rgb, xyz)

        np.save(msg.npy, data)

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

    def load_json(self, task_name):
        self.task_name = task_name

        with open(folder_path+'/'+task_name+'/'+task_name+'.json') as json_file:
            json_data = json.load(json_file)

            print(json_data)

            self.coords = json_data["coords"]
            self.task = json_data["steps"]
            self.len_step = len(self.task)

    def calc_position(self, object1, object2):
        ret, depth_raw_frame, color_raw_frame = self.rs.get_raw_frame()
        depth_frame = np.asanyarray(depth_raw_frame.get_data())
        plt.imsave(folder_path+'test.png', depth_frame)
        
        camera_intrinsics = self.rs.get_camera_intrinsics()  # 카메라 내부 파라미터 가져오기
        fx = camera_intrinsics['fx']
        fy = camera_intrinsics['fy']
        cx = camera_intrinsics['x_offset']
        cy = camera_intrinsics['y_offset']
        xyz = compute_xyz(depth_frame * self.depth_scale, camera_intrinsics).reshape((-1,3))

        pcd = o3d.t.geometry.PointCloud(o3c.Tensor(xyz, o3c.float32))
        downpcd = pcd.voxel_down_sample(voxel_size=0.005)

        plane_model, inliers = downpcd.segment_plane(distance_threshold=0.01,
                                                    ransac_n=3,
                                                    num_iterations=500)

        inlier_cloud = downpcd.select_by_index(inliers)
        outlier_cloud = downpcd.select_by_index(inliers, invert=True)

        labels = outlier_cloud.cluster_dbscan(eps=0.01, min_points=10, print_progress=True)

        # 새로운 코드: 군집 크기 필터링 및 중심점 추출
        min_cluster_size = 50  # 최소 군집 크기
        max_cluster_size = 1000  # 최대 군집 크기

        # 군집 레이블별 포인트 개수를 계산
        unique_labels, counts = np.unique(labels, return_counts=True)

        # 필터링된 군집의 중심점 및 PCA 결과 저장할 리스트
        centroids = []
        grasp_poses = []
        pick_ind = -1
        pick_min = 9999
        place_ind = -1
        place_min = 9999

        for label, count in zip(unique_labels, counts):
            if min_cluster_size <= count <= max_cluster_size:
                # 해당 군집의 포인트만 선택
                cluster_points = outlier_cloud.select_by_index(np.where(labels == label)[0])
                
                # 중심점 계산
                centroid = np.mean(cluster_points.point.positions.numpy(), axis=0)
                centroids.append(centroid)

                X, Y, Z = centroid
                u = (X * fx) / Z + cx
                v = (Y * fy) / Z + cy

                pixel = np.array([u,v])
                print(pixel, object1, object2)

                dis = np.linalg.norm(pixel-object1)
                if dis < pick_min:
                    pick_ind = len(centroids)-1
                    pick_min = dis

                dis = np.linalg.norm(pixel-object2)
                if dis < place_min:
                    place_ind = len(centroids)-1
                    place_min = dis
                
                # PCA를 통해 Grasp Pose 계산
                pca = PCA(n_components=3)
                pca.fit(cluster_points.point.positions.numpy())
                aabb = cluster_points.get_axis_aligned_bounding_box()
                aabb_extent = aabb.get_extent()  # 각 축에 대한 길이 (폭, 높이, 깊이)
                gripper_width = aabb_extent[0]/2.2  # Grasp 방향(주축)으로의 폭
                
                # PCA 주성분을 Grasp Pose로 사용
                principal_axes = pca.components_
                grasp_pose = {
                    'position': centroid,
                    'principal_axis': principal_axes[0],  # 주축 (Grasp 방향)
                    'secondary_axis': principal_axes[1],  # 수평축 (회전 정의)
                    'normal_axis': principal_axes[2],      # 법선축
                    'theta':  math.degrees(np.arctan2(principal_axes[0][1], principal_axes[0][0])),
                    'grasp_width': gripper_width*1000
                }
                grasp_poses.append(grasp_pose)


        # 결과 출력
        print(f"추출된 Grasp Pose 개수: {len(grasp_poses)}")
        for i, pose in enumerate(grasp_poses):
            print(f"Grasp Pose {i+1}:")
            print(f"  Position: {pose['position']}")
            # print(f"  Principal Axis (Grasp Direction): {pose['principal_axis']}")
            # print(f"  Secondary Axis: {pose['secondary_axis']}")
            # print(f"  Normal Axis: {pose['normal_axis']}")
            print(f"  theta : {pose['theta']}")
            # print(f"  grasp_width : {pose['grasp_width']}")

        # 시각화를 위한 코드 (옵션)
        # sphere_size = 0.01  # 중심점 크기 조절
        # centroid_spheres = []

        # for centroid in centroids:
        #     sphere = o3d.geometry.TriangleMesh.create_sphere(radius=sphere_size)
        #     sphere.translate(centroid)  # 중심점을 구 위치로 이동
        #     sphere.paint_uniform_color([0, 1, 0])  # 초록색으로 색칠
        #     centroid_spheres.append(sphere)

        # # Grasp Pose의 주축을 화살표로 시각화
        # arrows = []
        # for pose in grasp_poses:
        #     start_point = pose['position']
        #     end_point = start_point + pose['principal_axis'] * 0.1  # 화살표 길이 조절
        #     direction = pose['principal_axis']
            
        #     # 회전 행렬을 생성하여 화살표 회전
        #     z_axis = np.array([0, 0, 1])
        #     rotation_matrix = np.eye(3)
            
        #     if not np.allclose(direction, z_axis):
        #         axis = np.cross(z_axis, direction)
        #         angle = np.arccos(np.dot(z_axis, direction))
        #         rotation_matrix = o3d.geometry.get_rotation_matrix_from_axis_angle(axis * angle)
            
        #     arrow = o3d.geometry.TriangleMesh.create_arrow(
        #         cylinder_radius=0.01, cone_radius=0.02, cylinder_height=0.1, cone_height=0.02
        #     )
        #     arrow.translate(start_point)
        #     arrow.rotate(rotation_matrix, center=start_point)
        #     arrow.paint_uniform_color([1, 0, 0])  # 빨간색으로 색칠
            
        #     arrows.append(arrow)

        return grasp_poses[pick_ind], grasp_poses[place_ind]


    def coord_pub(self):
        self.step %= (self.len_step)
        print(self.step)
        current_step = self.task[self.step]
    
        object1 = self.coords[0][current_step["pick"]]
        object2 = self.coords[0][current_step["place"]]
        pick, place = self.calc_position(object1, object2)

        pick_position = np.array(pick["position"], dtype=float)*1000
        place_position = np.array(place["position"], dtype=float)*1000

        pick_position = transformation_camera(pick_position)
        place_position = transformation_camera(place_position)

        if -5 < pick_position[0] - place_position[0] < 5:
            if place_position is not None:
                place_position = self.place_coord

        self.place_coord = place_position

        place_position = [ -5.48332222, 237.76662217,  40.07654767]


        print(pick_position)
        print(place_position)

        coord = fl()
        coord.data = [pick_position[0], pick_position[1], pick_position[2]-5,
                      float(pick["theta"]), np.float32(pick["grasp_width"].numpy()),
                      place_position[0], place_position[1], place_position[2],
                      float(place["theta"])]
        print('gripper',np.float32(pick["grasp_width"].numpy()))
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