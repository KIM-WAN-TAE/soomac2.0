#!/usr/bin/env python
# -- coding: utf-8 --

import rospy

from std_msgs.msg import Bool, String
from soomac.msg import PickPlace

import os
import sys
sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import open3d as o3d
import open3d.core as o3c
import numpy as np
import matplotlib.pyplot as plt
from sklearn.decomposition import PCA
import copy
import json

from realsense.realsense_depth import DepthCamera
from realsense.utilities import compute_xyz


folder_path = '/home/choiyj/catkin_ws/src/soomac/src/gui/Task/'
resolution_width, resolution_height = (640, 480)
clip_distance_max = 10.00


class Vision:
    def __init__(self) -> None:
        self.vision_pub = rospy.Publisher('/GraspPose', PickPlace)

        self.rs = DepthCamera()
        self.depth_scale = self.rs.get_depth_scale()

        self.task_name = None
        self.coords = []
        self.task = []
        self.step = -1
        self.len_step = -1

    def load_json(self, task_name):
        self.task_name = task_name

        with open('json file path or name') as json_file:
            json_data = json.load(json_file)

            self.coords = json_data["coords"]
            self.task = json_data["steps"]
            self.len_step = len(self.task)

    def calc_position(self, object1, object2):
        ret, depth_raw_frame, color_raw_frame = self.rs.get_raw_frame()
        depth_frame = np.asanyarray(depth_raw_frame.get_data())
        xyz = compute_xyz(depth_frame * self.depth_scale, self.rs.get_camera_intrinsics()).reshape((-1,3))

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

                dis = np.linalg.norm(centroid-object1)
                if dis < pick_min:
                    pick_ind = len(centroids)-1
                    pick_min = dis

                dis = np.linalg.norm(centroid-object2)
                if dis < place_min:
                    place_ind = len(centroids)-1
                    place_min = dis
                
                # PCA를 통해 Grasp Pose 계산
                pca = PCA(n_components=3)
                pca.fit(cluster_points.point.positions.numpy())
                
                # PCA 주성분을 Grasp Pose로 사용
                principal_axes = pca.components_
                grasp_pose = {
                    'position': centroid,
                    'principal_axis': principal_axes[0],  # 주축 (Grasp 방향)
                    'secondary_axis': principal_axes[1],  # 수평축 (회전 정의)
                    'normal_axis': principal_axes[2],      # 법선축
                    'theta':  np.arctan2(principal_axes[0][1], principal_axes[0][0])
                }
                grasp_poses.append(grasp_pose)

        # 결과 출력
        print(f"추출된 Grasp Pose 개수: {len(grasp_poses)}")
        for i, pose in enumerate(grasp_poses):
            print(f"Grasp Pose {i+1}:")
            print(f"  Position: {pose['position']}")
            print(f"  Principal Axis (Grasp Direction): {pose['principal_axis']}")
            print(f"  Secondary Axis: {pose['secondary_axis']}")
            print(f"  Normal Axis: {pose['normal_axis']}")

        # 시각화를 위한 코드 (옵션)
        sphere_size = 0.01  # 중심점 크기 조절
        centroid_spheres = []

        for centroid in centroids:
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=sphere_size)
            sphere.translate(centroid)  # 중심점을 구 위치로 이동
            sphere.paint_uniform_color([0, 1, 0])  # 초록색으로 색칠
            centroid_spheres.append(sphere)

        # Grasp Pose의 주축을 화살표로 시각화
        arrows = []
        for pose in grasp_poses:
            start_point = pose['position']
            end_point = start_point + pose['principal_axis'] * 0.1  # 화살표 길이 조절
            direction = pose['principal_axis']
            
            # 회전 행렬을 생성하여 화살표 회전
            z_axis = np.array([0, 0, 1])
            rotation_matrix = np.eye(3)
            
            if not np.allclose(direction, z_axis):
                axis = np.cross(z_axis, direction)
                angle = np.arccos(np.dot(z_axis, direction))
                rotation_matrix = o3d.geometry.get_rotation_matrix_from_axis_angle(axis * angle)
            
            arrow = o3d.geometry.TriangleMesh.create_arrow(
                cylinder_radius=0.01, cone_radius=0.02, cylinder_height=0.1, cone_height=0.02
            )
            arrow.translate(start_point)
            arrow.rotate(rotation_matrix, center=start_point)
            arrow.paint_uniform_color([1, 0, 0])  # 빨간색으로 색칠
            
            arrows.append(arrow)

        return grasp_poses[pick_ind], grasp_poses[place_ind]


    def coord_pub(self):
        self.step %= self.len_step
        current_step = self.task[self.step]
    
        object1 = self.coords[0][current_step["Pick"]]
        object2 = self.coords[0][current_step["Place"]]
        pick, place = self.calc_position(object1, object2)

        coord = PickPlace()
        coord.Pick = [pick["position"][0], pick["position"][1], pick["position"][2], pick["theta"]]
        coord.Place = [place["position"][0], place["position"][1], place["position"][2], place["theta"]]

    def reset(self):
        self.task_name = None
        self.coords = []
        self.task = []
        self.step = -1
        self.len_step = -1


class Info:
    def __init__(self) -> None:
        self.vision = Vision()

        name_sub = rospy.Subscriber('task_name', String, self.name_callback)
        robot_sub = rospy.Subscriber('', Bool, self.robot_callback)
        task_sub = rospy.Subscriber('', Bool, self.task_callback)

        self.robot_pub = rospy.Publisher('/PerformPose', Bool)

        self.task_name = None
        self.robot_ready = False
        self.task_stop = False

    def name_callback(self, msg):
        self.task_name = msg.msg
        self.robot_pub.publish(True)
        self.vision.step += 1
        self.vision.load_json()

    def robot_callback(self, msg):
        self.task_name = msg.msg
        self.vision.coord_pub()

    def task_callback(self, msg):
        self.task_name = msg.msg
        self.vision.reset()



def main():
    rospy.init_node("vision_node")

    vision = Vision()

    info = Info()

    rospy.spin()

if __name__ == "__main__":
    main()