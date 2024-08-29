import numpy as np
import open3d as o3d
import open3d.core as o3c
from sklearn.decomposition import PCA

import os
import sys
sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))))

from control.camera_transformation import transformation_camera


# 기존 코드
img_file = '/home/choiyoonji/catkin_ws/src/soomac/src/vision/a/test_image_1.npy'
d = np.load(img_file, allow_pickle=True, encoding='bytes').item()

rgb_image = d['rgb']
depth_image = d['xyz'].reshape((-1, 3))

pcd = o3d.t.geometry.PointCloud(o3c.Tensor(depth_image, o3c.float32))
downpcd = pcd.voxel_down_sample(voxel_size=0.005)
o3d.visualization.draw(downpcd)

plane_model, inliers = downpcd.segment_plane(distance_threshold=0.01,
                                             ransac_n=3,
                                             num_iterations=500)

inlier_cloud = downpcd.select_by_index(inliers)
outlier_cloud = downpcd.select_by_index(inliers, invert=True)

o3d.visualization.draw(outlier_cloud)
labels = outlier_cloud.cluster_dbscan(eps=0.01, min_points=10, print_progress=True)

# 새로운 코드: 군집 크기 필터링 및 중심점 추출
min_cluster_size = 50  # 최소 군집 크기
max_cluster_size = 50000  # 최대 군집 크기

# 군집 레이블별 포인트 개수를 계산
unique_labels, counts = np.unique(labels, return_counts=True)

# 필터링된 군집의 중심점 및 PCA 결과 저장할 리스트
centroids = []
grasp_poses = []
pick_ind = -1
pick_min = 9999
place_ind = -1
place_min = 9999
object1 = [-0.12839559, -0.12268025]
object2 = [ 0.02387859, -0.07068099]

        
camera_intrinsics = {'fx': 387.5052185058594, 'fy': 387.5052185058594, 'x_offset': 324.73431396484375, 'y_offset': 238.08770751953125, 'img_height': 480, 'img_width': 640}
fx = camera_intrinsics['fx']
fy = camera_intrinsics['fy']
cx = camera_intrinsics['x_offset']
cy = camera_intrinsics['y_offset']

for label, count in zip(unique_labels, counts):
    if min_cluster_size <= count <= max_cluster_size:
        # 해당 군집의 포인트만 선택
        cluster_points = outlier_cloud.select_by_index(np.where(labels == label)[0])
        
        # 중심점 계산
        centroid = np.mean(cluster_points.point.positions.numpy(), axis=0)
        centroids.append(centroid)
        # print(centroid)

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
        gripper_width = aabb_extent[0]  # Grasp 방향(주축)으로의 폭
        
        # PCA 주성분을 Grasp Pose로 사용
        principal_axes = pca.components_
        grasp_pose = {
            'position': centroid,
            'principal_axis': principal_axes[0],  # 주축 (Grasp 방향)
            'secondary_axis': principal_axes[1],  # 수평축 (회전 정의)
            'normal_axis': principal_axes[2],      # 법선축
            'theta':  np.arctan2(principal_axes[0][1], principal_axes[0][0]),
            'grasp_width': gripper_width*1000
        }
        grasp_poses.append(grasp_pose)

print(f"추출된 Grasp Pose 개수: {len(grasp_poses)}")
for i, pose in enumerate(grasp_poses):
    print(f"Grasp Pose {i+1}:")
    print(f"  Position: {pose['position']}")
    # print(f"  Principal Axis (Grasp Direction): {pose['principal_axis']}")
    # print(f"  Secondary Axis: {pose['secondary_axis']}")
    # print(f"  Normal Axis: {pose['normal_axis']}")
    print(f"  theta : {pose['theta']}")
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

# 시각화: 필터링된 군집의 중심점(구)과 Grasp Pose의 화살표
o3d.visualization.draw_geometries([outlier_cloud.to_legacy()] + centroid_spheres + arrows)

pick =grasp_poses[pick_ind]
place =grasp_poses[place_ind]

pick_position = np.array(pick["position"], dtype=float)*1000
place_position = np.array(place["position"], dtype=float)*1000

pick_position = transformation_camera(pick_position)
place_position = transformation_camera(place_position)

print(pick_position)
print(place_position)
# print(pick_ind+1, grasp_poses[pick_ind]["position"], grasp_poses[pick_ind]["theta"])
# print(place_ind+1, grasp_poses[place_ind]["position"], grasp_poses[pick_ind]["theta"])

# import open3d as o3d
# import open3d.core as o3c
# import numpy as np
# import matplotlib.pyplot as plt
# import copy
# import os
# import sys


# # voxel -> ransac -> dbscan


# img_file = '/home/choiyoonji/catkin_ws/src/soomac/src/vision/a/test_image_4.npy'
# d = np.load(img_file, allow_pickle=True, encoding='bytes').item()

# rgb_image = d['rgb']
# depth_image = d['xyz'].reshape((-1,3))

# pcd = o3d.t.geometry.PointCloud(o3c.Tensor(depth_image, o3c.float32))

# print(pcd)
# downpcd = pcd.voxel_down_sample(voxel_size=0.005)
# print(downpcd)


# # o3d.visualization.draw_geometries([downpcd.to_legacy()])

# plane_model, inliers = downpcd.segment_plane(distance_threshold=0.01,
#                                          ransac_n=3,
#                                          num_iterations=500)
# inlier_cloud = downpcd.select_by_index(inliers)
# inlier_cloud = inlier_cloud.paint_uniform_color([1.0, 0, 0])
# outlier_cloud = downpcd.select_by_index(inliers, invert=True)
# # o3d.visualization.draw([inlier_cloud, outlier_cloud])

# labels = outlier_cloud.cluster_dbscan(eps=0.01, min_points=10, print_progress=True)
# print(labels)
# max_label = labels.max().item()
# colors = plt.get_cmap("tab20")(
#         labels.numpy() / (max_label if max_label > 0 else 1))
# colors = o3d.core.Tensor(colors[:, :3], o3d.core.float32)
# colors[labels < 0] = 0
# outlier_cloud.point.colors = colors

# # 새로운 코드: 군집 크기 필터링 및 중심점 추출
# min_cluster_size = 50  # 최소 군집 크기
# max_cluster_size = 1000  # 최대 군집 크기

# # 군집 레이블별 포인트 개수를 계산
# unique_labels, counts = np.unique(labels, return_counts=True)

# # 필터링된 군집의 중심점을 저장할 리스트
# centroids = []

# for label, count in zip(unique_labels, counts):
#     if min_cluster_size <= count <= max_cluster_size:
#         # 해당 군집의 포인트만 선택
#         cluster_points = outlier_cloud.select_by_index(np.where(labels == label)[0])
        
#         # 중심점 계산
#         centroid = np.mean(cluster_points.point.positions.numpy(), axis=0)
#         centroids.append(centroid)

# centroids = np.array(centroids)

# # 결과 출력
# print(f"추출된 중심점 개수: {len(centroids)}")
# print("중심점 좌표:")
# print(centroids)
# sphere_size = 0.01  # 중심점 크기 조절
# centroid_spheres = []

# for centroid in centroids:
#     sphere = o3d.geometry.TriangleMesh.create_sphere(radius=sphere_size)
#     sphere.translate(centroid)  # 중심점을 구 위치로 이동
#     sphere.paint_uniform_color([0, 1, 0])  # 초록색으로 색칠
#     centroid_spheres.append(sphere)
# # 시각화: 필터링된 군집의 중심점과 아웃라이어 포인트 클라우드
# o3d.visualization.draw_geometries([outlier_cloud.to_legacy()] + centroid_spheres)

# # o3d.visualization.draw([outlier_cloud])