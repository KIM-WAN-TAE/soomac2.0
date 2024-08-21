import open3d as o3d
import open3d.core as o3c
import numpy as np
import matplotlib.pyplot as plt
import copy
import os
import sys


# voxel -> ransac -> dbscan


img_file = '/home/choiyj/catkin_ws/src/soomac/src/vision/a/test/test_image_3.npy'
d = np.load(img_file, allow_pickle=True, encoding='bytes').item()

rgb_image = d['rgb']
depth_image = d['xyz'].reshape((-1,3))

pcd = o3d.t.geometry.PointCloud(o3c.Tensor(depth_image, o3c.float32))

print(pcd)
downpcd = pcd.voxel_down_sample(voxel_size=0.02)
print(downpcd)


# o3d.visualization.draw_geometries([downpcd.to_legacy()])

plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                         ransac_n=3,
                                         num_iterations=500)
inlier_cloud = pcd.select_by_index(inliers)
inlier_cloud = inlier_cloud.paint_uniform_color([1.0, 0, 0])
outlier_cloud = pcd.select_by_index(inliers, invert=True)
# o3d.visualization.draw([inlier_cloud, outlier_cloud])

labels = outlier_cloud.cluster_dbscan(eps=0.01, min_points=10, print_progress=True)

max_label = labels.max().item()
colors = plt.get_cmap("tab20")(
        labels.numpy() / (max_label if max_label > 0 else 1))
colors = o3d.core.Tensor(colors[:, :3], o3d.core.float32)
colors[labels < 0] = 0
outlier_cloud.point.colors = colors
o3d.visualization.draw([outlier_cloud])