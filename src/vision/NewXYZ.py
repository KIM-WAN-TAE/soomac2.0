import numpy as np
import cv2

# 깊이 이미지 로드 (8비트 또는 16비트 PNG 파일 등)
depth_image = cv2.imread('/home/choiyoonji/catkin_ws/src/soomac/src/vision/a/test_depth_0.png', cv2.IMREAD_GRAYSCALE)
camera_intrinsics = {'fx': 387.5052185058594, 'fy': 387.5052185058594, 'x_offset': 324.73431396484375, 'y_offset': 238.08770751953125, 'img_height': 480, 'img_width': 640}
fx = camera_intrinsics['fx']
fy = camera_intrinsics['fy']
cx = camera_intrinsics['x_offset']
cy = camera_intrinsics['y_offset']

# 포인트 클라우드 생성
height, width = (480, 640)
points = []
print(depth_image.shape)

for v in range(height):
    for u in range(width):
        z = depth_image[v, u]  # 깊이 값 (예: 밀리미터를 미터로 변환)
        if z == 0:  # 깊이 값이 0이면 포인트를 무시 (센서에서 해당 값이 없는 경우)
            continue
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        points.append([x, y, z])

# 포인트 클라우드를 NumPy 배열로 변환
point_cloud = np.array(points)
img_file = '/home/choiyoonji/catkin_ws/src/soomac/src/vision/a/test_image_1.npy'
d = np.load(img_file, allow_pickle=True, encoding='bytes').item()
depth_image = d['xyz'].reshape((-1, 3))
print(point_cloud)
print(depth_image)