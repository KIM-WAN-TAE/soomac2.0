import cv2
import numpy as np
from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt
from sklearn.decomposition import PCA

def check_valid(y, w, h):
    if y < 360:
        return 40 < w < 350 and 40 < h < 300
    else:
        return 300 < w < 350 and 200 < h < 300


# 이미지 로드 및 그레이스케일 변환
image = cv2.imread('/home/choiyoonji/catkin_ws/src/soomac/src/gui/Task/test_190/test_190_color_13.png')
# image = cv2.imread('/home/choiyoonji/catkin_ws/src/soomac/src/gui/Task/test_new/test_new_color_0.png')
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
# CLAHE 적용 (Contrast Limited Adaptive Histogram Equalization)
# clahe = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(8, 8))
# equalized_image = clahe.apply(gray)

# 가우시안 블러 적용 (이미지의 노이즈를 줄이기 위해)
blurred_image = cv2.GaussianBlur(gray, (3, 3), 0)  # (5, 5)는 커널 크기, 0은 표준편차

# Canny 엣지 감지
edges = cv2.Canny(blurred_image, threshold1=50, threshold2=135)

# 엣지 포인트의 좌표 추출
points = np.column_stack(np.nonzero(edges))

# DBSCAN 클러스터링 수행
dbscan = DBSCAN(eps=10, min_samples=10).fit(points)
labels = dbscan.labels_

# 각 클러스터에 대해 바운딩 박스 생성 및 그리기
for label in set(labels):
    if label == -1:  # 노이즈는 제외
        continue
    
    cluster_points = points[labels == label]
    y, x, h, w = cv2.boundingRect(cluster_points)
    
    if check_valid(y, w, h):
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        e = edges[y:y+h,x:x+w]
        object_pixels = np.column_stack(np.where(e == 255))
        pca = PCA(n_components=2)
        pca.fit(object_pixels)  

        # 주성분 벡터
        principal_axes = pca.components_

        # 7. 물체의 방향 벡터 시각화
        center = np.mean(object_pixels, axis=0)

        plt.figure(figsize=(8, 8))
        plt.imshow(e, cmap='gray')
        plt.quiver(
            center[1], center[0], 
            principal_axes[0, 1], -principal_axes[0, 0], 
            color='r', scale=5, label='Principal Axis 1'
        )
        plt.quiver(
            center[1], center[0], 
            principal_axes[1, 1], -principal_axes[1, 0], 
            color='b', scale=5, label='Principal Axis 2'
        )
        plt.legend()
        plt.title("Object Direction with PCA")
        plt.show()

# 결과 이미지 표시
cv2.imshow('gray', gray)
cv2.imshow('edge', edges)
cv2.imshow('Clusters with Bounding Boxes', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
