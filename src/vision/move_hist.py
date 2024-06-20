import cv2
import numpy as np
import pyrealsense2 as rs
from sklearn.cluster import DBSCAN
from scipy.spatial import distance

WIDTH = 640
HEIGHT = 480
img_num = 3

black_lower = np.array([99, 129, 0])
black_upper = np.array([164, 255, 46])
white_lower = np.array([92, 73, 175])
white_upper = np.array([115, 190, 255])

dbscan = DBSCAN(eps=0.1, min_samples=1)

black_mean_points = []
white_mean_points = []

for i in range(1,1+img_num):
    image = cv2.imread('a/frame_000'+ str(i) +'.jpg')
    
    img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
    
    black_mask = cv2.inRange(img, black_lower, black_upper) 
    white_mask = cv2.inRange(img, white_lower, white_upper) 
    mask = black_mask + white_mask
    
    black_mask_contours, black_hierarchy = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
    white_mask_contours, white_hierarchy = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 

    
    if len(black_mask_contours) != 0:
        for mask_contour in black_mask_contours:
            if cv2.contourArea(mask_contour) > 250:
                x, y, w, h = cv2.boundingRect(mask_contour)
                center = np.array([x+w/2, y+h/2])
                black_mean_points.append(center)
                
    
    if len(white_mask_contours) != 0:
        for mask_contour in white_mask_contours:
            if cv2.contourArea(mask_contour) > 250:
                x, y, w, h = cv2.boundingRect(mask_contour)
                center = np.array([x+w/2, y+h/2])
                white_mean_points.append(center)
                
                

print(black_mean_points)
print(white_mean_points)
for i in range(1, len(black_mean_points)):
    if distance.euclidean(black_mean_points[i-1], black_mean_points[i]) > 0.5:
        print("black cube: ", black_mean_points[i-1], ' -> ', black_mean_points[i])
    if distance.euclidean(white_mean_points[i-1], white_mean_points[i]) > 0.5:
        print("white cube: ", white_mean_points[i-1], ' -> ', white_mean_points[i])