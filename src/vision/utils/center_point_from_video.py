import cv2
import numpy as np
import pyrealsense2 as rs
from sklearn.cluster import DBSCAN

WIDTH = 640
HEIGHT = 480

black_lower = np.array([99, 129, 0])
black_upper = np.array([164, 255, 46])
white_lower = np.array([92, 73, 175])
white_upper = np.array([115, 190, 255])

dbscan = DBSCAN(eps=0.1, min_samples=1)

# webcam_video = cv2.VideoCapture(4)

pipeline = rs.pipeline()
config = rs.config()

pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, 6)
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, 30)

pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)


while True:
    
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    depth_info = depth_frame.as_depth_frame()
    
    color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics # 내부 파라미터
                
    color_image = np.asanyarray(color_frame.get_data())
    color_image = cv2.resize(color_image, (WIDTH, HEIGHT))
    
    img = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV) 
    
    black_mask = cv2.inRange(img, black_lower, black_upper) 
    white_mask = cv2.inRange(img, white_lower, white_upper) 
    mask = black_mask + white_mask
    
    black_mask_contours, black_hierarchy = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
    white_mask_contours, white_hierarchy = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
    
    black_mean_points = []
    
    if len(black_mask_contours) != 0:
        for mask_contour in black_mask_contours:
            if cv2.contourArea(mask_contour) > 250:
                x, y, w, h = cv2.boundingRect(mask_contour)
                z = round((depth_info.get_distance(int(x+w/2), int(y+h/2)) * 100), 2)
                center = [x+w/2, y+h/2, z]
                black_mean_points.append(center)
                
                cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 0, 255), 3)
                cv2.putText(color_image, "[{:.2f}, {:.2f}]".format(float(x+w/2),float(y+h/2)),
                (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (0,0,255), 2) 
                cv2.putText(color_image, "{} cm".format(z), (x + 5, y + 60), 0, 1.0, (0,0,255), 2)
                
    white_mean_points = []
    if len(white_mask_contours) != 0:
        for mask_contour in white_mask_contours:
            if cv2.contourArea(mask_contour) > 250:
                x, y, w, h = cv2.boundingRect(mask_contour)
                z = round((depth_info.get_distance(int(x+w/2), int(y+h/2)) * 100), 2)
                center = [x+w/2, y+h/2, z]
                white_mean_points.append(center)
                
                cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 0, 255), 3)
                cv2.putText(color_image, "[{:.2f}, {:.2f}]".format(float(x+w/2),float(y+h/2)),
                (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (0,0,255), 2) 
                cv2.putText(color_image, "{} cm".format(z), (x + 5, y + 60), 0, 1.0, (0,0,255), 2)
                
    # print()
    
    cv2.imshow("mask image", mask)
    cv2.imshow("window", color_image)
    
    # ESC 키를 누르면 프로그램이 종료됩니다.
    if cv2.waitKey(1) & 0xFF == 27:
        break

# webcam_video.release()
pipeline.stop()
cv2.destroyAllWindows()