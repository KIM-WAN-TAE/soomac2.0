import numpy as np
import pyrealsense2 as rs
from matplotlib import pyplot as plt
import cv2
from realsense_depth import DepthCamera

resolution_width, resolution_height = (640, 480)

clip_distance_max = 5.00  #remove from the depth image all values above a given value (meters).
                          # Disable by giving negative value (default)

def main():

    Realsensed435Cam = DepthCamera(resolution_width, resolution_height)
    depth_scale = Realsensed435Cam.get_depth_scale()

    while True:

        ret, depth_raw_frame, color_raw_frame = Realsensed435Cam.get_raw_frame()
        if not ret:
            print("Unable to get a frame")
        color_frame = np.asanyarray(color_raw_frame.get_data())
        depth_frame = np.asanyarray(depth_raw_frame.get_data())
        print("frame shape:", depth_frame.shape)
        cv2.imshow("Frame",  color_frame )
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cv2.imwrite("frame_color_cube.png", color_frame)
            plt.imsave("frame_depth_cube.png", depth_frame)
            break
    


if __name__ == '__main__':
    main()
