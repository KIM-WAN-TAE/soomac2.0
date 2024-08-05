import numpy as np
import pyrealsense2 as rs
from matplotlib import pyplot as plt
import cv2
from vision.realsense.realsense_depth import DepthCamera
from uois.src.util.utilities import compute_xyz, save_as_npy

resolution_width, resolution_height = (640, 480)

clip_distance_max = 5.00  #remove from the depth image all values above a given value (meters).
                          # Disable by giving negative value (default)

def main():

    Realsensed435Cam = DepthCamera(resolution_width, resolution_height)
    depth_scale = Realsensed435Cam.get_depth_scale()
    k = np.load("/home/choiyj/catkin_ws/src/soomac/src/vision/uois/example_images/OCID_image_0.npy", allow_pickle=True)
    # print(k.shape)
    # print(k.item().get('label'))

    i = 0
    image_root = "/home/choiyj/catkin_ws/src/soomac/src/vision/a/"
    image_name = "test_image_"

    while True:

        ret, depth_raw_frame, color_raw_frame = Realsensed435Cam.get_raw_frame()
        if not ret:
            print("Unable to get a frame")

        color_frame = np.asanyarray(color_raw_frame.get_data())
        depth_frame = np.asanyarray(depth_raw_frame.get_data())
        # print("frame shape:", depth_frame.shape)
        cv2.imshow("Frame",  color_frame )

        key = cv2.waitKey(1) & 0xFF
        if key == ord('a'):
            color_path = image_root + image_name + str(i) + '.png'
            depth_path = image_root + image_name + str(i) + '.png'
            cv2.imwrite(color_path, color_frame)
            plt.imsave(depth_path, depth_frame)

            rgb = cv2.cvtColor(color_frame, cv2.COLOR_BGR2RGB)
            xyz = compute_xyz(depth_frame * depth_scale, Realsensed435Cam.get_camera_intrinsics())
            label = k.item().get('label')
            data = save_as_npy(rgb, xyz, label)

            np_path = image_root + image_name + str(i) + '.npy'
            np.save(np_path, data)
            i += 1

            print(i, " Saved!")

        if key == ord('q'):    
            break
    


if __name__ == '__main__':
    main()
