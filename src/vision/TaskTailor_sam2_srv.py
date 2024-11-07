#!/usr/bin/env python
# -- coding: utf-8 --

import rospy

from std_msgs.msg import String
from soomac.srv import DefineTask, DefineTaskResponse

import os
os.environ['CUDA_VISIBLE_DEVICES'] = "0" # TODO: Change this if you have more than 1 GPU

import sys
sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
sys.path.append('/home/choiyoonji/sam2')
import json
from time import time
import glob

import cv2
import numpy as np
import matplotlib.pyplot as plt
import pyrealsense2

import torch
from torch.utils.data import DataLoader
from torchvision import transforms

import sam2.sam_main as sam_main

from siamese_network.eval import Siamese

from control.camera_transformation import transformation_camera
from vision.realsense.realsense_camera import DepthCamera


folder_path = '/home/choiyoonji/catkin_ws/src/soomac/src/gui/Task/'


def parse_obj_info(obj_info, rgb, depth, camera_intrinsics):
    crop_imgs = []
    pixel_coords = []
    world_coords = []

    print(rgb.shape, depth.shape)
    print(camera_intrinsics)
    for i in obj_info:
        x, y, h, w = list(map(int, i))
        crop = rgb[y:y+w, x:x+h, : ]
        crop_imgs.append(crop)
        pixel = [int((x*2+h)/2), int((y*2+w)/2)]
        world = transformation_camera(pyrealsense2.rs2_deproject_pixel_to_point(camera_intrinsics, pixel, depth[pixel[0], pixel[1]]))
        pixel_coords.append(pixel)
        world_coords.append(world)

    return crop_imgs, pixel_coords, world_coords


class GUI:
    def __init__(self) -> None:
        name_sub = rospy.Service('define_task', DefineTask, self.path_callback)
        self.task_name = None

        self.sam = sam_main.SAM2()
        self.siamese = Siamese()
        self.rs = DepthCamera(1280,960)
        self.camera_intrinsic = self.rs.get_camera_intrinsics()
        self.rs.release()
        print('===========================================')
        print("Realsense Released")
        print('===========================================')

    def path_callback(self, req):
        self.task_name = req.TaskName
        rgb_png = sorted(glob.glob(os.path.join(folder_path+self.task_name, '*.png')))    
        depth_npy = sorted(glob.glob(os.path.join(folder_path+self.task_name, '*.npy')))    
        
        img_num = len(rgb_png)

        for idx, path in enumerate(rgb_png):
            rgb_png[idx] = cv2.imread(path)

        for idx, path in enumerate(depth_npy):
            depth_npy[idx] = cv2.imread(path, cv2.IMREAD_UNCHANGED)

        task = {}
        task["name"] = self.task_name

        object_list = []
        task_list = []
        pixel_coord_list = []
        world_coord_list = []
        step = {'pick': 0, 'place': 0}

        print("load")
        print(img_num)
        for i in range(img_num):
            pixel_coords = []
            world_coords = []
            obj_info = self.sam.generate_mask(rgb_png[i])
            print(obj_info)
            print(i)

            crop_img, pixel_coord, world_coord = parse_obj_info(obj_info, rgb_png[i], depth_npy[i], self.camera_intrinsic)

            if i == 0:
                for idx, img in enumerate(crop_img):
                    object_list.append(img)
                    pixel_coords.append(pixel_coord[idx])
                    world_coords.append(world_coord[idx])
                    cv2.imwrite(folder_path+self.task_name+f'/object/object_{idx}.png', img)

            else:
                for idx, img in enumerate(object_list):
                    matched_idx = self.object_match(img, crop_img)
                    if matched_idx == -1:
                        pixel_coords.append([i-1][idx])
                        world_coords.append(world_coord_list[i-1][idx])
                    else:
                        pixel_coords.append(pixel_coord[matched_idx])
                        world_coords.append(world_coord[matched_idx])


            pixel_coord_list.append(world_coords)
            world_coord_list.append(world_coords)

        print('matching finished')

        task["pixel_coords"] = pixel_coord_list
        pixel_coord_list = np.array(pixel_coord_list)
        task["world_coords"] = world_coord_list
        world_coord_list = np.array(world_coord_list)
        # task["objects"] = object_list

        for i in range(1, int(img_num)):
            step = {'pick': 0, 'place': 0}
            dis = np.linalg.norm(pixel_coord_list[i-1] - pixel_coord_list[i], axis=1)
            pick_ind = np.argmax(dis)
            step['pick'] = int(pick_ind)

            if i == 1:
                step['place'] = world_coord_list[1][0] - world_coord_list[0][0]
            else:
                step['place'] = world_coord_list[i][pick_ind] - world_coord_list[i][0]

            task_list.append(step)

        task["steps"] = task_list
        print('define')
        
        with open(folder_path+self.task_name+'/'+self.task_name+'.json', 'w') as json_file:
            json.dump(task, json_file, ensure_ascii=False, indent=4)

        print('json')

        return DefineTaskResponse(True)

    def object_match(self, object_0, crop):
        max = -1
        matched_idx = -1
        for idx, img in enumerate(crop):
            similarity = self.siamese.eval(object_0, img)
            if similarity > max:
                max = similarity
                matched_idx = idx
  
        return matched_idx if max > 0.6 else -1


def main():
    rospy.init_node('task_tailor')
    gui_node = GUI()
    rospy.spin()


if __name__ == "__main__":
    main()