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
sys.path.append('/home/choiyoonji/sam2/sam2')
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

import sam_main
from utils.Seg2Crop import extract_objects_from_image

from siamese_network.eval import Siamese

from control.camera_transformation import transformation_define


folder_path = '/home/choiyoonji/catkin_ws/src/soomac/src/gui/Task/'


def parse_obj_info(obj_info, rgb, depth):
    camera_intrinsics = {'fx': 387.5052185058594, 'fy': 387.5052185058594, 'x_offset': 324.73431396484375, 'y_offset': 238.08770751953125, 'img_height': 960, 'img_width': 1280}
    crop_imgs = []
    pixel_coords = []
    world_coords = []

    for i in obj_info:
        crop_img = []
        pixel_coord = []
        world_coord = []
        for j in i['bbox']:
            crop_img.append(rgb[j[1]:j[1]+j[3], j[0]:j[0]+j[2]])
            pixel = [j[0]*2+j[2], j[1]*2+j[3]]
            world = pyrealsense2.rs2_deproject_pixel_to_point(camera_intrinsics, pixel, depth[pixel])
            pixel_coord.append(pixel)
            world_coord.append(world)
            
        crop_imgs.append(crop_img)
        pixel_coords.append(pixel_coord)
        world_coords.append(world_coord)

    return crop_imgs, pixel_coords, world_coords


class GUI:
    def __init__(self) -> None:
        name_sub = rospy.Service('define_task', DefineTask, self.path_callback)
        self.task_name = None

        self.sam = sam_main.SAM2()
        self.siamese = Siamese()

    def path_callback(self, req):
        self.task_name = req.TaskName
        png_files = sorted(glob.glob(os.path.join(folder_path+self.task_name, '*.png')))
        
        img_num = len(png_files)
        rgb_png = png_files[:int(img_num/2)]
        depth_png = png_files[int(img_num/2):]

        task = {}
        task["name"] = self.task_name

        object_list = []
        task_list = []
        pixel_coord_list = []
        world_coord_list = []
        step = {'pick': 0, 'place': 0}

        print("load")

        for i in range(int(img_num/2)):
            pixel_coords = []
            world_coords = []
            obj_info = self.sam.generate_mask(rgb_png[i])

            crop_img, pixel_coord, world_coord = parse_obj_info(obj_info, rgb_png[i], depth_png[i])

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

        for i in range(1, int(img_num/2)):
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