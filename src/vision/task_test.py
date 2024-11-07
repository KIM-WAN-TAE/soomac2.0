
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

from control.camera_transformation import transformation_define
from vision.realsense.realsense_camera import DepthCamera


folder_path = '/home/choiyoonji/catkin_ws/src/soomac/src/gui/Task/'


def parse_obj_info(obj_info, rgb, depth, camera_intrinsics):
    crop_imgs = []
    pixel_coords = []
    world_coords = []

    for i in obj_info:
        x, y, h, w = list(map(int, i))
        crop = rgb[y:y+w, x:x+h, : ]
        crop_imgs.append(crop)
        pixel = [int((x*2+h)/2), int((y*2+w)/2)]
        world = pyrealsense2.rs2_deproject_pixel_to_point(camera_intrinsics, pixel, depth[pixel[1], pixel[0]])
        pixel_coords.append(pixel)
        world_coords.append(world)

    return crop_imgs, pixel_coords, world_coords


class GUI:
    def __init__(self) -> None:
        self.task_name = None

        self.sam = sam_main.SAM2()
        self.siamese = Siamese()
        self.rs = DepthCamera(1280,720)
        self.camera_intrinsic = self.rs.depth_intrinsics
        self.rs.release()

    def path_callback(self, TaskName):
        self.task_name = TaskName
        rgb_png = sorted(glob.glob(os.path.join(folder_path+self.task_name, '*.png')))    
        depth_npy = sorted(glob.glob(os.path.join(folder_path+self.task_name, '*.npy')))    
        
        img_num = int(len(rgb_png)/2)

        for idx, path in enumerate(rgb_png):
            rgb_png[idx] = cv2.imread(path)

        for idx, path in enumerate(depth_npy):
            depth_npy[idx] = np.load(path)

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
                    cv2.imwrite(folder_path+self.task_name+f'/object{i}_{idx}.png', img)

            else:
                for idx, img in enumerate(crop_img):
                    cv2.imwrite(folder_path+self.task_name+f'/object{i}_{idx}.png', img)

                for idx, img in enumerate(object_list):
                    matched_idx = self.object_match(img, crop_img)
                    print(matched_idx)
                    if matched_idx == -1:
                        pixel_coords.append(pixel_coord_list[i-1][idx])
                        world_coords.append(world_coord_list[i-1][idx])
                    else:
                        pixel_coords.append(pixel_coord[matched_idx])
                        world_coords.append(world_coord[matched_idx])


            pixel_coord_list.append(pixel_coords)
            world_coord_list.append(world_coords)

        print('matching finished')

        task["pixel_coords"] = pixel_coord_list
        pixel_coord_list = np.array(pixel_coord_list)
        task["world_coords"] = world_coord_list
        world_coord_list = np.array(world_coord_list)
        # task["objects"] = object_list

        for i in range(1, int(img_num)):
            step = {'pick': 0, 'place': 0}
            if i == 1:
                step['pick'] = 0
                step['place'] = (world_coord_list[1][0] - world_coord_list[0][0]).tolist()
            else:
                dis = np.linalg.norm(pixel_coord_list[i-1][1:] - pixel_coord_list[i][1:], axis=1)
                print(dis)
                pick_ind = np.argmax(dis)+1
                print(pick_ind)
                step['pick'] = int(pick_ind)
                step['place'] = (world_coord_list[i][pick_ind] - world_coord_list[i][0]).tolist()

            task_list.append(step)

        task["steps"] = task_list
        print('define')

        with open(folder_path+self.task_name+'/'+self.task_name+'.json', 'w') as json_file:
            json.dump(task, json_file, ensure_ascii=False, indent=4)

        print('json')


    def object_match(self, object_0, crop):
        max = -1
        matched_idx = -1
        o_shape = np.array(object_0.shape)
        for idx, img in enumerate(crop):
            c_shape = np.array(img.shape)
            size = np.sum(np.abs(o_shape - c_shape))
            similarity = self.siamese.eval(object_0, img)
            print(idx, similarity)
            if similarity > max and size < 20:
                max = similarity
                matched_idx = idx
  
        return matched_idx if max > 0.6 else -1


def main():
    gui_node = GUI()
    gui_node.path_callback('test_new')


if __name__ == "__main__":
    main()