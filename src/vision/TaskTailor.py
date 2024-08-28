#!/usr/bin/env python
# -- coding: utf-8 --

import rospy

from std_msgs.msg import String

import os
os.environ['CUDA_VISIBLE_DEVICES'] = "0" # TODO: Change this if you have more than 1 GPU

import sys
import json
from time import time
import glob

import cv2
import numpy as np
import matplotlib.pyplot as plt

import torch
from torch.utils.data import DataLoader
from torchvision import transforms

from uois.Uois import Uois
from utils.Seg2Crop import extract_objects_from_image

from siamese_network.eval import Siamese


folder_path = '/home/choiyoonji/catkin_ws/src/soomac/src/gui/Task/'


class GUI:
    def __init__(self) -> None:
        self.path_sub = rospy.Subscriber('/task_name', String, self.path_callback, queue_size=1)
        self.done_pub = rospy.Publisher('/done', bool, queue_size=1)
        self.task_name = None

        self.uois = Uois()
        self.siamese = Siamese()

    def path_callback(self, msg):
        self.task_name = msg.msg
        npy_files = sorted(glob.glob(os.path.join(folder_path+self.task_name, '*.png')))

        # rgb_list = []
        # seg_list = []
        # crop_list = []

        object_list = []
        coord_list = []
        task_list = []
        task = {'pick': 0, 'place': 0}

        for i, img in enumerate(npy_files):
            rgb, seg = self.uois.run(img, folder_path+self.task_name+'/segmask_'+str(i)+'.png')
            seg = cv2.imread(folder_path+self.task_name+'/segmask_'+str(i)+'.png', cv2.IMREAD_GRAYSCALE)
            cropped_images = extract_objects_from_image(rgb, seg)
            coord = []

            if i == 0:
                for idx, img in enumerate(cropped_images):
                    object_list[idx] = img
                    coord.append(img[0])
                    cv2.imwrite(folder_path+self.task_name+f'/object/object_{idx}.png', img[1])

            else:
                object_img = cv2.imread(folder_path+self.task_name+f'/object/object_{idx}.png', cv2.IMREAD_COLOR)
                for idx, img in enumerate(object_list):
                    coord.append(self.object_match(img,cropped_images))

            coord_list.append(coord)

        coord_list = np.array(coord_list)

        for i in len(npy_files-1):
            dis = np.linalg.norm(coord_list[i] - coord_list[i+1], axis=1)
            pick_ind = np.argmax(dis)
            task['pick'] = pick_ind

            dis = np.linalg.norm(coord_list[i] - coord_list[i+1][pick_ind], axis=1)
            place_ind = np.argmin(dis)
            task['place'] = place_ind

            task_list.append(task)

        with open(folder_path+self.task_name+'.json', 'w') as json_file:
            json.dump(task_list, json_file, ensure_ascii=False, indent=4)

        self.done_pub.publish(True)

    def object_match(self, object_0, crop):
        max = -1
        coord = [0,0]
        for img in crop:
            similarity = self.uois(object_0, img[1])
            if similarity > max:
                max = similarity
                coord = img[0]

        return coord
    
        
def main():
    rospy.init_node('task_tailor')
    gui_node = GUI()
    rospy.spin()


if __name__ == "__main__":
    main()