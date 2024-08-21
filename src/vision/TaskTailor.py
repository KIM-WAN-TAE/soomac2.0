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
import utils.Seg2Crop

from siamese_network.model import SiameseNetwork
from siamese_network.dataset import Dataset


class GUI:
    def __init__(self) -> None:
        path_sub = rospy.Subscriber('/task_path', String, self.path_callback, queue_size=1)
        self.task_path = None

    def path_callback(self, msg):
        self.task_path = msg.msg



def main():
    rospy.init_node('task_tailor')
    gui_node = GUI()


if __name__ == "__main__":
    main()