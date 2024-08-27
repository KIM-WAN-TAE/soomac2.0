#!/usr/bin/env python
# -- coding: utf-8 --

import rospy

from std_msgs.msg import Bool, String


folder_path = '/home/choiyj/catkin_ws/src/soomac/src/gui/Task/'

class Vision:
    def __init__(self) -> None:
        name_sub = rospy.Service('task_name', String, self.task_callback)
        self.task_name = None

    def task_callback(self, msg):
        self.task_name = msg.msg

    