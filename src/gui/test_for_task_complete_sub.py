#!/usr/bin/env python
# -- coding: utf-8 --

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(f"Task completion received: {data.data}")

def listener():
    rospy.init_node('task_complete_sub', anonymous=True)
    rospy.Subscriber('task_complete', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
