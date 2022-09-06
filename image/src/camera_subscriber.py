#!/usr/bin/env python

import os
import cv2
import glob
import math
import time
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class Subscriber():

    def __init__(self):
        self.br = CvBridge()
        self.sub = rospy.Subscriber("/camera_image", Image, self.antonello)


    def antonello(self, msg):
        #rospy.loginfo("I received an image")
        frame = self.br.imgmsg_to_cv2(msg)
        #cv2.imshow('image', frame)
        #cv2.waitKey(0)


if __name__ == '__main__':
    rospy.init_node("subscriber_node")
    my_node = Subscriber()
    while not rospy.is_shutdown():
        rospy.spin()
