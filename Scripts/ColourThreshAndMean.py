# -*- coding: utf-8 -*-
"""
Created on Fri Feb  8 09:57:38 2019

@author: student
"""

#!/usr/bin/env python

import rospy
import cv2
import numpy
from cv2 import namedWindow, cvtColor, imshow
from cv2 import destroyAllWindows, startWindowThread
from cv2 import COLOR_BGR2GRAY, waitKey
from cv2 import blur, Canny
from numpy import mean
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String


class image_converter:

    def __init__(self):

        self.bridge = CvBridge()
        self.pub=rospy.Publisher('/result_topic', String)        
        self.image_sub = rospy.Subscriber(
            "/camera/rgb/image_raw",
            Image, self.callback)

    def callback(self, msg):
         cv2.namedWindow("window", 1)
         image = self.bridge.imgmsg_to_cv2(msg)
         hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
         lower_yellow = numpy.array([10, 60, 10])
         upper_yellow = numpy.array([255, 255, 255])
         mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
         cv2.bitwise_and(image, image, mask=mask)
         cv2.imshow("window", mask)
         cv2.waitKey(1)
         
         a = numpy.mean(hsv[:, :, 0])
         
         hello_str = a
         rospy.loginfo(hello_str)
         self.pub.publish(hello_str)        
    

startWindowThread()
rospy.init_node('image_converter')
ic = image_converter()
rospy.spin()

destroyAllWindows()
