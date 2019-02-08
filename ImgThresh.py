# -*- coding: utf-8 -*-
"""
Created on Fri Feb  8 10:37:08 2019

@author: student
"""

import rospy
#from cv2 import namedWindow, cvtColor, imshow
#from cv2 import destroyAllWindows, startWindowThread
import cv2
from cv_bridge import CvBridge
#from cv_bridge import CvBridgeError
#from cv2 import COLOR_BGR2GRAY, waitKey
#from cv2 import blur, Canny
import numpy 
from sensor_msgs.msg import Image
#from cv_bridge import CvBridge


class Follower:

    def __init__(self):

        self.bridge = CvBridge()
        #self.image_sub = rospy.Subscriber("/camera/image_raw",
        #                                  Image, self.callback)
        self.image_sub = rospy.Subscriber(
             "/camera/rgb/image_raw",
             Image, self.callback)

    def callback(self, msg):
        cv2.namedWindow("window", 1)
        image = self.bridge.imgmsg_to_cv2(msg)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([10, 60, 170])
        upper_yellow = numpy.array([255, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        cv2.bitwise_and(image, image, mask=mask)
        cv2.imshow("window", mask)
        cv2.waitKey(1)
        
cv2.startWindowThread()
rospy.init_node('follower')
follower = Follower()
rospy.spin()
cv2.destroyAllWindows()