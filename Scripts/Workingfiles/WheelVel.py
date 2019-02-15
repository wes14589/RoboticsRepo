# -*- coding: utf-8 -*-
"""
Created on Fri Feb 15 09:53:31 2019

@author: student
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

from cv_bridge import CvBridge
from std_msgs.msg import String
import cv2

class wheelDrive:
   wheel_radius = 0.05
   robot_radius = 0.25
    
   
   def __init__(self):
      #self.pub = rospy.Publisher('',) # input dir and type
      self.w_left = rospy.Subscribe("/wheel_vel_left",std_msgs/Float32,movewheels)
      self.pub = rospy.Publisher("/mobile_base/commands/velocity",Twist)
        
    


   # computing the forward kinematics for a differential drive
   def forward_kinematics(w_l, w_r):
      c_l = wheel_radius * w_l
      c_r = wheel_radius * w_r
      v = (c_l + c_r) / 2
      a = (c_r - c_l) / (2 * robot_radius)
      return (v, a)


    # computing the inverse kinematics for a differential drive
   def inverse_kinematics(v, a):
      c_l = v - (robot_radius * a)
      c_r = v + (robot_radius * a)
      w_l = c_l / wheel_radius
      w_r = c_r / wheel_radius
      return (w_l, w_r)

    # inverse kinematics from a Twist message (This is what a ROS robot has to do)
   def inverse_kinematics_from_twist(t):
      return inverse_kinematics(t.linear.x, t.angular.z)

   if __name__ == "__main__":

      (w_l, w_r) = inverse_kinematics(0.0, 1.0)
      print "w_l = %f,\tw_r = %f" % (w_l, w_r)

      (v, a) = forward_kinematics(w_l, w_r)
      print "v = %f,\ta = %f" % (v, a)

      from geometry_msgs.msg import Twist
      t = Twist()

      t.linear.x = 0.3
      t.angular.z = 0.8

      (w_l, w_r) = inverse_kinematics_from_twist(t)
      print "w_l = %f,\tw_r = %f" % (w_l, w_r)

        
   def movewheels(lw,rw):
      (v,a) = forward_kinematics(lw,rw)
      twist_msg = Twist()
      twist_msg.linear.x = v
      twist_msg.angular.z = a
      self.pub.pulish(twist_msg)
      waitKey(1)
        


startwindowThread()
rospy.init_node('wheelDrive')
ic = wheelDrive()
rospy.spin()
destroyAllWindows()
