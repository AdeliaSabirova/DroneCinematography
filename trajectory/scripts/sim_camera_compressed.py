# -*- coding: utf-8 -*-
"""
Created on Mon May 25 17:27:58 2020

@author: robotic
"""

import rospy
import math
import tf
from geometry_msgs.msg import Vector3, Vector3Stamped
import rospy
import std_msgs.msg
from sensor_msgs.msg import CameraInfo, CompressedImage
from trajectory_msgs.msg import MultiDOFJointTrajectory

def camera(data):
    print('ok')
    print(data.data)

def listener():
    #msg = rospy.wait_for_message("/trajectory", PolynomialTrajectory)
    rospy.Subscriber("/sim/camera/compressed", CompressedImage, camera)
    rospy.spin()
    	
		

if __name__ == '__main__':
	try:
         rospy.init_node('sim_camera_check')
         listener()
	except rospy.ROSInterruptException:
		pass