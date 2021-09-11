# -*- coding: utf-8 -*-

#!/usr/bin/env python 
#libraries
import rospy
import math
import tf

#ROS messages
from geometry_msgs.msg import Vector3Stamped
import std_msgs.msg
from sensor_msgs.msg import CameraInfo
from trajectory_msgs.msg import MultiDOFJointTrajectory


def build_camera_info(attributes):
    """
    Private function to compute camera info
    camera info doesn't change over time
    """
    camera_info = CameraInfo()
    # store info without header
    camera_info.header.frame_id = "velodyne"
    camera_info.width = int(attributes['width'])
    camera_info.height = int(attributes['height'])
    camera_info.distortion_model = 'plumb_bob'
    cx = camera_info.width / 2.0
    cy = camera_info.height / 2.0
    fx = camera_info.width / (
        2.0 * math.tan(float(attributes['fov']) * math.pi / 360.0))
    fy = fx
    camera_info.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    camera_info.D = [0, 0, 0, 0, 0]
    camera_info.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
    camera_info.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1.0, 0]
    return camera_info

#function for generation and publishing message
def gimbal(trajectory):
    pub_camera_info = rospy.Publisher("/sim/camera_info", CameraInfo, queue_size=1)
    pub_gimbal_angle = rospy.Publisher('/sim/gimbal_angle', Vector3Stamped, queue_size=1)
    n = len(trajectory.points)
    rate = rospy.Rate(100.0)
    
    attributes = dict()
    attributes['width'] = 1920
    attributes['height'] = 1080
    attributes['fov'] = 26.9914
    camera_info = build_camera_info(attributes)
    
    for i in range(n):
        rx = trajectory.points[i].transforms[0].rotation.x
        ry = trajectory.points[i].transforms[0].rotation.y
        rz = trajectory.points[i].transforms[0].rotation.z
        rw = trajectory.points[i].transforms[0].rotation.w
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([rx,ry,rz,rw])
        msg = Vector3Stamped()
        msg.header = std_msgs.msg.Header()
        msg.header.stamp = rospy.Time.now()
        msg.vector.x = 0
        msg.vector.y = 0
        msg.vector.z = 180 + math.degrees(yaw)
        pub_gimbal_angle.publish(msg)

        rate.sleep()
        camera_info.header.stamp = rospy.Time.now()
        pub_camera_info.publish(camera_info)
         

#subscriber for camera message
def listener():
    rospy.Subscriber("/firefly/command/trajectory", MultiDOFJointTrajectory, gimbal)
    rospy.spin()
    	
		

if __name__ == '__main__':
	try:
         #initialization of the node and name
         rospy.init_node('sim_gimbal')
         listener()
	except rospy.ROSInterruptException:
		pass