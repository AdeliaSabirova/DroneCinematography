#!/usr/bin/env python


import rospy
import math
import tf
from geometry_msgs.msg import Vector3, Vector3Stamped
import rospy
import std_msgs.msg
from dji_sdk.msg import Gimbal
from mav_planning_msgs.msg import PolynomialTrajectory


def gimbal():
	pub = rospy.Publisher('/dji_sdk/gimbal_angle_cmd', Gimbal, queue_size = 10)
	rospy.init_node('gimbal', anonymous = False)
	rate = rospy.Rate(10)
	
	gimbal = Gimbal()
	gimbal.header.stamp = rospy.Time.now()
	gimbal.header.frame_id = 'frame'
	i = 0
	drone = open("/home/robotic/catkin_ws/src/dji_adelia/DroneCoordinates.txt","r")
	camera = open("/home/robotic/catkin_ws/src/dji_adelia/CameraCoordinates.txt","r")
	drone_coordinates = []
	camera_coordinates = []
	drone_lines = drone.readlines()
	camera_lines = camera.readlines()
	for line in drone_lines:
		vector_drone = [float(x) for x in line.split()]
		drone_coordinates.append(vector_drone)
	for line in camera_lines:
		vector_camera = [float(x) for x in line.split()]
		camera_coordinates.append(vector_camera)
	for i in range(len(drone_coordinates)):
		dalt = camera_coordinates[i][2] - drone_coordinates[i][2]
		dy = math.sqrt((-1*camera_coordinates[i][0] + drone_coordinates[i][0])**2+(-1*camera_coordinates[i][1] + drone_coordinates[i][1])**2)
		angle = math.atan2(dalt,dy);
			
		gimbal.ts = 2
		gimbal.mode = 1
		gimbal.roll = 0.
		gimbal.pitch = angle
		gimbal.yaw = 0.
		rospy.sleep(0.5)
		pub.publish(gimbal)
		#rospy.sleep(1)
		


def obtain_control():
	rospy.wait_for_service('/dji_sdk/sdk_control_authority')
	try:
		sdk_ctrl_authority_service = rospy.ServiceProxy('/dji_sdk/sdk_control_authority', SDKControlAuthority)

		response = sdk_ctrl_authority_service.call(1)
		#print response
		
		if not response.result:
			return False
		else:
			return True

		
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e		
		

if __name__ == '__main__':
	try:
		#while not obtain_control():
		#	rospy.rostime.wallsleep(0.25)
		gimbal()
	except rospy.ROSInterruptException:
		pass
