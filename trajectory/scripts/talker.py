#!/usr/bin/env python

import rospy
import math
import tf
import numpy as np
from mav_msgs.msg import RollPitchYawrateThrust
from geometry_msgs.msg import Vector3, PoseStamped, Pose, Point, Quaternion, Transform, Twist
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
import rospy
import std_msgs.msg
from decimal import Decimal
from dji_sdk.srv import SDKControlAuthority



def talker():
	pub = rospy.Publisher('/firefly/command/trajectory', MultiDOFJointTrajectory, queue_size = 10)

	rate = rospy.Rate(10)
	
	
	trajectory = MultiDOFJointTrajectory()
	trajectory.header.stamp = rospy.Time.now()
	trajectory.header.frame_id = 'frame'
	trajectory.joint_names.append('base_link')
	i = 0
	drone = open("/home/robotic/catkin_ws/src/dji_adelia/Data.txt","r")
	camera = open("/home/robotic/catkin_ws/src/dji_adelia/Camera.txt","r")
	velocity = open("/home/robotic/catkin_ws/src/dji_adelia/DataVelocity.txt","r")
	drone_coordinates = []
	camera_coordinates = []
	velocity_coordinates = []
	drone_lines = drone.readlines()
	camera_lines = camera.readlines()
	velocity_lines = velocity.readlines()
	for line in drone_lines:
		vector_drone = [float(x) for x in line.split()]
		drone_coordinates.append(vector_drone)
	for line in camera_lines:
		vector_camera = [float(x) for x in line.split()]
		camera_coordinates.append(vector_camera)
	for line in velocity_lines:
		vector_velocity = [float(x) for x in line.split()]
		velocity_coordinates.append(vector_velocity)
	transforms = Transform()
	velocities = Twist()
	accelerations = Twist()
	point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Time())
	trajectory.points.append(point)
	t = 0.25
	r = 0.19
	for i in range(len(drone_coordinates)):
		transforms = Transform()
		velocities = Twist()
		velocities.linear.x = velocity_coordinates[i][0]
		velocities.linear.y = velocity_coordinates[i][1]
		velocities.linear.z = velocity_coordinates[i][2]
		accelerations = Twist()
		transforms.translation.x = -1*drone_coordinates[i][0]
		transforms.translation.y = -1*drone_coordinates[i][1]
		transforms.translation.z = drone_coordinates[i][2]
		
		dx = -1*camera_coordinates[i][0] + drone_coordinates[i][0]
		dy = -1*camera_coordinates[i][1] + drone_coordinates[i][1]
		angle = dx / math.sqrt(dx*dx+dy*dy)
		if dy >= 0:
			yaw = math.acos(angle)
		else:
			yaw = -1*math.acos(angle)
		#print(yaw)
		#velocities.angular.z = yaw
		#velocities.angular.x = velocity_coordinates[i][0]/r
		#velocities.angular.y = velocity_coordinates[i][1]/r
		#velocities.angular.z = velocity_coordinates[i][2]/r
		
		quat = tf.transformations.quaternion_from_euler(0.0,0.0,yaw)
		r = tf.transformations.euler_matrix(0.0,0.0,yaw)
		r = np.asarray(r)
		r = np.transpose(r)
		rd = np.array([[-math.sin(yaw), -math.cos(yaw), math.cos(yaw)+math.sin(yaw), 0],
				[math.cos(yaw), -math.sin(yaw), math.sin(yaw)-math.cos(yaw), 0],	
				[-1, 1, 0, 0],
				[0, 0, 0, 0]])
		omega = np.dot(rd,r)
		w = np.array([omega[2,1], omega[0,2], omega[1,0], 0])
		#w = np.dot(r,w)

		velocities.angular.x = w[0]
		velocities.angular.y = w[1]
		velocities.angular.z = w[2]

		transforms.rotation.x = quat[0]
		transforms.rotation.y = quat[1]
		transforms.rotation.z = quat[2]
		transforms.rotation.w = quat[3]
	
		point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Time((t)))
		t += 0.25
		trajectory.points.append(point)
		
	rospy.sleep(1)
	
	pub.publish(trajectory)


def obtain_control():
	print('obtain_control')
	#rospy.wait_for_service('/dji_sdk/sdk_control_authority')
	try:
		sdk_ctrl_authority_service = rospy.ServiceProxy('/dji_sdk/sdk_control_authority', SDKControlAuthority)

		response = sdk_ctrl_authority_service.call(1)
		print response
		
		if not response.result:
			return False
		else:
			return True

		
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
		return False
		
		

if __name__ == '__main__':
	rospy.init_node('talker', anonymous = False)
	try:
		
		#while not obtain_control():
		#	rospy.sleep(0.25)
		print('aaa')
		#if obtain_control():	
		talker()
	except rospy.ROSInterruptException:
		pass
