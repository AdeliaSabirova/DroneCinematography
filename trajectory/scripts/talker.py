#!/usr/bin/env python

import rospy
import math
import tf
import numpy as np
from mav_msgs.msg import RollPitchYawrateThrust
from geometry_msgs.msg import Vector3, PoseStamped, Pose, Point, Quaternion, Transform, Twist
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
import rospy
from std_msgs.msg import String
from decimal import Decimal
from dji_sdk.srv import SDKControlAuthority


string = " "

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    string = data.data

#read coordinates from files
def read_from_file(filename):
	read_points = []
	points = open(filename, "r")
	points_lines = points.readlines()
	for line in points_lines:
		vector = [float(x) for x in line.split()]
		read_points.append(vector)
	return np.asarray(read_points)

#initialization function of coordinates fromfile
def initialization():
	drone_coordinates = read_from_file("/home/robotic/catkin_ws/src/dji_adelia/DroneCoordinates.txt")
	camera_coordinates = read_from_file("/home/robotic/catkin_ws/src/dji_adelia/CameraCoordinates.txt")
	drone_velocity = read_from_file("/home/robotic/catkin_ws/src/dji_adelia/DroneVelocity.txt")
	return drone_coordinates, camera_coordinates, velocity_coordinates


def talker():
    pub = rospy.Publisher('/inspire1/command/trajectory', MultiDOFJointTrajectory, queue_size = 10)
    rate = rospy.Rate(10)
    
    drone_coordinates, camera_coordinates, velocity_coordinates = initialization()
    
    trajectory = MultiDOFJointTrajectory()
    trajectory.header.stamp = rospy.Time.now()
    trajectory.header.frame_id = 'frame'
    trajectory.joint_names.append('base_link')
    transforms = Transform()
    velocities = Twist()
    accelerations = Twist()
    point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Time())
    trajectory.points.append(point)
    
    t = 0.25
    for i in range(0, len(drone_coordinates)):
        
        transforms = Transform()
        velocities = Twist()
        velocities.linear.x = velocity_coordinates[i,0]
        velocities.linear.y = velocity_coordinates[i,1]
        velocities.linear.z = velocity_coordinates[i,2]
        accelerations = Twist()
        transforms.translation.x = -1*drone_coordinates[i,0]
        transforms.translation.y = -1*drone_coordinates[i,1]
        transforms.translation.z = drone_coordinates[i,2]

 	dx = -1*camera_coordinates[i][0] + drone_coordinates[i][0]
 	dy = -1*camera_coordinates[i][1] + drone_coordinates[i][1]
 	angle = dx / math.sqrt(dx*dx+dy*dy)
 	if dy >= 0:
 		yaw = math.acos(angle)
 	else:
 		yaw = -1*math.acos(angle)

		
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
		
		while not obtain_control():
			rospy.sleep(0.25)
		if obtain_control():	
			talker()
	except rospy.ROSInterruptException:
		pass
