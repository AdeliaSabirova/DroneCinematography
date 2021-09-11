# -*- coding: utf-8 -*-
"""
Created on Wed Mar 18 18:39:21 2020

@author: robotic
"""

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
    read_points = np.asanyarray(read_points)
    return read_points

#initialization function of coordinates fromfile
def initialization():
    drone_coordinates = read_from_file("/home/robotic/catkin_ws/src/dji_adelia/DroneCoordinates.txt")
    camera_coordinates = read_from_file("/home/robotic/catkin_ws/src/dji_adelia/CameraCoordinates.txt")
    drone_velocity = read_from_file("/home/robotic/catkin_ws/src/dji_adelia/DroneVelocity.txt")
	#object_coordinates = read_from_file("/home/robotic/catkin_ws/src/dji_adelia/ObjectCoordinates.txt")
	#object_velocity = read_from_file("/home/robotic/catkin_ws/src/dji_adelia/ObjectVelocity.txt")
    gradient_drone = np.gradient(drone_coordinates[:,1], drone_coordinates[:,0])
	#gradient_object = np.gradient(object_coordinates[:,1], object_coordinates[:,0])
    set_drone_velocity = setting_to_zero(gradient_drone)
	#set_object_velocity = setting_to_zero(gradient_object)
    return drone_coordinates, camera_coordinates, drone_velocity, None, None, set_drone_velocity, None

def initialization_programming():
    trajectory_object = []
    for x in np.arange(2,6,0.1):
        y = x*x*x
        trajectory_object.append([x,y,0])
    trajectory_object = np.asarray(trajectory_object)
   
    gradient_object = np.gradient(trajectory_object[:,1], trajectory_object[:,0])
    velocity = setting_to_zero(gradient_object)
    
    trajectory_quadrotor = []
    for x in np.arange(-5,5,0.1):
        y = 0.5*x*x - 5
        trajectory_quadrotor.append([x, y, 20])
    trajectory_quadrotor = np.asarray(trajectory_quadrotor)
    
    trajectory_camera = []
    for x in np.arange(-5,5,0.1):
        y = x*x - 4
        trajectory_camera.append([x, y, 18])
    trajectory_camera = np.asarray(trajectory_camera)
    
    gradient_quadrotor = np.gradient(trajectory_quadrotor[:,1], trajectory_quadrotor[:,0])
    velocity_quadrotor = setting_to_zero(gradient_quadrotor)
    
    return trajectory_object, velocity, trajectory_quadrotor, velocity_quadrotor, trajectory_camera

#minimazation between trajectory points and position point
def minimization(trajectory_object, position):
    distance = np.sqrt(np.power((trajectory_object[:,0]-position[0]),2)+np.power((trajectory_object[:,1]-position[1]),2))
    min_index = np.argmin(distance)
    return trajectory_object[min_index,:], min_index

#integration function
def integral(gradient, estimated_point_index, t, l):
    integral = gradient[0]
    list_integral = []
    if l == 1:
        list_integral.append(integral)
    if estimated_point_index == 0:
            pass
    else:
        for i in range(1, estimated_point_index):
            integral += 2*gradient[i]
            if l == 1:
                list_integral.append(integral - gradient[i])
        integral += gradient[estimated_point_index]
        if l == 1:
            list_integral.append(integral)
    return integral*t*0.5, np.asarray(list_integral)*t*0.5

#percentage estimation function 
def percentage(gradient, estimated_point_index, integral_of_trajectory, t):
    integral_passed, _ = integral(gradient, estimated_point_index, t, 0)
    return integral_passed/integral_of_trajectory

#estimation integral from percentge and integral of whole trajectory
def integral_estimation(integral, percentage):
    return percentage*integral

#point estimation on trajectory function from integrals
def point_estimation(t, trajectory, index_previos, percentage, integral_whole, list_integrals):
    integral_passed = integral_estimation(integral_whole, percentage)
    index = np.argmin(np.absolute(list_integrals[index_previos:index_previos+11]-integral_passed))
    return trajectory[index_previos+index,:], index+index_previos
    
#setting vector to zero coordinate frame    
def setting_to_zero(vector):
    set_vector = vector-vector[0]
    return set_vector

#function for position test
def read_position():
    position_object = []
    error = np.random.normal(0.01, 0.1)
    for x in np.arange(2,6,0.1):
        if x == 2:
            y = x*x*x
            position_object.append([x, y, 0])
        else:
            y = x*x*x+ error
            position_object.append([x+error, y, 0])
    position_object = np.asarray(position_object)
    return position_object
	#return read_from_file("/home/robotic/catkin_ws/src/dji_adelia/Position.txt")

#function on obtaining control panel
def obtain_control():
	print('obtain_control')
	rospy.wait_for_service('/dji_sdk/sdk_control_authority')
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
		
def trajectory_publishing(index, index_previous, drone_coordinates, camera_coordinates, pub, rate):   
    #initialization a message for publishing
    trajectory = MultiDOFJointTrajectory()
    trajectory.header.stamp = rospy.Time.now()
    trajectory.header.frame_id = 'frame'
    trajectory.joint_names.append('base_link')
    transforms = Transform()
    velocities = Twist()
    accelerations = Twist()
    point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Time())
    trajectory.points.append(point)
    t = 0.5
    r = 0.19
    for i in range(index_previous, index+1):
        #position
        transforms = Transform()
        velocities = Twist()
#==============================================================================
#         velocities.linear.x = drone_velocity[i,0]
#         velocities.linear.y = drone_velocity[i,1]
#         velocities.linear.z = drone_velocity[i,2]
#==============================================================================
        accelerations = Twist()
        transforms.translation.x = -1*drone_coordinates[i,0]
        transforms.translation.y = -1*drone_coordinates[i,1]
        transforms.translation.z = drone_coordinates[i,2]

        #orientation
        dx = -1*camera_coordinates[i,0] + drone_coordinates[i,0]
        dy = -1*camera_coordinates[i,1] + drone_coordinates[i,1]
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

        #message forming  
        point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Time((t)))
        t += 0.5
        trajectory.points.append(point)
    
    rospy.sleep(1)
    pub.publish(trajectory)
    rate.sleep()
    print("published")
    
def talker():
    
    pub = rospy.Publisher('/firefly/command/trajectory', MultiDOFJointTrajectory, queue_size = 10)
    rate = rospy.Rate(10)
    #initialization trajectories and velocities coordinates
    #drone_coordinates, camera_coordinates, drone_velocity, object_coordinates, object_velocity, set_drone_velocity, set_object_velocity = initialization()
    object_coordinates, set_object_velocity, drone_coordinates, set_drone_velocity, camera_coordinates = initialization_programming()    
    position_object = read_position()
    
    #initial integrals
    h = (object_coordinates[len(object_coordinates)-1,0] - object_coordinates[0,0])/len(object_coordinates)
    integral_object, _ = integral(set_object_velocity, len(set_object_velocity)-1, h, 0)
    t = (drone_coordinates[len(drone_coordinates)-1,0] - drone_coordinates[0,0])/len(drone_coordinates)
    integral_quadrotor, list_integral_quadrotor = integral(set_drone_velocity, len(set_drone_velocity)-1, t, 1)
    integral_passed_previous, _ = integral(set_drone_velocity, 0, t, 0)
    index_previous = 0
    #trajectory_publishing(index_previous, 1, drone_velocity, drone_coordinates, camera_coordinates)
    trajectory_publishing(index_previous, index_previous, drone_coordinates, camera_coordinates, pub, rate)
    
    
        
    for i in range(1, len(position_object)):
        estimated_point, estimated_index = minimization(object_coordinates, position_object[i,:])
        percent = percentage(set_object_velocity, estimated_index, integral_object, h)
        point_quadrotor, index = point_estimation(t, drone_coordinates, index_previous, percent, integral_quadrotor, list_integral_quadrotor)
        #trajectory_publishing(index, index_previous, drone_velocity, drone_coordinates, camera_coordinates)     
        trajectory_publishing(index, index_previous, drone_coordinates, camera_coordinates, pub, rate)
        index_previous = index
            


if __name__ == '__main__':
	rospy.init_node('talker', anonymous = False)
	try:
#==============================================================================
# 		while not obtain_control():
# 			rospy.sleep(0.25)
# 		if obtain_control():	
#               talker()
#==============================================================================
         talker()
                
	except rospy.ROSInterruptException:
		pass