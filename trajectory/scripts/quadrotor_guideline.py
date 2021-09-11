# -*- coding: utf-8 -*-

#libraries
import rospy, threading
import time
import math
import tf
import numpy as np

#Ros messages
from geometry_msgs.msg import Transform, Twist
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from mav_planning_msgs.msg import PolynomialTrajectory




class Guidance:
    #convertation recieved message to required format for controller        
    def callback(self, data):
        #initialization of MultiDOFJointTrajectory message
        trajectory = MultiDOFJointTrajectory()
        trajectory.header.stamp = rospy.Time.now()
        trajectory.header.frame_id = 'frame'
        trajectory.joint_names.append('base_link')
        transforms = Transform()
        velocities = Twist()
        accelerations = Twist()
        point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Time())
        trajectory.points.append(point)
        #initialization of recieved message
        polynomialSegments = data.segments
        time_previous = 0
        coordinates = []
        #transformation data form PolynomialTrajectory format to MultiDOFJointTrajectory
        for msg_segment in polynomialSegments:
            D = 3
            if len(msg_segment.yaw) > 0:
                D=4
            if len(msg_segment.rx) > 0 and len(msg_segment.ry) > 0 and len(msg_segment.rz) > 0:
                D = 6
            time_step = round(msg_segment.segment_time.to_sec())
            for t in np.arange(0, int(time_step), 0.5):
                new_time = np.zeros(len(msg_segment.x))
                for i in range(len(msg_segment.x)):
                    new_time[i] = math.pow(t,i)
                x = np.asarray(msg_segment.x)                    
                x = np.multiply(x,new_time)
                y = np.asarray(msg_segment.y)                    
                y = np.multiply(y,new_time)
                z = np.asarray(msg_segment.z)                    
                z = np.multiply(z,new_time)
                transforms = Transform()
                velocities = Twist()
                accelerations = Twist()
                transforms.translation.x = np.sum(x)
                transforms.translation.y = np.sum(y)
                transforms.translation.z = np.sum(z)
                if D > 3:
                    if D == 4:
                        yaw = np.asarray(msg_segment.yaw)                    
                        yaw = np.multiply(yaw,new_time)
                        yaw = np.sum(yaw)
                        quat = tf.transformations.quaternion_from_euler(0.0,0.0,yaw)
                        transforms.rotation.x = quat[0]
                        transforms.rotation.y = quat[1]
                        transforms.rotation.z = quat[2]
                        transforms.rotation.w = quat[3]
                    if D == 6:
                        rx = np.asarray(msg_segment.rx)   
                        rx = np.multiply(rx,new_time)   
                        ry = np.asarray(msg_segment.ry) 
                        ry = np.multiply(ry,new_time)                    
                        rz = np.asarray(msg_segment.rz)  
                        rz = np.multiply(rz,new_time) 
                        quat = tf.transformations.quaternion_from_euler(np.sum(rx),np.sum(ry),np.sum(rz))
                        transforms.rotation.x = quat[0]
                        transforms.rotation.y = quat[1]
                        transforms.rotation.z = quat[2]
                        transforms.rotation.w = quat[3]
                time_from_start = rospy.rostime.Duration(t + time_previous) 
                point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],time_from_start)                
                trajectory.points.append(point)
                coordinates.append([transforms.translation.x, transforms.translation.y])
            time_previous += int(time_step)
        coordinates = np.asarray(coordinates)
        self.shift = (trajectory.points[len(trajectory.points)-1].transforms[0].translation.x - trajectory.points[0].transforms[0].translation.x)/len(trajectory.points)
        self.trajectory = trajectory
        self.gradient = self.setting_to_zero(np.nan_to_num(np.gradient(coordinates[:,1], coordinates[:,0])))
        

    #initialization of programmed position of dynamic object
    def initialization_programming(self):
        trajectory_object = []
        #choose the trajectory
#==============================================================================
#        #trajectory 1 
#        for x in np.arange(2,8,0.1):
#             y = x*x*x
#             trajectory_object.append([x,y,0])
#==============================================================================
#==============================================================================
#       #trajectory 2
#        for x in np.arange(0.1, 5, 0.1):
#             y = 1/x
#             trajectory_object.append([x,y,0])
#==============================================================================
        #trajectory 3        
        for x in np.arange(0, 7, 0.1):
            y = math.sqrt(x)
            trajectory_object.append([x,y,0])
        trajectory_object = np.asarray(trajectory_object)
        gradient_object = np.gradient(trajectory_object[:,1], trajectory_object[:,0])
        velocity = self.setting_to_zero(gradient_object)
        return trajectory_object, velocity

    #minimazation between trajectory points and position point by comparing distances
    def minimization(self, trajectory_object, position, index, length):
        if length-index-1<16:
            distance = np.sqrt(np.power((trajectory_object[index:length,0]-position[0]),2)+np.power((trajectory_object[index:length,1]-position[1]),2))
        else:
            distance = np.sqrt(np.power((trajectory_object[index:index+16,0]-position[0]),2)+np.power((trajectory_object[index:index+16,1]-position[1]),2))
        min_index = np.argmin(distance)
        return trajectory_object[min_index+index,:], min_index+index

    #integration function
    def integral(self, estimated_point_index, gradient = None, t = None):
        if t == None:
            gradient = self.gradient
            t = self.shift
        integral = gradient[0]
        list_integral = []
        list_integral.append(integral)
        if estimated_point_index == 0:
            pass
        else:
            for i in range(1, estimated_point_index):
                integral += 2*gradient[i]
                list_integral.append(integral - gradient[i])
            integral += gradient[estimated_point_index]
            list_integral.append(integral)
        return integral*t*0.5, np.asarray(list_integral)*t*0.5

    #percentage estimation function 
    def percentage(self, gradient, estimated_point_index, integral_of_trajectory, t):
        integral_passed, _ = self.integral(estimated_point_index, gradient, t)
        return integral_passed/integral_of_trajectory

    #integral estimation from percentge and integral of whole trajectory
    def integral_estimation(self, integral, percentage):
        return percentage*integral

    #point estimation on trajectory function from integrals by comparing the minimum of differences
    def point_estimation(self, index_previos, percentage, integral_whole, list_integrals, length):
        integral_passed = self.integral_estimation(integral_whole, percentage)
        if length-index_previos < 16:
            index = np.argmin(np.absolute(list_integrals[index_previos:length]-integral_passed))
        else:
            index = np.argmin(np.absolute(list_integrals[index_previos:index_previos+16]-integral_passed))
        return index+index_previos
    
    #setting vector to zero coordinate frame    
    def setting_to_zero(self, vector):
        set_vector = vector-vector[0]
        return set_vector

    #function for errored position
    def read_position(self):
        position_object = []
        error = np.random.normal(0.01, 0.1)
        # choose the error trajectory for each
#==============================================================================
#        #trajectory 1 
#        for x in np.arange(2,8,0.1):
#             if x == 2:
#                 y = x*x*x
#                 position_object.append([x, y, 0])
#             else:
#                 y = x*x*x+ error
#                 position_object.append([x+error, y, 0])
#==============================================================================
#==============================================================================
#        #trajectory 2 
#        for x in np.arange(0.1, 5, 0.1):
#             if x == 0.1:
#                 y = 1/x
#                 position_object.append([x,y,0])
#             else:
#                 y = 1/x+error
#                 position_object.append([x+error,y,0])
#==============================================================================
        #trajectory 3        
        for x in np.arange(0, 7, 0.1):
            if x == 0:
                y = math.sqrt(x)
                position_object.append([x,y,0])
            else:
                y = math.sqrt(x)+error
                position_object.append([x+error,y,0])
        position_object = np.asarray(position_object)
        return position_object
    
    #creation of nmessage from computed indexes on trajectory for publishing    
    def trajectory_publishing(self, index, index_previous, increase):   
        #message initialization for publishing
        trajectory = MultiDOFJointTrajectory()
        trajectory.header.stamp = rospy.Time.now()
        trajectory.header.frame_id = 'frame'
        trajectory.joint_names.append('base_link')
        for i in range(index_previous, index + 1):
            point = self.trajectory.points[i]
            if increase == True:  
                point.time_from_start = rospy.Time((i-index_previous)/4)
                point.accelerations[0].linear.x = 1
            else:
                point.time_from_start = rospy.Time((i-index_previous)/2)
                point.velocities[0].linear.x = 0.1
            trajectory.points.append(point)
        return trajectory
        
        
    def __init__(self):
        # Initialization of the node and name.
        rospy.init_node('quadrotor_guideline', anonymous=True)
        #publisher
        self.pub =  rospy.Publisher('/firefly/command/trajectory', MultiDOFJointTrajectory, queue_size = 10)      
        self.rate = rospy.Rate(15)
        #quadrotor trajectory        
        self.trajectory = MultiDOFJointTrajectory()
        self.t = threading.Timer(1.0, self.talker)
        #gradient of the trajectory (first derivative)        
        self.gradient = []
        #the interval between values        
        self.shift = 0        
     
    #Publisher for message of computed trajectory for quadrotor 
    def talker(self):
        #initialization trajectory and gradient coordinates of dynamic object
        object_coordinates, set_object_velocity = self.initialization_programming()    
        position_object = self.read_position()
        #initial integrals of object and quadrotor
        h = (object_coordinates[len(object_coordinates)-1,0] - object_coordinates[0,0])/len(object_coordinates)    
        integral_object, _ = self.integral(len(set_object_velocity)-1, set_object_velocity,  h)
        integral_quadrotor, list_integral_quadrotor = self.integral(len(self.gradient)-1)
        integral_passed_previous, _ = self.integral(0)
        #start indexes
        index_previous = 0
        i = 0
        passed_time = []
        estimated_index_previous = 0

        while not rospy.is_shutdown():
            if i == len(object_coordinates):
                break
            else:
                start_time = time.time()
                #core functions computation
                estimated_point, estimated_index = self.minimization(object_coordinates, position_object[i,:], i, len(object_coordinates))
                percent = self.percentage(set_object_velocity, estimated_index, integral_object, h)
                index = self.point_estimation(index_previous, percent, integral_quadrotor, list_integral_quadrotor, len(list_integral_quadrotor))  
                if index == index_previous:
                    if estimated_index_previous == estimated_index:
                        print("Trajectory didnot publish")
                    else:
                        if estimated_index - estimated_index_previous > 5:
                            trajectory = self.trajectory_publishing(index+1, index_previous, False)
                            self.pub.publish(trajectory)
                            print("Trajectory published from index previous", index_previous, "to new index", index+1)
                            index_previous = index + 3
                else:
                    if index - index_previous > 5:
                        trajectory = self.trajectory_publishing(index, index_previous, True)
                    else: 
                        trajectory = self.trajectory_publishing(index, index_previous, False)
                    print("Trajectory published from index previous", index_previous, "to new index", index)
                    self.pub.publish(trajectory)
                    self.rate.sleep()
                    index_previous = index 
                end_time = time.time()
                passed_time.append(end_time-start_time)
                estimated_index_previous = estimated_index
            i+=1
        passed_time = np.asarray(passed_time)
        #printed computation time
        print(np.mean(passed_time), np.max(passed_time), np.min(passed_time))
            
    #subscriber for message of generated trajectory        
    def listener(self):
        data = rospy.wait_for_message("/trajectory", PolynomialTrajectory)
        self.callback(data)


#main function
if __name__ == '__main__':           
    quadrotor = Guidance()
    try:
        quadrotor.listener()
        quadrotor.talker()       
    except rospy.ROSInterruptException:
        pass
