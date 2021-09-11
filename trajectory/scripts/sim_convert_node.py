# -*- coding: utf-8 -*-


#libraries
PKG = 'trajectory'
import roslib; roslib.load_manifest(PKG)
import rospy
import std_msgs
import pyproj
import math


# ROS messages.
from sensor_msgs.msg import Joy
from mav_msgs.msg import Actuators
from trajectory_msgs.msg import MultiDOFJointTrajectory
from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import NavSatFix


#Node for convertation controller messages to siulation messages
class SIMConvertNode():
    def __init__(self):
        # Create subscribers and publishers.
        #Latitude, longitude, altitude of start position
        start_lat = 55.753285
        start_long = 48.745248
        start_alt = 4
        self.pub_sim = rospy.Publisher("/sim/actuators", Joy, queue_size = 1)
        self.pub_odom = rospy.Publisher("/sim/attitude", QuaternionStamped, queue_size = 1)
        self.pub_pose = rospy.Publisher("/sim/gps_position", NavSatFix, queue_size = 1)
        #coordinates transformation ellipses        
        ecef = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
        lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
        self.t=pyproj.Transformer.from_proj(ecef, lla)
        #Latitude, longitude, and altitude trasformation to xyz coordinates
        self.start_x_from_lat, self.start_y_from_lon, self.start_z_from_alt = pyproj.transform(lla, ecef, start_long, start_lat, start_alt, radians=False)
        
           
     #actuators mototr speed   
    def actuator_callback(self, data):
        #print(data.angular_velocities)
        sim_msg = Joy()
        sim_msg.header = std_msgs.msg.Header()
        sim_msg.header.stamp = rospy.Time.now()
        # FR
        sim_msg.axes.append(round(data.angular_velocities[0], 1))
        #RL
        sim_msg.axes.append(round(data.angular_velocities[2], 1))
        #FL
        sim_msg.axes.append(round(data.angular_velocities[3], 1))
        #RR
        sim_msg.axes.append(round(data.angular_velocities[1], 1))
        self.pub_sim.publish(sim_msg)
    
    #position and odometry computation for publishing 
    def position_callback(self, trajectory):
        n = len(trajectory.points)
        q = QuaternionStamped()
        q.header = std_msgs.msg.Header()
        q.header.stamp = rospy.Time.now()
        new_position = NavSatFix()
        for i in range(n):
            lon, lat, alt = self.convert_from_ecef_to_lla(trajectory.points[i].transforms[0].translation.x, trajectory.points[i].transforms[0].translation.y, trajectory.points[i].transforms[0].translation.z)         
            new_position.latitude = lat
            new_position.longitude = lon
            new_position.altitude = alt
            q.quaternion.w = 0
            q.quaternion.x = 0
            q.quaternion.z = 180+math.degrees(trajectory.points[i].transforms[0].rotation.z)
            q.quaternion.y = 0
            #publishers for computed messgaes
            self.pub_pose.publish(new_position)
            self.pub_odom.publish(q)
    
    #function for convertation from xyz coordinate fram to latitude-longitude-altitude coordinated frame    
    def convert_from_ecef_to_lla(self,x,y,z):
        lon, lat, alt = self.t.transform(self.start_x_from_lat + x, self.start_y_from_lon + y, self.start_z_from_alt + z, radians=False)
        return lon, lat, alt
        
    #subscriber for messages of trajectory and motor speed    
    def listener(self):
        rospy.Subscriber("/firefly/command/trajectory", MultiDOFJointTrajectory, self.position_callback)
        rospy.Subscriber("/firefly/command/motor_speed", Actuators, self.actuator_callback)        
        rospy.spin()
        

# Main function.
if __name__ == '__main__':
    # Initialization of the node and name.
    rospy.init_node('sim_convert_node')
    try:
        sim_convert_node = SIMConvertNode()
        sim_convert_node.listener()
    except rospy.ROSInterruptException: pass
