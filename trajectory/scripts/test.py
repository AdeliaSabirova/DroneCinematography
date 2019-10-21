import math

drone = open("/home/robotic/catkin_ws/src/dji_adelia/Data.txt","r")
camera = open("/home/robotic/catkin_ws/src/dji_adelia/Camera.txt","r")
drone_coordinates = []
camera_coordinates = []
drone_lines = drone.readlines()
camera_lines = camera.readlines()
pitch = []
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
    pitch.append(angle)