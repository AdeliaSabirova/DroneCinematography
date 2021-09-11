# -*- coding: utf-8 -*-
"""
Created on Tue Mar  3 10:54:36 2020

@author: adely
"""

import numpy as np
from matplotlib.pyplot import plot


def initialization():
    trajectory_object = []
    for x in np.arange(2,6,0.1):
        y = x*x*x
        trajectory_object.append([x,y])
    trajectory_object = np.asarray(trajectory_object)

    position_object = []
    error = np.random.normal(0.01, 0.1)
    for x in np.arange(2,6,0.1):
        if x == 2:
            y = x*x*x
            position_object.append([x, y])
        else:
            y = x*x*x+ error
            position_object.append([x+error, y])
    position_object = np.asarray(position_object)
    
    gradient_object = np.gradient(trajectory_object[:,1], trajectory_object[:,0])
    velocity = setting_to_zero(gradient_object)
    
    trajectory_quadrotor = []
    for x in np.arange(-5,5,0.1):
        y = x*x - 5
        trajectory_quadrotor.append([x, y])
    trajectory_quadrotor = np.asarray(trajectory_quadrotor)
    
    gradient_quadrotor = np.gradient(trajectory_quadrotor[:,1], trajectory_quadrotor[:,0])
    velocity_quadrotor = setting_to_zero(gradient_quadrotor)
    
    plot(trajectory_object[:,0], trajectory_object[:,1])
    plot(position_object[:,0], position_object[:,1])
    #plot(trajectory_quadrotor[:,0], trajectory_quadrotor[:,1])
    
    return trajectory_object, position_object, trajectory_quadrotor, velocity, velocity_quadrotor


def minimization(trajectory_object, position):
    distance = np.sqrt(np.power((trajectory_object[:,0]-position[0]),2)+np.power((trajectory_object[:,1]-position[1]),2))
    min_index = np.argmin(distance)
    return trajectory_object[min_index,:], min_index

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

def percentage(gradient, estimated_point_index, integral_of_trajectory, t):
    integral_passed, _ = integral(gradient, estimated_point_index, t, 0)
    return integral_passed/integral_of_trajectory

def integral_estimation(integral, percentage):
    return percentage*integral


def point_estimation(t, trajectory, index_previos, percentage, integral_whole, list_integrals):
    integral_passed = integral_estimation(integral_whole, percentage)
    index = np.argmin(np.absolute(list_integrals[index_previos:index_previos+11]-integral_passed))
    return trajectory[index_previos+index,:], index+index_previos
    
        
def setting_to_zero(trajectory):
    set_trajectory = trajectory-trajectory[0]
    return set_trajectory





trajectory_object, position_object, trajectory_quadrotor, velocity, velocity_quadrotor = initialization()


h = (trajectory_object[len(trajectory_object)-1,0] - trajectory_object[0,0])/len(trajectory_object)
integral_object, _ = integral(velocity, len(velocity)-1, h, 0)

t = (trajectory_quadrotor[len(trajectory_quadrotor)-1,0] - trajectory_quadrotor[0,0])/len(trajectory_quadrotor)
integral_quadrotor, list_integral_quadrotor = integral(velocity_quadrotor, len(velocity_quadrotor)-1, t, 1)

integral_passed_previous, _ = integral(velocity_quadrotor, 0, t, 0)
index_previous = 0
estimated_points = []
estimated_points.append(trajectory_quadrotor[0,:])



for i in range(1, len(position_object)):
    estimated_point, estimated_index = minimization(trajectory_object, position_object[i,:])
    percent = percentage(velocity, estimated_index, integral_object, h)
    point_quadrotor, index = point_estimation(t, trajectory_quadrotor, index_previous, percent, integral_quadrotor, list_integral_quadrotor)
    index_previous = index
    estimated_points.append(point_quadrotor)
    
    
    
    
#estimated_points_in_trajectory = []
#for i in range(len(position_object)):
#    estimated_points_in_trajectory.append(minimization(trajectory_object, position_object[i,:]))
    
#estimated_points_in_trajectory = np.asarray(estimated_points_in_trajectory)


#integral_of_trajectory = integral(trajectory_object, None)


#from scipy import integrate
#integral_of_trajectory_library = integrate.simps(trajectory_object[:,1], trajectory_object[:,0])

        