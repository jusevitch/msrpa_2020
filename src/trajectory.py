#!/usr/bin/env python3

# 
#      ____  ___   _____ ______   __          __    
#     / __ \/   | / ___// ____/  / /   ____ _/ /_   
#    / / / / /| | \__ \/ /      / /   / __ `/ __ \  
#   / /_/ / ___ |___/ / /___   / /___/ /_/ / /_/ /  
#  /_____/_/  |_/____/\____/  /_____/\__,_/_.___/   
#                                                   
# 
# Author: James Usevitch
# 
# Description: File containing functions which calculate the position of a point along
#              a time-varying trajectory. 
# 


import numpy as np 



def circular2D(t, c=(0,0), R=1, w=0.1, theta0=0):
    # Parameters:
    #   t : Time variable. Assumes t0 = 0.
    #   c : Array-like object with x,y coordinates of circle center. Can be
    #       numpy array or anything you can make a numpy array out of.
    #   R : Radius of circle
    #   theta0 : Initial offset angle of line between point and c and the
    #            x-axis of world frame. For example, theta0 = 0 will start
    #            the point moving from the position [R+c[0], 0]. 
    #   w : Angular velocity. Determines how fast point moves around
    #       edge of circle.
    #
    # Returns both the point and the time derivative of the point (i.e. velocity)
    #
    # output_point:         [x, y, phi], where phi is the direction of the formation
    #                       frame x axis
    #
    # deriv_output_point:   [dx, dy, dphi]

    phi = w*t + theta0 + np.pi/2.0
    dphi = w
    
    formation_state = np.array([c[0] + R*np.cos(w*t + theta0), c[1] + R*np.sin(w*t + theta0), phi])
    deriv_formation_state = np.array([-R*w*np.sin(w*t + theta0), R*w*np.cos(w*t + theta0), dphi])

    
    return (formation_state, deriv_formation_state)


def circular3D(t, c=(0,0,0), R=1, w=0.1, theta0=0):
    # TODO: Currently this just specifies a circle parallel to the floor. Make option to
    #       rotate the circle arbitrarily
    # Parameters:
    #   t : Time variable. Assumes t0 = 0.
    #   c : Array-like object with x,y,z coordinates of circle center. Can be
    #       numpy array or anything you can make a numpy array out of.
    #   R : Radius of circle
    #   theta0 : Initial offset angle of line between point and c and the
    #            x-axis of world frame. For example, theta0 = 0 will start
    #            the point moving from the position [R+c[0], 0]. 
    #   w : Angular velocity. Determines how fast point moves around
    #       edge of circle.

    # Returns both the point and the time derivative of the point (i.e. velocity)
    #
    # output_point:         [x, y, z, phi], where phi is the direction of the formation
    #                       frame x axis
    #
    # deriv_output_point:   [dx, dy, dz, dphi]  

    phi = w*t + theta0 
    dphi = w
    
    formation_state = np.array([c[0] + R*np.cos(w*t + theta0), c[1] + R*np.sin(w*t + theta0), c[2], phi])
    deriv_formation_state = np.array([-R*w*np.sin(w*t + theta0), R*w*np.cos(w*t + theta0), 0, dphi])
    return (formation_state, deriv_formation_state)


def line(t, p0, p1, t0=0, t1=1):
    if type(t) is rospy.rostime.Time:
        t = t.to_sec()

    if type(t0) is rospy.rostime.Time:
        t0 = t0.to_sec()

    if type(t1) is rospy.rostime.Time:
        t1 = t1.to_sec()

    phi = atan2(p1[1] - p0[1], p1[0] - p0[0])

    if t < t0:
        formation_state = np.append(p0, phi)
        deriv_formation_state = np.zeros(p0.shape[0] + 1) # derivative of formation state and phi is zero
    elif t > t1:
        formation_state = np.append(p1, phi)
        deriv_formation_state = np.zeros(p1.shape[0] + 1) # derivative of formation state and phi is zero
    else:
        # Find the interpolating value
        s = (t - t0) / (t1 - t0)
        formation_state = np.append(s*p1 + (1-s)*p0, phi)
        deriv_formation_state = (1/(t1 - t0))*(p1 - p0)

    return (formation_state, deriv_formation_state)


def polynomial(t, coefficients, t0=0, t1=1):
    # Coefficients must be in the form
    # [[x coeff]
    #  [y coeff]
    #  [z coeff]]
    #
    # This function will normalize time variable s to be in the range
    # [0,1]

    if type(t) is rospy.rostime.Time:
        t = t.to_sec()

    if type(t0) is rospy.rostime.Time:
        t0 = t0.to_sec()

    if type(t1) is rospy.rostime.Time:
        t1 = t1.to_sec()

    # TODO : This can be made faster by using np.multiply rather than a matrix
    powers = np.arange(coefficients.shape[1])
    deriv_matrix = np.diag(powers[1:],1)
    
    if t < t0:
        formation_state = coefficients[:,0]
        deriv_formation_state = coefficients[:,1]
        accel_formation_state = coefficients[:,2]

        phi = arctan2(deriv_formation_state[1], deriv_formation_state[0])
        dphi = arctan2(accel_formation_state[1], accel_formation_state[0])

        formation_state = np.append(formation_state, phi)
        deriv_formation_state = np.append(deriv_formation_state, dphi)

        return (formation_state, deriv_formation_state)

    elif t > t1:
        formation_state = np.sum(coefficients, axis=1)
        deriv_formation_state = np.sum(deriv_matrix.dot(coefficients.T), axis=0)
        accel_formation_state = np.sum((deriv_matrix**2).dot(coefficients.T), axis=0)

        phi = arctan2(deriv_formation_state[1], deriv_formation_state[0])
        dphi = arctan2(accel_formation_state[1], accel_formation_state[0])

        formation_state = np.append(formation_state, phi)
        deriv_formation_state = np.append(deriv_formation_state, dphi)

        return (formation_state, deriv_formation_state)
        
    else:
        s = (t - t0)/(t1 - t0)
        svec = s**powers
        formation_state = svec.dot(coefficients.T)
        
        deriv_formation_state = svec.dot(deriv_matrix.dot(coefficients.T))

        accel_formation_state = svec.dot((deriv_matrix**2).dot(coefficients.T))

        phi = arctan2(deriv_formation_state[1], deriv_formation_state[0])
        dphi = arctan2(accel_formation_state[1], accel_formation_state[0])
        
        formation_state = np.append(formation_state, phi)
        deriv_formation_state = np.append(deriv_formation_state, dphi)
        
    
    return (formation_state, deriv_formation_state)
        











if __name__ == '__main__':
    print('This module is not meant to be run as a standalone program. Please import it into another script.')

