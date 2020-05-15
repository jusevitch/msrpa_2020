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
from math import atan2

# Trajectory Formatting:
# 
# Every SimpleMSRPA message has the following fields:
#
# string trajectory_type
# float64[] data
# 
# The possible trajectory type strings are as follows:
#   * "linear"
#   * "polynomial"
#   * "circular"
#
# The format of the data depends on the trajectory type, however the first two entries of the data
# always contain t0 and t1 (initial and ending time of trajectory). The descriptions are below.
# 
#   linear:     p0, p1.                     # Each point p0,p1 is a vector of length 3 representing a point in 3D space
#   polynomial: b_0, b_1, ..., b_p          # Each bp is a vectory of length 3 representing a point in 3D space
#   circular:   R, theta0, w, cx, cy, cz    # This will take a bit of explanation; see below
#
#               R:          Scalar radius of circle
#               theta0:     Scalar initial offset angle of point from x-axis
#               w:          Angular velocity of line connecting point to circle center
#               cx, cy, cz: x,y,z coordinates of circle center (each is scalar)
#
# Functions appended with 2D will in general ignore any z-axis or z-position information.


## Bezier Curve Functions
# For computing blazingly fast factorial functions.
# I assume that no one will be using greater than 14th order polynomials for trajectory planning...
nk_lookup = [[1],
             [1,1],
             [1,2,1],
             [1,3,3,1],
             [1,4,6,4,1],
             [1,5,10,10,5,1],
             [1,6,15,20,15,6,1],
             [1,7,21,35,35,21,7,1],
             [1,8,28,56,70,56,28,8,1],
             [1,9,36, 84, 126, 126, 84, 36, 9, 1],
             [1,10, 45, 120, 210, 252, 210, 120, 45, 10, 1],
             [1,11, 55, 165, 330, 462, 462, 330, 165, 55, 11, 1],
             [1,12, 66, 220, 495, 792, 924, 792, 495, 220, 66, 12, 1],
             [1,13, 78, 286, 715, 1287, 1716, 1716, 1287, 715, 286, 78, 13, 1],
             [1,14, 91, 364, 1001, 2002, 3003, 3432, 3003, 2002, 1001, 364, 91, 14, 1]]





def circular2D(t, data):
    # data:
    #   Entries 0, 1 are t0, t1 respectively. This implementation ignores t1 and
    #   has the agents follow the circle indefinitely.
    #   Remaining entries are in this order:
    #       circular    : R, theta0, w, cx, cy, cz (Rovers will ignore cz) 
    #
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

    R = data[2]
    theta0 = data[3]
    w = data[4]
    c = [data[5], data[6]]

    phi = w*t + theta0 + np.pi/2.0
    dphi = w
    
    formation_state = np.array([c[0] + R*np.cos(w*t + theta0), c[1] + R*np.sin(w*t + theta0), phi])
    deriv_formation_state = np.array([-R*w*np.sin(w*t + theta0), R*w*np.cos(w*t + theta0), dphi])
    
    return (formation_state, deriv_formation_state)


def circular3D(t, data):
    # data:
    #   Entries 0, 1 are t0, t1 respectively. This implementation ignores t1 and
    #   has the agents follow the circle indefinitely.
    #   Remaining entries are in this order:
    #       circular    : R, theta0, w, cx, cy, cz (Rovers will ignore cz) 
    #
    # TODO: Currently this just specifies a circle parallel to the floor. Make option to
    #       rotate the circle arbitrarily
    # Parameters:
    #   t : Time variable. Assumes normalized time; i.e. t \in [0,1]
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

    R = data[2]
    theta0 = data[3]
    w = data[4]
    c = data[5:8]

    phi = w*t + theta0 
    dphi = w
    
    formation_state = np.array([c[0] + R*np.cos(w*t + theta0), c[1] + R*np.sin(w*t + theta0), c[2], phi])
    deriv_formation_state = np.array([-R*w*np.sin(w*t + theta0), R*w*np.cos(w*t + theta0), 0, dphi])
    return (formation_state, deriv_formation_state)


def linear2D(t, data):
    # Assumes that t has already been normalized
    
    t0 = data[0]
    t1 = data[1]
    # Ignores z information
    p0 = np.array(data[2:4])
    p1 = np.array(data[5:7])

    print("\np0:\n{} \np1:\n{}".format(p0,p1))
    phi = atan2(p1[1] - p0[1], p1[0] - p0[0])

    formation_state = np.append(t*p1 + (1-t)*p0, phi)
    deriv_formation_state = (1/(t1 - t0))*(p1 - p0)
    dphi = 0
    deriv_formation_state = np.append(deriv_formation_state, dphi)

    return (formation_state, deriv_formation_state)



def linear3D(t, data):
    # Assumes that t has already been normalized
    
    t0 = data[0]
    t1 = data[1]
    p0 = np.array(data[2:5])
    p1 = np.array(data[5:8])

    phi = atan2(p1[1] - p0[1], p1[0] - p0[0])

    formation_state = np.append(t*p1 + (1-t)*p0, phi)
    deriv_formation_state = (1/(t1 - t0))*(p1 - p0)
    dphi = 0
    deriv_formation_state = np.append(deriv_formation_state, dphi)

    return (formation_state, deriv_formation_state)

# FIXME: This needs to be changed to Bezier curves
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

