#!/usr/bin/env python3

# File containing functions which calculate the position of a point along
# a time-varying trajectory. 

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

    # Returns both the point and the time derivative of the point (i.e. velocity)
    
    point = np.array([c[0] + R*np.cos(w*t + theta0), c[1] + R*np.sin(w*t + theta0)])
    point_deriv = np.array([-R*w*np.sin(w*t + theta0), R*w*np.cos(w*t + theta0)])
    return (point, point_deriv)


# def square:


















if __name__ == '__main__':
    print('This module is not meant to be run as a standalone program. Please import it into another script.')

