#!/usr/bin/env python

import rospy
import osqp
import numpy as np
from scipy.sparse import csc_matrix

from trajectory import circular2D

# Import messages
from geometry_msgs.msg import Twist

## Publishers

pub = rospy.Publisher('cmd_vel/input/teleop', Twist, queue_size=1) # Double check to make sure this has correct namespace later


## Main function

def R1_QP_control():
    
    rospy.init_node('R1_QP_control')

    # Subscribers

    rospy.Subscriber("INSERT_SUBSCRIBER_TOPIC_HERE", IOcallback)

    rospy.spin()



## Variables setup

# Twist message
vel_msg = Twist()

# Objective function weights
P = csc_matrix(np.identity(3)) 

# Input constraints
vmax = 1;
umax = 1;

# Infinity norm bounds
Au = np.array([[1,0],[-1,0],[0,1],[0,-1]])
bu = np.array([vmax, vmax, umax, umax])

# Initial time
t0 = np.inf

trajectory_type = None

## Helper functions

def IOcallback(msg):
    # Freeze all necessary information
    # Current time
    rosTime = rospy.get_rostime()
    if (t0 < np.inf and rosTime != 0):
        if (t0 >= rosTime):
            t = rosTime - t0
        else:
            t = 0
    else:
        t = 0


    # Current state
    state = #TODO

    # Object states


    # Solve for nominal trajectory tracking controller

    uhat = nominalCtrl() #TODO
    
    # Minimally modify nominal controller with CBF QP
    A = #TODO Create A matrix
    b = #TODO Create b vector
      
    results = solve(P=csc_matrix(P), A=csc_matrix(A), u=u)

    # Set u equal to results if the problem was solved; otherwise set it to zero
    # See https://osqp.org/docs/interfaces/status_values.html#status-values for solver_val values
    if(results.info.solver_val == 1 || results.info.solver_val == 2):
        u = results.x
    else:
        u = np.zeros(2)

    vel_msg.linear.x = u[0]
    vel_msg.angular.z = u[1]

    pub.publish(vel_msg)

def nominalCtrl(msg):
    # Assumes that msg is of type [TODO]
    


def h(x,xo,R,Ro):
    return (np.linalg.norm(x - xo, 2))**2 - max(R,Ro)

def dhdx(x,xo):
    return 2*(x-xo)

def g(x):
    return np.array([[np.cos(x[3]), 0], [np.sin(x[3]), 0], [0,1]])




## Calls main loop

if __name__ == '__main__':
    try:
        R1_QP_control()
    except rospy.ROSInterruptException: pass
