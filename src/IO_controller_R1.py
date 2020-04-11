#!/usr/bin/env python

import rospy
import osqp
import numpy as np
from numpy import cos, sin
from scipy.sparse import csc_matrix

from trajectory import circular2D

# Import messages
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates

## Publishers

pub = rospy.Publisher('cmd_vel/input/teleop', Twist, queue_size=1) # Double check to make sure this has correct namespace later


## Main function

def R1_QP_control():
    
    rospy.init_node('R1_QP_control')

    # Subscribers

    rospy.Subscriber("INSERT_SUBSCRIBER_TOPIC_HERE", IOcallback)

    rospy.spin()



## Variables setup

# Agent controller parameters
b = 0.1 # Parameter for Siciliano input / output controller
k = 1

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

# Trajectory setup
t0 = np.inf # Initial time

trajectory_type = "circular" # Types can be None, circular, square, polynomial

traj_func = {
    "circular": circular2D
}

## Obstacle radii Dictionary
## Temporary measure. More permanent solution needs to be created, perhaps with rosparam?
Radii = {
    "rover1": 1,
    "box1": 1.5
}

objectStates = ModelStates()

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


    # Object states
    currentObjectStates = objectStates
    state = a


    # Solve for nominal trajectory tracking controller
    if trajectory_type is not None:
        p, dp = traj_func[trajectory_type](t,c=(0,0), R=10, w=0.1)
        
        uhat = nominalCtrl() #TODO
    else:
        p = state[0:2]
        dp = np.zeros(2)
    
    uhat = nominalCtrl(state, p, dp)
    
    # Minimally modify nominal controller with CBF QP
    q = 2*uhat
    
    A = #TODO Create A matrix
    b = #TODO Create b vector
      
    results = solve(P=csc_matrix(P), A=csc_matrix(A), q=q, u=u)

    # Set u equal to results if the problem was solved; otherwise set it to zero
    # See https://osqp.org/docs/interfaces/status_values.html#status-values for solver_val values
    if(results.info.solver_val == 1 || results.info.solver_val == 2):
        u = results.x
    else:
        u = np.zeros(2)

    vel_msg.linear.x = u[0]
    vel_msg.angular.z = u[1]

    pub.publish(vel_msg)


def nominalCtrl(state, point, point_deriv):
    # Assumes that all inputs are numpy arrays
    y1 = state[0] + b*cos(state[2])
    y2 = state[1] + b*sin(state[2])

    u = np.zeros(2)
    u[0] = point_deriv[0] - k*(point[0] - y1)
    u[1] = point_deriv[1] - k*(point[1] - y2)

    T = np.array([[cos(state[2]), sin(state[2])],[-sin(state[2]/b), cos(state[2]/b)]])
    return T.dot(u)

       


def h(x,xo,R,Ro):
    return (np.linalg.norm(x - xo, 2))**2 - max(R,Ro)

def dhdx(x,xo):
    return 2*(x-xo)

def g(x):
    return np.array([[cos(x[3]), 0], [sin(x[3]), 0], [0,1]])


def objectCallback(msg):
    # Updates the global obstacleStates
    objectStates = msg
    


## Calls main loop

if __name__ == '__main__':
    try:
        R1_QP_control()
    except rospy.ROSInterruptException: pass
