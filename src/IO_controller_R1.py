#!/usr/bin/env python3

import rospy
import osqp
import numpy as np
from numpy import cos, sin, arccos
from scipy.sparse import csc_matrix
import tf.transformations as trans # For quaternion operations
import argparse

from trajectory import circular2D

# Import messages
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates



## Publishers

pub = rospy.Publisher('cmd_vel/input/teleop', Twist, queue_size=1) # Double check to make sure this has correct namespace later


## Variables setup

rover_namespace = 'rover0' 

state = np.zeros(3)

# Agent controller parameters
b = 0.1 # Parameter for Siciliano input / output controller
k = 1

# Twist message
vel_msg = Twist()

# Objective function weights
P = csc_matrix(np.identity(3)) 

# Hz for how often input callback is called
input_Hz = 10

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
patterns = ['rover', 'quad','obstacle']




## Main function

def R1_QP_control():
    
    rospy.init_node('R1_QP_control')

    # Subscribers

    rospy.Subscriber("/gazebo/model_states", objectCallback)
    rospy.Timer(rospy.Duration(1.0/input_Hz), IOcallback)

    rospy.spin()


## Helper functions

def IOcallback(msg):
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
    state_index = currentObjectStates.name.index(rover_namespace) # Gets the index of agent state info
    state[0] = currentObjectStates.pose[state_index].position.x
    state[1] = currentObjectStates.pose[state_index].position.y
    state[2] = determine_theta(currentObjectStates.pose[state_index].orientation)

    # Find the indices for all rovers, quads, and objects specifically labeled "obstacle"
    # The array 'patterns' is defined above in the global scope
    reduced_indices = [array.index(s) for s in array if any(xs in s for xs in patterns)]

    # Solve for nominal trajectory tracking controller
    if trajectory_type is not None:
        p, dp = traj_func[trajectory_type](t,c=(0,0), R=10, w=0.1)
    else:
        p = state[0:2]
        dp = np.zeros(2)
    
    uhat = nominalCtrl(state, p, dp)
    
    # Minimally modify nominal controller with CBF QP
    q = 2*uhat
    
    # TODO: This currently assumes everything has the same safety radius.
    #       Need to create a parameter server which lists the safety radii of all nodes.
    R = 1.5 

    A = np.zeros((len(reduced_indices),2))
    b = np.zeros(len(reduced_indices))

    for i in reduced_indices:
        obstacle_state = np.array([currentobjectstates[reduced_indices[i]].pose.position.x, currentobjectstates[reduced_indices[i]].pose.position.y])
        A[i,:] = dhdx(state[0:2],obstacle_state).dot(g(state))
        b[i] = -h(state,obstacle_state,R,R)

      
    results = solve(P=csc_matrix(P), A=csc_matrix(A), q=q, u=u, verbose=False)

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
    # x and xo refer to the positional (x,y) state of the agent (x) and obstacle (xo)
    return (np.linalg.norm(x[0:2] - xo, 2))**2 - max(R,Ro)

def dhdx(x,xo):
    # x and xo refer to the positional (x,y) state of the agent (x) and obstacle (xo)
    return 2*(x[0,2]-xo)

def g(x):
    return np.array([[cos(x[3]), 0], [sin(x[3]), 0], [0,1]])


def determine_theta(quaternion, tolerance=0.01):
    # Determines a (very rough) approximation of theta by projecting the forward direction axis of the agent
    # (assumed to be x-axis) onto the ground plane (x-y plane in Gazebo). This operation can be undefined,
    # so be careful and make sure that your agents never have their forward axis pointing perpendicular
    # to the ground plane.

    # Tolerance determines how close the x-axis must be to vertical before returning an error. This
    # will need to be improved in future versions.

    # Quaternion must be either numpy vector or geometry_msgs/Quaternion message
    if type(quaternion) is Quaternion:
        rotation_quaternion = np.array([0,0,0,0])
        rotation_quaternion[0] = quaternion.x
        rotation_quaternion[1] = quaternion.y
        rotation_quaternion[2] = quaternion.z
        rotation_quaternion[3] = quaternion.w
    else:
        rotation_quaternion = quaternion

    quat_vector = np.array([1,0,0,0]) # x-axis in local frame
    # Perform the quaternion rotation qvq^-1
    quat_vector = trans.quaternion_multiply(rotation_quaternion, quat_vector)
    quat_vector = trans.quaternion_multiply(quat_vector, trans.quaternion_conjugate(rotation_quaternion))

    # Project resulting vector onto the x-y plane and normalize
    two_dim_vec = np.array([quat_vector[0], quat_vector[1]])
    if np.linalg.norm(two_dim_vec) < tolerance:
        raise RuntimeError('Cannot determine theta: x-vector is too close to vertical. Check model poses or change the tolerance.')
    
    # Determine theta as angle between two_dim_vector and x-axis of global frame
    # If two_dim_vec is zero, return an error...TODO
    return arccos(two_dim_vector.dot(np.array([1,0]))/np.linalg.norm(two_dim_vector))


def objectCallback(msg):
    # Updates the global obstacleStates
    objectStates = msg
    


## Calls main loop

if __name__ == '__main__':
    # Takes inputs from command line
    myargv = rospy.myargv(argv=sys.argv)
    
    parser = argparse.ArgumentParser(description='IO_controller_R1 : Node containing the QP controller for R1 Rovers.')
    parser.add_argument('--rover_namespace', help="The namespace of the rover; i.e. rover0.")
    parser.parse_args()
    
    if args.rover_namespace is not None:
        rover_namespace = args.rover_namespace
    try:
        R1_QP_control()
    except rospy.ROSInterruptException: pass
