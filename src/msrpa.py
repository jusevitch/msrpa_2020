#!/usr/bin/env python

# 
#      ____  ___   _____ ______   __          __    
#     / __ \/   | / ___// ____/  / /   ____ _/ /_   
#    / / / / /| | \__ \/ /      / /   / __ `/ __ \  
#   / /_/ / ___ |___/ / /___   / /___/ /_/ / /_/ /  
#  /_____/_/  |_/____/\____/  /_____/\__,_/_.___/   
#                                                   
# 
# author: james usevitch
# 
# Description: This file contains the input/output linearization controller for the
#              Hector Quadrotor model in Gazebo. Obstacle avoidance and input constraint satisfaction
#              is calculated using a control barrier function and QP method.
# 


import rospy
import sys
import numpy as np
import argparse
from copy import deepcopy
from msrpa_2020.msgs import MSRPA, PiecewiseLinear, Polynomial, PiecewisePolynomial, Circular


class MSRPA_node:
    def __init__(self, n_quads=0, n_rovers=0, leader_list=[], malicious_list=[], L=np.array([],dtype=int), input_Hz=10.0, eta=1):
        self.n_quads = n_quads
        self.n_rovers = n_rovers
        self.n = n_quads + n_rovers
        self.leader_list = leader_list
        self.malicious_list = malicious_list
        self.L = L
        self.eta = eta
        
        self.pubtopics = ["quad" + str(nq) for nq in range(n_quads)]
        self.pubtopics = ["rover" + str(nr) for nr in range(n_rovers)]

        self.publist = [rospy.Publisher(topic, MSRPA, queue_size=1) for topic in self.pubtopics]
    
        self.timer = rospy.Timer(rospy.Duration(1.0/input_Hz), MSRPA_callback)


    def MSRPA_callback:
        # Do the main callback function



n_quads = 0
n_rovers = 0
n = n_quads + n_rovers

leader_list = []
malicious_list = []

L = np.zeros(n)

pubtopics = []


def MSRPA_init():
    # Initialize all values


def MSRPA_main():
    # Run the main loop here










if __name__ == "__main__":
    
    
    parser = argparse.ArgumentParser(description='msrpa : Node simulating the MS-RPA algorithm for all agents.')
    parser.add_argument('--n_quads', type=int, help="The number of quads in the network.")
    parser.add_argument('--n_rovers', type=int, help="The number of rovers in the network.")
    parser.add_argument('--eta', type=float, help="The parameter eta from the MS-RPA algorithm")
    parser.add_argument('--quad_leader_list', nargs='+', type=int, help="Comma-separated list of quad leaders")
    parser.add_argument('--quad_malicious_list', nargs='+', type=int, help="Comma-separated list of quad malicious agents")
    parser.add_argument('--rover_leader_list', nargs='+', type=int, help="Comma-separated list of rover leaders")
    parser.add_argument('--rover_leader_list', nargs='+', type=int, help="Comma-separated list of rover malicious agents")
    parser.add_argument('--input_Hz', type=float, help="Hz rate at which to run the MS-RPA algorithm")
    parser.add_argument('--L', nargs='+', type=int, help="Laplacian matrix. Input as 1D array with rows appended together.")

    
    if args.n is not None:
        n = args.n

    if args.eta is not None:
        eta = args.eta

    
    try:
        rospy.init_node('MSRPA_master')
        MSRPA_master = MSRPA(
                 
       )
