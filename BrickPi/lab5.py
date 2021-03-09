"""
File:           lab5.py
Date:           3/2/2021
Description:    lab5 code for e-puck
Author:         Abha Agrawal (abhaa@andrew.cmu.edu)

"""

# import packages and files for helper functions 
import brickpi3
from odometry import run_odometry
import math
import plsWork as pp

# create the Robot instance.
BP = brickpi3.BrickPi3()

initialx = 0
initialy = 0
destx =  60
desty = 44

waypoints = [] # list of waypoints

       
def pathPlanning():
    waypoints = pp.grabWaypoints(initialx, initialy, destx, desty)
    run_odometry(BP, waypoints, initialx, initialy)
    
pathPlanning()   