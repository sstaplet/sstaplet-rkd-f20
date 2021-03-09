"""
File:           odometry.py
Date:           3/2/2021
Description:    odometry code for e-puck
Author:         Abha Agrawal (abhaa@andrew.cmu.edu)

"""
from __future__ import division
import math
import time
import odometry_help as od

TIME_STEP = 0.01      # 10 ms
MAX_SPEED = 100       # TBD

PI = 3.14159265
RAD2DEG = (180.0/PI)

WHEEL_RADIUS = 1.7 / 2  # inches
WHEEL_BASE = 4.5        # inches

def run_odometry(BP, waypoints, start_x, start_y):
    
    #get devices 
    BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B))
    BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C))

    #leftMotor = robot.getDevice('left wheel motor')
    #rightMotor = robot.getDevice('right wheel motor')
    #leftEncoder = robot.getDevice('left wheel sensor')
    #rightEncoder = robot.getDevice('right wheel sensor')
    
    #enable devices
    #leftEncoder.enable(TIME_STEP)
    #rightEncoder.enable(TIME_STEP)
    
    #set up wheels 
    #leftMotor.setPosition(float('inf'))
    #rightMotor.setPosition(float('inf'))
    
    # assuming PORT_C for left
    # assuming PORT_B for right

    BP.set_motor_power(BP.PORT_B, 0)
    BP.set_motor_power(BP.PORT_C, 0)

    # initial values 
    left_encoder_value = 0
    right_encoder_value = 0
    state = [start_x, start_y, PI/2] # in, in, rad
    last_state = [start_x, start_y, PI/2]
    
    last_left = 0
    last_right = 0
    current_time = 0
    last_time = 0
    
    # theta we want to rotate to
    th = od.find2(start_x, start_y, waypoints[0][0], waypoints[0][1])
  
    # list that says whether we have acheived our desired thetas 
    # and variable to keep track of which theta we're on
    found = [0] * (len(waypoints) + 1)
    fNum = 0
    
    print("first th:", th * 180 / PI)
    
    
    #@TODO: Write your dead-reckoning/odometry algorithm. You may only use
    #the position sensors left_encoder and right_encoder for this
    #task (no GPS!). You must stop for >=1 second after reaching each
    #destination and print the (x,y) location your robot thinks it is
    #Do not print anything else during demos. You can call 
    #wb_motor_set_velocity() with values between -MAX_SPEED and MAX_SPEED 
    # inclusive.
    while (True):

        right_encoder_value = BP.get_motor_encoder(BP.PORT_B)*(math.pi/180)  # right
        left_encoder_value = BP.get_motor_encoder(BP.PORT_C)*(math.pi/180) # left
        current_time = time.time()
        t = current_time - last_time
        
        # TIME_STEP in milliseconds
        # w_left, w_right in rad/s
        w_left = (left_encoder_value - last_left) / t
        w_right = (right_encoder_value - last_right) / t
    
        #left_speed, right_speed in m/s
        left_speed = w_left * WHEEL_RADIUS
        right_speed = w_right * WHEEL_RADIUS
    
        robot_speed = (left_speed + right_speed) / 2 # in m/s
        w_robot = (right_speed - left_speed) / WHEEL_BASE  # in rad/s
    
        k00 = robot_speed * math.cos(last_state[2])  #last_state[2] in radians
        k01 = robot_speed * math.sin(last_state[2])  #last_state[2] in radians
    
        k10 = (robot_speed) * math.cos(last_state[2] + ((t * w_robot) / 2))
        k11 = (robot_speed) * math.sin(last_state[2] + ((t * w_robot) / 2))
        #note that k20 = k10, k21 = k11
    
        k30 = robot_speed * math.cos(last_state[2] + (t * w_robot))
        k31 = robot_speed * math.sin(last_state[2] + (t * w_robot))
    
        state[0] = last_state[0] + ((t / 6) * (k00 + 4 * k10 + k30)) # x
        state[1] = last_state[1] + ((t / 6) * (k01 + 4 * k11 + k31)) # y
        state[2] = math.fmod(last_state[2] + (t * w_robot), 2 * PI)
        
        print("(x, y, theta):", state[0], state[1], state[2])
        
        # rotating to th[fNum]
        if (found[fNum] == 0):
            # if we're close to our theta
            if (math.fabs(state[2] - th) < 0.02): 
                print("found theta", state[2])
              
                # we have found our theta and start heading towards our destination 
                found[fNum] = 1
                BP.set_motor_power(BP.PORT_B, 100)
                BP.set_motor_power(BP.PORT_C, 100)
                
            # if we're not close to our theta, continue to rotate
            else:
                if (th > state[2]):
                    # rotate anticlockwise
                    BP.set_motor_power(BP.PORT_B, 50)
                    BP.set_motor_power(BP.PORT_C, -50)
                else:
                    # rotate clockwise
                    BP.set_motor_power(BP.PORT_B, -50)
                    BP.set_motor_power(BP.PORT_C, 50)
    
        # if we have rotated to th[fNum]
        elif (found[fNum] == 1):
            # if we're close to our desired position
            if (od.getDistance(state[0], state[1], 
                               waypoints[fNum][0], waypoints[fNum][1]) < 1e-2):
                #increment fNum, since we need to move onto the next point
                fNum += 1
                # stop our motor
                BP.set_motor_power(BP.PORT_B, 0)
                BP.set_motor_power(BP.PORT_C, 0)
                print(f"State (x, y) = ({state[0]}, {state[1]})\n")
                # break if we've reached the end of our list, 
                # otherwise find the next theta
                if fNum == len(waypoints):
                    BP.set_motor_power(BP.PORT_B, 0)
                    BP.set_motor_power(BP.PORT_C, 0)
                    break
                else:
                    th = od.find2(state[0], state[1], waypoints[fNum][0],
                                  waypoints[fNum][1])
                    print("next th:", th)
            # if we haven't reached our position, keep going
            else:
                BP.set_motor_power(BP.PORT_B, 100)
                BP.set_motor_power(BP.PORT_C, 100)
        
        #update our last state values
        last_left = left_encoder_value
        last_right = right_encoder_value
        last_time = current_time
        last_state[0] = state[0]
        last_state[1] = state[1]
        last_state[2] = state[2]
        time.sleep(TIME_STEP)
    