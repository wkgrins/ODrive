# -*- coding: utf-8 -*-
"""
Created on Wed Mar 24 15:40:47 2021
Updated on Fri May 07 11:20:00 2021

@author: Willa Grinsfelder

Purpose: Drive a generator at set speeds and manually measure frequency
Goal: Derrive equation for frequency as a function of shaft speed

Note: No data logging capabilities.
"""

from __future__ import print_function

import odrive
from odrive.enums import *
import time
import math

# Test if odrive is connected
try:
    type(odrv)
except NameError:
    print("We're lost in the sauce! There's no odrive connected!")
    print("finding an odrive...")
    odrv = odrive.find_any()

#Set dynamometer motor to torque control mode
odrv.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

#Set input mode for dynamometer motor to torque ramp up
odrv.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
odrv.axis0.controller.config.vel_ramp_rate = 0.005

#Make sure dynamometer is in closed loop control mode
if odrv.axis0.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
    odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

#Define range of motor one speeds
speeds=[0, 1, 5, 10, 15]

#Initialize list for data logging
omega=[]    #List for motor speed in turns/s
tau=[]      #List for motor torque in Nm
velin=[]
vgen=[]     #List for measured generator voltage
igen=[]     #List for measured generator current
#Iterate motor speeds and collect rpm and torque data during operation
for i in speeds:
    odrv.axis0.controller.input_vel=i       #Loop through velocities
    readystate=input("Are you ready to continue?")
    if readystate=='Y':
        print("Onto the next windspeed we go!")
        time.sleep(0.5)
    else:
        odrv.axis0.requested_state = AXIS_STATE_IDLE
        break
        
#Shut down at end of script
odrv.axis0.requested_state = AXIS_STATE_IDLE

print("Complete!")