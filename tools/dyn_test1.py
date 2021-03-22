#!/usr/bin/env python3
"""
Run dynamometer with one motor in velocity control and the other in torque 
control mode, logging rpm and torque data for motors.
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

#Set motor one to ramped velocity control mode, and set motor 0 to torque control mode.
odrv.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

#Set input mode for motor one to ramped velocity
odrv.axis1.controller.config.input_mode = INPUT_MODE_VEL_RAMP
#Set input mode for motor zero to passthrough
odrv.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH

if odrv.axis0.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
    odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

#Set motor zero torque value
odrv.axis0.controller.input_torque = 0.1

#Define range of motor one speeds
speeds=[1,2,3,4]

#Initialize list for data logging
omega=[]    #List for motor speed in turns/s
tau=[]      #List for motor torque in Nm
key=[]

#Iterate motor speeds and collect rpm and torque data during operation
for i in speeds:
    odrv.axis1.controller.input_vel=i
    j=0         #Itinerant
    while j<30: #Log data
        j=j+1
        key.append(i)
        omega.append(odrv.axis1.encoder.vel_estimate)
        tau.append(8.27*odrv.axis1.motor.current_control.Iq_measured/270)
        time.sleep(0.5)   
        
#Shut down at end of script
odrv.axis0.requested_state = AXIS_STATE_IDLE
odrv.axis1.requested_state = AXIS_STATE_IDLE

#Write output to csv file format

with open('dyntest.txt', 'w') as filehandle:
    filehandle.write("Key, Speed (turns/s), Torque (Nm)\n")
    for i in omega:
        ind=omega.index(i)
        string=str(key[ind]) + ", " + str(i) + ", " + str(tau[ind]) + "\n"
        filehandle.write(string)

