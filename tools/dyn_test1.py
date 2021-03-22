#!/usr/bin/env python3
"""
Example usage of the ODrive python library to monitor and control ODrive devices
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

odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

#Check axis state
print(odrv.axis1.current_state)    

#Run motor 1 at 1 turn/s 
odrv.axis1.controller.input_vel=2
sp=odrv.axis1.controller.vel_setpoint

print("We're cookin' with sauce now my friend!")
# To read a value, simply read the property
print("Bus voltage is " + str(odrv.vbus_voltage) + "V")

#Shut down at end of script
odrv.axis0.requested_state = AXIS_STATE_IDLE
odrv.axis1.requested_state = AXIS_STATE_IDLE
# Or to change a value, just assign to the property
#my_drive.axis0.controller.pos_setpoint = 3.14
#print("Position setpoint is " + str(my_drive.axis0.controller.pos_setpoint))

# And this is how function calls are done:
#for i in [1,2,3,4]:
    #print('voltage on GPIO{} is {} Volt'.format(i, my_drive.get_adc_voltage(i)))

# A sine wave to test
#t0 = time.monotonic()
#while True:
#    setpoint = 10000.0 * math.sin((time.monotonic() - t0)*2)
#    print("goto " + str(int(setpoint)))
#    my_drive.axis0.controller.pos_setpoint = setpoint
#    time.sleep(0.01)








