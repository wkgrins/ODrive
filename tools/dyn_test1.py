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

# Allow motor state to fall to idle
while odrv.axis0.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

odrv.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

#Set motor one to ramped velocity control mode, and set motor 0 to torque control mode.
odrv.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
odrv.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

print("We're cookin' with sauce now my friend!")
# To read a value, simply read the property
#print("Bus voltage is " + str(my_drive.vbus_voltage) + "V")

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








