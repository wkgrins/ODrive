#!/usr/bin/env python3
"""
Configure odrive connected to two motors for operation in Dynamometer
All configuration settings are saved to the odrive firmware upon completion.
Must boot up odrivetool and run odrv0.erase_configuration() to erase existing configurations. 
After erase configuration, power cycle odrive. Then board is ready for new config. 

Author: Willa Grinsfelder willa@grinsfelder.com
Date: 22 March 2021
"""

from __future__ import print_function

import odrive
from odrive.enums import *
import time
import math

# Find a connected ODrive (this will block until you connect)
print("finding an odrive...")
odrv = odrive.find_any()

# Configure Motor Attributes
odrv.axis0.motor.config.current_lim=40		#current limit, amps
odrv.axis1.motor.config.current_lim=40

odrv.axis0.controller.config.vel_limit=17		#velocity limit, turns/second
odrv.axis1.controller.config.vel_limit=17

odrv.axis0.motor.config.pole_pairs=7		#pole pairs in 270KV motors
odrv.axis1.motor.config.pole_pairs=7

odrv.axis0.motor.config.torque_constant=8.27/270	#torque constant for motors	
odrv.axis1.motor.config.torque_constant=8.27/270
	
odrv.axis0.motor.config.motor_type=MOTOR_TYPE_HIGH_CURRENT
odrv.axis1.motor.config.motor_type=MOTOR_TYPE_HIGH_CURRENT

#no brake resistor
#no thermistor

odrv.axis0.encoder.config.cpr=2048*4		#CUI encoders set to max resolution
odrv.axis1.encoder.config.cpr=2048*4

#Startup Procedure
odrv.axis0.config.startup_motor_calibration = True #Set motors to run cailbration on startup
odrv.axis1.config.startup_motor_calibration = True
odrv.axis0.config.startup_encoder_offset_calibration=True #Set encoders to run calibration on startup
odrv.axis1.config.startup_encoder_offset_calibration=True
odrv.axis0.config.startup_closed_loop_control=True #Set axis state to closed loop control on startup
odrv.axis0.config.startup_closed_loop_control=True

# Save updated Odrive configuration to board
odrv.save_configuration()

# Confirm end of program 
print("Configuration complete!")











