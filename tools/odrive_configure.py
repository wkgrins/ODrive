#!/usr/bin/env python3
"""
Configure odrive connected to two motors for operation in Dynamometer
All configuration settings are saved to the odrive firmware upon completion.
"""

from __future__ import print_function

import odrive
from odrive.enums import *
import time
import math

# Find a connected ODrive (this will block until you connect)
print("finding an odrive...")
odrv = odrive.find_any()

#Erase existing configuration, automatically reboots odrive
odrv.erase_configuration()

# Find the rebooted ODrive
print("finding the rebooted odrive...")
odrv = odrive.find_any()
print("You're reconnected now!")

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
odrv.axis0.config.startup_motor_calibration = True
odrv.axis1.config.startup_motor_calibration = True
odrv.axis0.config.startup_encoder_offset_calibration=True
odrv.axis1.config.startup_encoder_offset_calibration=True
odrv.axis0.config.startup_closed_loop_control=True
odrv.axis0.config.startup_closed_loop_control=True

# Save updated Odrive configuration
odrv.save_configuration()
odrv.reboot()
# Confirm end of program 
print("Configuration complete! Checking axis states at startup")


#when working configuration is finally loaded, backup using .json file
#odrivetool backup-config my_config.json










