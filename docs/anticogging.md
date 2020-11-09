# Anti-cogging

ODrive supports an anti-cogging algorithm that attempts to compensate for the rather high cogging torques seen in hobby motors.

`<odrv>.<axis>.controller.config.anticogging` is a configuration structure that contains the following items:

Name | Type | Use
-- | -- | --
integrator_gain | float | Integrator gain used during anticogging
max_torque | float | Max torque that the anticogging map can apply.
pre_calibrated | bool | If true and using index or absolute encoder, load anticogging map from NVM at startup
calib_anticogging | bool | True when calibration is ongoing
anticogging_enabled | bool | Enable or disable anticogging.  A valid anticogging map can be ignored by setting this to `false`
start_vel | float | Starting velocity for anticogging. Should be high enough for the motor to run and low enough to show visible cogging
end_vel | float | Ending velocity for anticogging. A good value is 1/10 start_vel.
end_tolerance | float | Used in anticogging to determine running speed, gain, and filter bandwidths. This tolerance refers to the mean absolute velocity error.

## Calibration

To calibrate anticogging, first make sure you can adequately control the motor in velocity control.  It should respond to velocity commands. Anticogging uses your setting for `<odrv>.<axis>.controller.config.vel_integrator_gain`, so you must have stable control before running anticogging. It doesn't have to be perfect, however - the stock gains are fairly loose and they work for anticogging with the ODrive D5065 and D6374 motors.

If you have an encoder with an index, set `odrv0.axisX.encoder.config.use_index = true`. Run the calibration as normal. Anticogging will work with a non-index encoder, but the calibration will have to run during *every startup*. Only with an absolute encoder or incremental + index encoder can you save the anticogging map to NVM and use it across reboots.

Start by putting the axis in `AXIS_STATE_CLOSED_LOOP` with `CONTROL_MODE_VELOCITY_CONTROL` and `INPUT_MODE_PASSTHROUGH`.  Make sure you have good control of the motor in this state (it responds to velocity commands). It is normal for the motor to experience some cogging torque in this mode before the anticogging calibration has been applied.

Run `controller.start_anticogging_calibration()`.  The motor will turn quickly at first and slow down as the cogging map improves. You can monitor `<odrv>.<axis>.controller.input_vel` to see how it is progressing. The end condition is for the motor to have completed 10 turns and for `input_vel` to be within 10% of `end_vel`.

When anticogging is complete, the motor will stop. After anticogging calibration is complete, set `<odrv>.<axis>.requested_state = AXIS_STATE_IDLE` and run `<odrv>.<axis>.anticogging_remove_bias()`. After this point, if you are using an encoder with an index, you can run `<odrv>.<axis>.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH` and then go into closed loop control.

If the anticogging performance is not good enough for you, you can run calibration multiple times and the map will improve. Remember to run `start_anticogging_calibration()` and then `anticogging_remove_bias()` after every run to remove the friction bias from the map.

Here are some plots of how the anticogging performs. Both plots are at 0.05 turns/s, with `input_vel` in blue and `vel_estimate` in orange. Note that at low speeds, `vel_estimate` has quite a bit of noise.

Pre-Anticogging velocity control:

![pre-anticogging](screenshots/pre_anticog_vel.png)

Velocity control with anticogging active:

![post-anticogging](screenshots/post_anticog_vel.png)

## Saving to NVM

As of v0.5.1, the anticogging map is saved to NVM after calibrating and calling `odrv0.save_configuration()`

The anticogging map can be reloaded automatically at startup by setting `controller.config.anticogging.pre_calibrated = True` and saving the configuration.  However, this map is only valid and will only be loaded for absolute encoders, or encoders with index pins after the index search.

## Example

``` Py
odrv0.axis0.encoder.config.use_index = True
odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.axis0.motor.config.pre_calibrated = True

odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

odrv0.axis0.controller.start_anticogging_calibration()

# calibration is over when the motor stops.

odrv0.axis0.requested_state = AXIS_STATE_IDLE
odrv0.axis0.controller.remove_anticogging_bias()

odrv0.save_configuration()
odrv0.reboot()
```
