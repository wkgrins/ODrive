
#include "odrive_main.h"
#include <algorithm>
#include <numeric>

bool Controller::apply_config() {
    config_.parent = this;
    update_filter_gains();
    return true;
}

void Controller::reset() {
    pos_setpoint_ = 0.0f;
    vel_setpoint_ = 0.0f;
    vel_integrator_torque_ = 0.0f;
    torque_setpoint_ = 0.0f;
}

void Controller::set_error(Error error) {
    error_ |= error;
}

//--------------------------------
// Command Handling
//--------------------------------


void Controller::move_to_pos(float goal_point) {
    axis_->trap_traj_.planTrapezoidal(goal_point, pos_setpoint_, vel_setpoint_,
                                 axis_->trap_traj_.config_.vel_limit,
                                 axis_->trap_traj_.config_.accel_limit,
                                 axis_->trap_traj_.config_.decel_limit);
    axis_->trap_traj_.t_ = 0.0f;
    trajectory_done_ = false;
}

void Controller::move_incremental(float displacement, bool from_input_pos = true){
    if(from_input_pos){
        input_pos_ += displacement;
    } else{
        input_pos_ = pos_setpoint_ + displacement;
    }

    input_pos_updated();
}


void Controller::start_anticogging_calibration() {
    if (axis_->error_ == Axis::ERROR_NONE) {
        input_vel_ = config_.anticogging.start_vel;
        anticogging_start_pos = *axis_->encoder_.pos_estimate_.get_current();
        old_vel_integrator_gain = config_.vel_integrator_gain;
        config_.anticogging.integrator_gain = 25.0f * config_.vel_integrator_gain;
        config_.anticogging.bandwidth = config_.anticogging.start_vel / 0.8f;
        // reset
        config_.vel_integrator_gain = 0.0f;
        anticogging_average_error_ = 0.0f;
        anticogging_average_error_old_ = 0.0f;
        anticogging_turn_count_ = 0;
        config_.anticogging.vel_error_filtered = 0.0f;
        config_.anticogging.calib_anticogging = true;
    }
}

void Controller::stop_anticogging_calibration() {
    input_vel_ = 0.0f;
    config_.anticogging.calib_anticogging = false;
    config_.vel_integrator_gain = old_vel_integrator_gain;
}

// find the mean of the anticogging map and subtract it from every bin
void Controller::anticogging_remove_bias() {
    auto& cogmap = config_.anticogging.cogging_map;
    float mean = std::accumulate(cogmap.begin(), cogmap.end(), 0.0f) / cogmap.size();
    for(auto& val : cogmap)
        val -= mean;
}

float Controller::anticogging_get_val(uint32_t index) {
    if (index >= 0 && index < config_.anticogging.cogging_map.size()) {
        return config_.anticogging.cogging_map[index];
    }
    return 0.0f;
}

void Controller::anticogging_set_val(uint32_t index, float val) {
    if (index >= 0 && index < config_.anticogging.cogging_map.size()) {
        config_.anticogging.cogging_map[index] = val;
    }
}

/*
 * This anticogging calibration uses integrator action to populate a cogging map
 * Takes approximately 10 minutes to run
 */
void Controller::anticogging_calibration(float pos_estimate, float vel_estimate, float vel_setpoint) {
    if (config_.anticogging.calib_anticogging && config_.control_mode == CONTROL_MODE_VELOCITY_CONTROL) {
        float vel_error = vel_setpoint - vel_estimate;
        config_.anticogging.vel_error_filtered += 10.0f * current_meas_period * (vel_error - config_.anticogging.vel_error_filtered);
        float pos_single_turn_ = fmodf_pos(pos_estimate, 1.0f); // pos_circular not guaranteed to be [0,1)

        // termination condition is average error being within 0.5% across turns and having done over 10 turns.
        bool done = false;
        if ((((int)(pos_estimate - anticogging_start_pos)) != anticogging_turn_count_) && (anticogging_turn_count_ > 0)) {
            if ((input_vel_ < 1.10f * config_.anticogging.end_vel) && anticogging_turn_count_ > 10) {
                done = true;
            }
            anticogging_average_error_old_ = anticogging_average_error_;
        }
        anticogging_turn_count_ = (int)(pos_estimate - anticogging_start_pos);
        
        // do at least one complete turn before reducing the width, gain, and speed
        bool one_turn = anticogging_turn_count_ > 0;
        anticogging_average_error_ += config_.anticogging.bandwidth * current_meas_period * (std::abs(config_.anticogging.vel_error_filtered)/input_vel_ - anticogging_average_error_);
        float width = (float)config_.anticogging.cogging_map.size()/64.0f; //config_.anticogging.width * config_.anticogging.cogging_map.size();

        if (one_turn) {
            float range = anticog_err_max_ - config_.anticogging.end_tolerance;
            float scale_factor = std::clamp((anticogging_average_error_ - config_.anticogging.end_tolerance) / range, 0.0f, 1.0f); // from 0 to 1

            // scale gain, speed, width, and error filter bandwidth
            // start at 25x vel int gain, end at 5x vel int gain
            config_.anticogging.integrator_gain = scale_factor * (20.0f * old_vel_integrator_gain) + 5.0f * old_vel_integrator_gain;

            // need some space between error changing and there parameters changing. Filter applied to them also.
            float new_vel = scale_factor * (config_.anticogging.start_vel - config_.anticogging.end_vel) + config_.anticogging.end_vel;
            input_vel_ += 1.0f * current_meas_period * (new_vel - input_vel_);
            
            // width is what fraction of the cogging map we do the gaussian broadcast to
            // ideally this should track something like pole_pairs, but it crashes if there are too many calls.
            float end_width = 5.0f / (float)config_.anticogging.cogging_map.size();
            float start_width = 16.0f / (float)config_.anticogging.cogging_map.size();
            float new_width = (float)config_.anticogging.cogging_map.size() * scale_factor * (start_width - end_width) + end_width;
            width += 1.0f * current_meas_period * (new_width - width);

            // The filter should scale with velocity to present a good error measure across different speds
            float end_bandwidth = config_.anticogging.end_vel / 4.0f;
            float start_bandwidth = config_.anticogging.start_vel / 2.0f;
            float new_bandwidth = scale_factor * (start_bandwidth - end_bandwidth) + end_bandwidth;
            config_.anticogging.bandwidth += 1.0f * current_meas_period * (new_bandwidth - config_.anticogging.bandwidth);
        }
        else {
            // find our max error to use for proportionally reducing gains
            anticog_err_max_ = std::max(anticog_err_max_, anticogging_average_error_);
        }

        // used for calculating the right x in call to pdf
        float idxf = pos_single_turn_ * config_.anticogging.cogging_map.size();
        size_t idx = (size_t)idxf;
        float frac = idxf - (float)idx;

        // Calculate cogmap effort and then discretize it
        float cogmap_correction_rate = config_.anticogging.integrator_gain * vel_error;
        float cogmap_correction = cogmap_correction_rate * current_meas_period;

        // apply anticogging with gaussian distribution
        for(int i = 0; i < (int)width; i++) {
            int offset = (i - ((int)width) / 2);
            float x = frac + (float)offset;
            // 1% to 1% for pdf is roughly sigma * 6
            float sigma = width / 6.0f;
            float gaussVal = cogmap_correction * pdf(sigma, x);
            config_.anticogging.cogging_map[(idx + offset) % config_.anticogging.cogging_map.size()] += std::clamp(gaussVal, -config_.anticogging.max_torque, config_.anticogging.max_torque);
        }

        // RMS correction for reporting
        anticogging_correction_pwr_ += 0.001f * (cogmap_correction_rate*cogmap_correction_rate - anticogging_correction_pwr_);

        // exit condition
        if (done) {
            Controller::stop_anticogging_calibration();
            //Controller::anticogging_remove_bias();
            config_.anticogging.pre_calibrated = true;
        }
    }
}

void Controller::update_filter_gains() {
    float bandwidth = std::min(config_.input_filter_bandwidth, 0.25f * current_meas_hz);
    input_filter_ki_ = 2.0f * bandwidth;  // basic conversion to discrete time
    input_filter_kp_ = 0.25f * (input_filter_ki_ * input_filter_ki_); // Critically damped
}

static float limitVel(const float vel_limit, const float vel_estimate, const float vel_gain, const float torque) {
    float Tmax = (vel_limit - vel_estimate) * vel_gain;
    float Tmin = (-vel_limit - vel_estimate) * vel_gain;
    return std::clamp(torque, Tmin, Tmax);
}

bool Controller::update() {
    std::optional<float> pos_estimate_linear = pos_estimate_linear_src_.get_current();
    std::optional<float> pos_estimate_circular = pos_estimate_circular_src_.get_current();
    std::optional<float> pos_wrap = pos_wrap_src_.get_current();
    std::optional<float> vel_estimate = vel_estimate_src_.get_current();

    std::optional<float> anticogging_pos_estimate = axis_->encoder_.pos_estimate_.get_current();
    std::optional<float> anticogging_vel_estimate = axis_->encoder_.vel_estimate_.get_current();

    // TODO also enable circular deltas for 2nd order filter, etc.
    if (config_.circular_setpoints) {
        // Keep pos setpoint from drifting
        input_pos_ = fmodf_pos(input_pos_, config_.circular_setpoint_range);
    }

    // Update inputs
    switch (config_.input_mode) {
        case INPUT_MODE_INACTIVE: {
            // do nothing
        } break;
        case INPUT_MODE_PASSTHROUGH: {
            pos_setpoint_ = input_pos_;
            vel_setpoint_ = input_vel_;
            torque_setpoint_ = input_torque_; 
        } break;
        case INPUT_MODE_VEL_RAMP: {
            float max_step_size = std::abs(current_meas_period * config_.vel_ramp_rate);
            float full_step = input_vel_ - vel_setpoint_;
            float step = std::clamp(full_step, -max_step_size, max_step_size);

            vel_setpoint_ += step;
            torque_setpoint_ = (step / current_meas_period) * config_.inertia;
        } break;
        case INPUT_MODE_TORQUE_RAMP: {
            float max_step_size = std::abs(current_meas_period * config_.torque_ramp_rate);
            float full_step = input_torque_ - torque_setpoint_;
            float step = std::clamp(full_step, -max_step_size, max_step_size);

            torque_setpoint_ += step;
        } break;
        case INPUT_MODE_POS_FILTER: {
            // 2nd order pos tracking filter
            float delta_pos = input_pos_ - pos_setpoint_; // Pos error
            float delta_vel = input_vel_ - vel_setpoint_; // Vel error
            float accel = input_filter_kp_*delta_pos + input_filter_ki_*delta_vel; // Feedback
            torque_setpoint_ = accel * config_.inertia; // Accel
            vel_setpoint_ += current_meas_period * accel; // delta vel
            pos_setpoint_ += current_meas_period * vel_setpoint_; // Delta pos
        } break;
        case INPUT_MODE_MIRROR: {
            if (config_.axis_to_mirror < AXIS_COUNT) {
                std::optional<float> other_pos = axes[config_.axis_to_mirror].encoder_.pos_estimate_.get_current();
                std::optional<float> other_vel = axes[config_.axis_to_mirror].encoder_.vel_estimate_.get_current();

                if (!other_pos.has_value() || !other_vel.has_value()) {
                    set_error(ERROR_INVALID_ESTIMATE);
                    return false;
                }

                pos_setpoint_ = *other_pos * config_.mirror_ratio;
                vel_setpoint_ = *other_vel * config_.mirror_ratio;
            } else {
                set_error(ERROR_INVALID_MIRROR_AXIS);
                return false;
            }
        } break;
        // case INPUT_MODE_MIX_CHANNELS: {
        //     // NOT YET IMPLEMENTED
        // } break;
        case INPUT_MODE_TRAP_TRAJ: {
            if(input_pos_updated_){
                move_to_pos(input_pos_);
                input_pos_updated_ = false;
            }
            // Avoid updating uninitialized trajectory
            if (trajectory_done_)
                break;
            
            if (axis_->trap_traj_.t_ > axis_->trap_traj_.Tf_) {
                // Drop into position control mode when done to avoid problems on loop counter delta overflow
                config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
                pos_setpoint_ = input_pos_;
                vel_setpoint_ = 0.0f;
                torque_setpoint_ = 0.0f;
                trajectory_done_ = true;
            } else {
                TrapezoidalTrajectory::Step_t traj_step = axis_->trap_traj_.eval(axis_->trap_traj_.t_);
                pos_setpoint_ = traj_step.Y;
                vel_setpoint_ = traj_step.Yd;
                torque_setpoint_ = traj_step.Ydd * config_.inertia;
                axis_->trap_traj_.t_ += current_meas_period;
            }
        } break;
        default: {
            set_error(ERROR_INVALID_INPUT_MODE);
            return false;
        }
        
    }

    // Calib_anticogging is only true when calibration is occurring
    if (config_.anticogging.calib_anticogging) {
        if (!anticogging_pos_estimate.has_value() || !anticogging_vel_estimate.has_value()) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        // non-blocking
        anticogging_calibration(*anticogging_pos_estimate, *anticogging_vel_estimate, vel_setpoint_);
    }

    // Position control
    // TODO Decide if we want to use encoder or pll position here
    float gain_scheduling_multiplier = 1.0f;
    float vel_des = vel_setpoint_;
    if (config_.control_mode >= CONTROL_MODE_POSITION_CONTROL) {
        float pos_err;

        if (config_.circular_setpoints) {
            if (!pos_estimate_circular.has_value() || !pos_wrap.has_value()) {
                set_error(ERROR_INVALID_ESTIMATE);
                return false;
            }
            // Keep pos setpoint from drifting
            pos_setpoint_ = fmodf_pos(pos_setpoint_, *pos_wrap);
            // Circular delta
            pos_err = pos_setpoint_ - *pos_estimate_circular;
            pos_err = wrap_pm(pos_err, *pos_wrap);
        } else {
            if (!pos_estimate_linear.has_value()) {
                set_error(ERROR_INVALID_ESTIMATE);
                return false;
            }
            pos_err = pos_setpoint_ - *pos_estimate_linear;
        }

        vel_des += config_.pos_gain * pos_err;
        // V-shaped gain shedule based on position error
        float abs_pos_err = std::abs(pos_err);
        if (config_.enable_gain_scheduling && abs_pos_err <= config_.gain_scheduling_width) {
            gain_scheduling_multiplier = abs_pos_err / config_.gain_scheduling_width;
        }
    }

    // Velocity limiting
    float vel_lim = config_.vel_limit;
    if (config_.enable_vel_limit) {
        vel_des = std::clamp(vel_des, -vel_lim, vel_lim);
    }

    // Check for overspeed fault (done in this module (controller) for cohesion with vel_lim)
    if (config_.enable_overspeed_error) {  // 0.0f to disable
        if (!vel_estimate.has_value()) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        if (std::abs(*vel_estimate) > config_.vel_limit_tolerance * vel_lim) {
            set_error(ERROR_OVERSPEED);
            return false;
        }
    }

    // TODO: Change to controller working in torque units
    // Torque per amp gain scheduling (ACIM)
    float vel_gain = config_.vel_gain;
    float vel_integrator_gain = config_.vel_integrator_gain;
    if (axis_->motor_.config_.motor_type == Motor::MOTOR_TYPE_ACIM) {
        float effective_flux = axis_->async_estimator_.rotor_flux_;
        float minflux = axis_->motor_.config_.acim_gain_min_flux;
        if (std::abs(effective_flux) < minflux)
            effective_flux = std::copysignf(minflux, effective_flux);
        vel_gain /= effective_flux;
        vel_integrator_gain /= effective_flux;
        // TODO: also scale the integral value which is also changing units.
        // (or again just do control in torque units)
    }

    // Velocity control
    float torque = torque_setpoint_;

    // Anti-cogging is enabled during calibration and afterwards
    // has to run live!
    if (config_.anticogging.calib_anticogging || (anticogging_valid_ && config_.anticogging.anticogging_enabled)) {
        if(!pos_estimate_linear.has_value()) {
                set_error(ERROR_INVALID_ESTIMATE);
                return false;
        }
        float pos_ratio = fmodf_pos(*pos_estimate_linear, 1.0f);
        float idxf = pos_ratio * config_.anticogging.cogging_map.size();
        size_t idx = (size_t)idxf;
        size_t idx1 = (idx + 1) % config_.anticogging.cogging_map.size();
        // linear interpolation
        float frac = idxf - (float)idx;
        float cogmap_torque = (1.0f - frac) * config_.anticogging.cogging_map[idx] + frac * config_.anticogging.cogging_map[idx1];
        torque += cogmap_torque;
    }

    float v_err = 0.0f;
    if (config_.control_mode >= CONTROL_MODE_VELOCITY_CONTROL) {
        if (!vel_estimate.has_value()) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }

        v_err = vel_des - *vel_estimate;
        torque += (vel_gain * gain_scheduling_multiplier) * v_err;

        // Velocity integral action before limiting
        torque += vel_integrator_torque_;
    }

    // Velocity limiting in current mode
    if (config_.control_mode < CONTROL_MODE_VELOCITY_CONTROL && config_.enable_current_mode_vel_limit) {
        if (!vel_estimate.has_value()) {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        torque = limitVel(config_.vel_limit, *vel_estimate, vel_gain, torque);
    }

    // Torque limiting
    bool limited = false;
    float Tlim = axis_->motor_.max_available_torque();
    if (torque > Tlim) {
        limited = true;
        torque = Tlim;
    }
    if (torque < -Tlim) {
        limited = true;
        torque = -Tlim;
    }

    // Velocity integrator (behaviour dependent on limiting)
    if (config_.control_mode < CONTROL_MODE_VELOCITY_CONTROL) {
        // reset integral if not in use
        vel_integrator_torque_ = 0.0f;
    } else {
        if (limited) {
            // TODO make decayfactor configurable
            vel_integrator_torque_ *= 0.99f;
        } else {
            vel_integrator_torque_ += ((vel_integrator_gain * gain_scheduling_multiplier) * current_meas_period) * v_err;
        }
    }

    torque_output_ = torque;

    // TODO: this is inconsistent with the other errors which are sticky.
    // However if we make ERROR_INVALID_ESTIMATE sticky then it will be
    // confusing that a normal sequence of motor calibration + encoder
    // calibration would leave the controller in an error state.
    error_ &= ~ERROR_INVALID_ESTIMATE;
    return true;
}
