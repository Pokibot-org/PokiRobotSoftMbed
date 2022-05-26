/*
 * Copyright (c) 2022, CATIE, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Maintainer : ACH
 * Creation : 01/2022
 */


#include "PID.h"

namespace sixtron {

    PID::PID(float Kp, float Ki, float Kd, float dt_seconds, float Kf, PID_format format, bool anti_windup) :
			_format(format),
			_dt(dt_seconds),
			_kp(Kp),
			_ki(Ki),
			_kd(Kd),
			_kf(Kf),
			_target_clamped(0.0f),
			_target_delta_limit(1000.0f),
			_err(0.0f),
			_err_previous(0.0f),
			_i_count(0.0f),
			_d_input_previous(0.0f),
			_output_previous(0.0f),
			_anti_windup(anti_windup),
			_output_clamp(false),
			_i_count_previous(0.0f) {

        // Tf must be superior or equal to _dt, it's not a filter otherwise.
        if (_kf < 1.0f)
            _kf = 1.0f;

        // Set all limits to 0 (unlimited power)
        setLimit(PID_limit::input_limit_HL, 0.0f);
        setLimit(PID_limit::output_limit_HL, 0.0f);
    }

    PID::PID(PID_params pid_parameters, float dt_seconds) : PID(pid_parameters.Kp,
                                              pid_parameters.Ki,
                                              pid_parameters.Kd,
                                              dt_seconds,
                                              pid_parameters.Kf,
                                              pid_parameters.format,
                                              pid_parameters.anti_windup){}

    PID::PID(PID_params pid_parameters) : PID(pid_parameters.Kp,
                                              pid_parameters.Ki,
                                              pid_parameters.Kd,
                                              pid_parameters.dt_seconds,
                                              pid_parameters.Kf,
                                              pid_parameters.format,
                                              pid_parameters.anti_windup){}

    // Must be called periodically
    void PID::compute(PID_args *args) {

		// RAMP
		if ((args->target - _target_clamped) > _target_delta_limit){
			_target_clamped += _target_delta_limit;

		} else if ((args->target - _target_clamped) < -_target_delta_limit){
			_target_clamped += - _target_delta_limit;

		} else {
			_target_clamped = args->target;

		}

        // cap input
        if ((_limit_in_high != 0.0f) && ((_target_clamped + args->feedForward) > _limit_in_high))
			_target_clamped = _limit_in_high;
        if ((_limit_in_low != 0.0f) && ((_target_clamped + args->feedForward) < _limit_in_low))
			_target_clamped = _limit_in_low;

        // Calcul the error depending on the target
        _err = (_target_clamped + args->feedForward) - args->actual;

        // Compute using the right PID
        if (_format == PID_Parallel) { // default
            computeParallel();
        } else if (_format == PID_Serie) {
            computeSerie();
        } else if (_format == PID_Mixte) {
            computeMixte();
        }

        // PID formula
        _output = _p + _i + _d;

        // Apply output filter
        _output = _output_previous + (_dt / (_dt * _kf)) * (_output - _output_previous);

        // Save new error for next round
        _err_previous = _err;

        // Update Output and clamp if necessary
        if ((_limit_out_high != 0.0f) && (_output > _limit_out_high)) {
            _output = _limit_out_high;
            _output_clamp = true;
        } else if ((_limit_out_low != 0.0f) && (_output < _limit_out_low)) {
            _output = _limit_out_low;
            _output_clamp = true;
        } else
            _output_clamp = false;

        // Integral Anti windup if output is clamp
        if (_anti_windup && _output_clamp)
            _i_count = _i_count_previous; // clamp the integral to stop increasing
        else
            _i_count_previous = _i_count;

        // Save output
        args->output = _output;
        _output_previous = _output;
    }

    void PID::computeParallel() {

        // Proportional term
        _p = _kp * _err;

        // Integral term
        _i_count += _err * _dt; // update _i_count for next round
        _i = _ki * _i_count;

        // Derivative term
        _d = _kd * ((_err - _err_previous) / _dt);

    }

    void PID::computeSerie() {

        // Proportional term
        _p = _kp * _err;

        // Integral term
        _i_count += _p * _dt;
        _i = _ki * _i_count;

        // Derivative term
        _d_input = _p + _i;
        _d = _kd * ((_d_input - _d_input_previous) / _dt);
        _d_input_previous = _d_input; // save for next round

    }

    void PID::computeMixte() {

        // Proportional term
        _p = _kp * _err;

        // Integral term
        _i_count += _p * _dt;
        _i = _ki * _i_count;

        // Derivative term
        _d_input = _p;
        _d = _kd * ((_d_input - _d_input_previous) / _dt);
        _d_input_previous = _d_input; // save for next round

    }

    void PID::setLimit(PID_limit select, float limit) {
        switch (select) {
            case input_limit_HL:
                _limit_in_high = limit;
                _limit_in_low = -limit;
                break;
            case input_limit_H:
                _limit_in_high = limit;
                break;
            case input_limit_L:
                _limit_in_low = limit;
                break;
            case output_limit_HL:
                _limit_out_high = limit;
                _limit_out_low = -limit;
                break;
            case output_limit_H:
                _limit_out_high = limit;
                break;
            case output_limit_L:
                _limit_out_low = limit;
                break;
        }
    }

    float PID::getValue(PID_value value) {
        switch (value) {
            case error:
                return _err;
            case integral_count:
                return _i_count;
            case term_p:
                return _p;
            case term_i:
                return _i;
            case term_d:
                return _d;
            case output:
                return _output;
            default:
                return 0.0f; // supress warning
        }
    }



} // namespace sixtron

