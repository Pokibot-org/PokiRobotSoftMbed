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


#ifndef PID_H
#define PID_H

#include <stdint.h>

namespace sixtron {


/*!
 *  \enum PID_format
 *  List of PID format used by compute(). Has to be define when PID is instantiate.
 *  For details, see https://www.numlor.fr/elearning/auto/co/91dCorrecteursPID.html
 */
    typedef enum {
        PID_Parallel,
        PID_Serie,
        PID_Mixte
    } PID_format;

#define PID_DEFAULT     PID_Parallel

/*!
 *  \enum PID_limit
 *  List of limit you can set by using setLimit() function
 */
    typedef enum {
        input_limit_HL,
        input_limit_H,
        input_limit_L,
        output_limit_HL,
        output_limit_H,
        output_limit_L
    } PID_limit;

/*!
 *  \enum PID_value
 *  List of values from PID you can get by using getValue() function
 */
    typedef enum {
        error,
        integral_count,
        term_p,
        term_i,
        term_d,
        output
    } PID_value;

/*!
 *  \struct PID_param
 *  PID parameters structure
 */
    typedef struct PID_params PID_params;
    struct PID_params {
        float Kp = 0.0f;
        float Ki = 0.0f;
        float Kd = 0.0f;
        float Kf = 1.0f;
        float dt_seconds = 0.0f;
        PID_format format = PID_DEFAULT;
        bool anti_windup = true;
    };

/*!
 *  \struct PID_args
 *  PID args structure
 */
    typedef struct PID_args PID_args;
    struct PID_args {
        float target = 0.0f; // The Target input value
        float feedForward = 0.0f;
        float actual = 0.0f; // The actual status of the value (for exemple, the sensor of the motor, or the current speed)
        float output = 0.0f; // The result of the compute() function
    };


/*!
 *  \class PID
 *  PID MBED Library
 */
    class PID {

    public:

        /*!
         *  Default PID constructor.
         *
         *  @param Kp Proportional gain.
         *  @param Ki Integral gain.
         *  @param Kd Derivative gain.
         *  @param dt_seconds sampling time in seconds [s].
         *  @param Kf Filter gain. Default is 1.0f (no filter). Can be increase only.
         *  @param PID_format format used for the PID calculation.
         *
         */
        PID(float Kp, float Ki, float Kd, float dt_seconds, float Kf = 1.0f, PID_format format = PID_DEFAULT,
            bool anti_windup = true);

        PID(PID_params pid_parameters, float dt_seconds);
        PID(PID_params pid_parameters);

        /*!
         *  Set a limit to the PID.
         *
         *  @param select Select the limit to change. See PID_limit Struct for the list of limit.
         *  @param limit Value to set for the limit.
         *
         */
        void setLimit(PID_limit select, float limit);

        /*!
         *  Get a PID value.
         *
         *  @param value Select the Value to get. See PID_value Struct for the list of value.
         *
         */
        float getValue(PID_value value);

        /*!
         *  Compute PID with current arguments.
         *  Must be called periodically
         *
         *  @param args Pointer to the PID arguments structure.
         *
         */
        void compute(PID_args *args);

    private:
        // Current PID format
        PID_format _format;

        void computeParallel();

        void computeSerie();

        void computeMixte();

        // PID limits values
        float _dt; // in seconds
        float _kp;
        float _ki;
        float _kd;
        float _kf;

        // PID parameters values
        float _limit_in_high;
        float _limit_in_low;
        float _limit_out_high;
        float _limit_out_low;

        // PID calculs
		float _target_clamped, _target_delta_limit;
        float _err, _err_previous, _i_count;
        float _p, _i, _d; // Corrector terms
        float _d_input, _d_input_previous; // used for parallel and serie PID
        float _output, _output_previous;

        // PID Integral Anti-Windup
        bool _anti_windup;
        bool _output_clamp;
        float _i_count_previous;

    };

} // namespace sixtron

#endif