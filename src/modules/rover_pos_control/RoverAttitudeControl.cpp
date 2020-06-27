/****************************************************************************
 *
 *   Copyright (c) 2013-2016 Estimation and Control Library (ECL). All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ECL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ecl_yaw_controller.cpp
 * Implementation of a simple orthogonal coordinated turn yaw PID controller.
 *
 * Authors and acknowledgements in header.
 */

#include "RoverAttitudeControl.hpp"
#include <float.h>
#include <lib/ecl/geo/geo.h>
#include <mathlib/mathlib.h>


float ECL_YawController::control_bodyrate(const struct ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.roll) && PX4_ISFINITE(ctl_data.pitch) && PX4_ISFINITE(ctl_data.body_y_rate) &&
	      PX4_ISFINITE(ctl_data.body_z_rate) && PX4_ISFINITE(ctl_data.pitch_rate_setpoint) &&
	      PX4_ISFINITE(ctl_data.airspeed_min) && PX4_ISFINITE(ctl_data.airspeed_max) &&
	      PX4_ISFINITE(ctl_data.scaler))) {

		return math::constrain(_last_output, -1.0f, 1.0f);
	}

	/* get the usual dt estimate */
	uint64_t dt_micros = hrt_elapsed_time(&_last_run);
	_last_run = hrt_absolute_time();
	float dt = (float)dt_micros * 1e-6f;

	/* lock integral for long intervals */
	bool lock_integrator = ctl_data.lock_integrator;

	if (dt_micros > 500000) {
		lock_integrator = true;
	}

	/* input conditioning */
	float airspeed = ctl_data.airspeed;

	if (!PX4_ISFINITE(airspeed)) {
		/* airspeed is NaN, +- INF or not available, pick center of band */
		airspeed = 0.5f * (ctl_data.airspeed_min + ctl_data.airspeed_max);

	} else if (airspeed < ctl_data.airspeed_min) {
		airspeed = ctl_data.airspeed_min;
	}

	/* Close the acceleration loop if _coordinated_method wants this: change body_rate setpoint */
	if (_coordinated_method == COORD_METHOD_CLOSEACC) {
		// XXX lateral acceleration needs to go into integrator with a gain
		//_bodyrate_setpoint -= (ctl_data.acc_body_y / (airspeed * cosf(ctl_data.pitch)));
	}

	/* Calculate body angular rate error */
	_rate_error = _bodyrate_setpoint - ctl_data.body_z_rate; // body angular rate error

	if (!lock_integrator && _k_i > 0.0f && airspeed > 0.5f * ctl_data.airspeed_min) {

		float id = _rate_error * dt;

		/*
		 * anti-windup: do not allow integrator to increase if actuator is at limit
		 */
		if (_last_output < -1.0f) {
			/* only allow motion to center: increase value */
			id = math::max(id, 0.0f);

		} else if (_last_output > 1.0f) {
			/* only allow motion to center: decrease value */
			id = math::min(id, 0.0f);
		}

		/* add and constrain */
		_integrator = math::constrain(_integrator + id * _k_i, -_integrator_max, _integrator_max);
	}

	/* Apply PI rate controller and store non-limited output */
	_last_output = (_bodyrate_setpoint * _k_ff + _rate_error * _k_p + _integrator) * ctl_data.scaler *
		       ctl_data.scaler;  //scaler is proportional to 1/airspeed


	return math::constrain(_last_output, -1.0f, 1.0f);
}
