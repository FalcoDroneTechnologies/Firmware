/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_controls.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include "Controller/Controller.hpp"
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/rc_channels.h>


#include <float.h>


#include <lib/controllib/blocks.hpp>
#include <lib/flight_tasks/FlightTasks.hpp>






extern "C" __EXPORT int fdt_position_motor_main(int argc, char *argv[]);


class fdt_position_motor : public ModuleBase<fdt_position_motor>, public control::SuperBlock, public ModuleParams
{
public:
    fdt_position_motor(int example_param, bool example_flag);
    ~fdt_position_motor() override;

    //virtual ~fdt_position_motor() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
    static fdt_position_motor *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

    bool init();


    /** @see ModuleBase::print_status() */
    void start_up();

    /** @see ModuleBase::print_status() */
   // struct vehicle_local_position_s NED();
    //struct airspeed_s Airspeed(int airspeed_sub, const px4_pollfd_struct_t fds);
    void _updateORBMessages();
    px4_pollfd_struct_t fds[3];

    //Orb Messaeg Structures
    struct vehicle_local_position_s loc_pos;
    struct airspeed_s loc_airspeed;
    struct rc_channels_s _rc_input;





private:

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);


	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
	)

	// Subscriptions
	uORB::Subscription	_parameter_update_sub{ORB_ID(parameter_update)};
    int vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    int airspeed_sub = orb_subscribe(ORB_ID(airspeed));
    int rc_channel_sub = orb_subscribe(ORB_ID(rc_channels));

    //Publications
    uORB::Publication<actuator_controls_s> _actuator_controls_pub2{ORB_ID(actuator_controls_6)};
    uORB::Publication<actuator_controls_s>		_actuator_controls_pub{ORB_ID(actuator_controls_0)};  /**< actuator controls publication */

    Controller _control;
    PositionControlStates _states;
    vehicle_local_position_setpoint_s setpoint;

    matrix::Vector3f position;
    matrix::Vector3f velocity;
    matrix::Vector3f acceleration;

    hrt_abstime	_time_stamp_last_loop{0};		/**< time stamp of last loop iteration */

    actuator_controls_s				_act_controls{};		/**< direct control of actuators */

    actuator_controls_s				_aux_act_controls{};



};

