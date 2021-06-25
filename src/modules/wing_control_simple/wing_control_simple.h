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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/input_rc.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>


#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_attitude.h>


#include <float.h>








extern "C" __EXPORT int wing_control_simple_main(int argc, char *argv[]);


class wing_control_simple : public ModuleBase<wing_control_simple>,  public ModuleParams
{
public:
    wing_control_simple(int example_param, bool example_flag);
    ~wing_control_simple() override;

    //virtual ~wing_control_simple() = default;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static wing_control_simple *instantiate(int argc, char *argv[]);

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
    bool _updateORBMessages();
    px4_pollfd_struct_t fds[2];

    //Orb Messaeg Structures
    struct vehicle_local_position_s loc_pos;
    struct airspeed_s loc_airspeed;
    struct rc_channels_s _rc_input;
    struct vehicle_attitude_s attitude;

    //RC Channel
    int rc_channel;

    float neutral_position;





private:

    /**
     * Check for parameter changes and update them if needed.
     * @param parameter_update_sub uorb subscription to parameter_update
     * @param force for a parameter update
     */
    void parameters_update(bool force = false);


    DEFINE_PARAMETERS(
        (ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
        (ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig,  /**< another parameter */
        (ParamFloat<px4::params::SWC_SERVO_SCALE>) _param_swc_servo_scale,
        (ParamFloat<px4::params::SWC_SERVO_L>) _param_swc_servo_scale_L,
        (ParamFloat<px4::params::SWC_SERVO_R>) _param_swc_servo_scale_R,
        (ParamFloat<px4::params::SWC_OFFSET_L>) _param_swc_offset_L,
        (ParamFloat<px4::params::SWC_OFFSET_R>) _param_swc_offset_R,
        (ParamFloat<px4::params::SWC_CONT_SCALE>) _param_swc_control_scale

    )
    // Subscriptions
    uORB::Subscription	_parameter_update_sub{ORB_ID(parameter_update)};
    int vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    int airspeed_sub = orb_subscribe(ORB_ID(airspeed));
    int rc_channel_sub = orb_subscribe(ORB_ID(rc_channels));
    int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));

    //Publications
    uORB::Publication<actuator_controls_s> _actuator_controls_pub2{ORB_ID(actuator_controls_6)};

    vehicle_local_position_setpoint_s setpoint;

    matrix::Vector3f position;
    matrix::Vector3f velocity;
    matrix::Vector3f acceleration;

    hrt_abstime	_time_stamp_last_loop{0};		/**< time stamp of last loop iteration */

    actuator_controls_s				_aux_act_controls{};

};

