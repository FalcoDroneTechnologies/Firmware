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

#include "fdt_position_motor.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/actuator_outputs.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include "Controller/Controller.hpp"
#include "Controller/Controller.cpp"


using namespace matrix;
using namespace time_literals;


fdt_position_motor::fdt_position_motor(int example_param, bool example_flag)
    : SuperBlock(nullptr, "MPC"),
      ModuleParams(nullptr)

{
    PX4_INFO("Say Something");
}


fdt_position_motor::~fdt_position_motor()
{
    PX4_INFO("Destruct... setting PWM out to 0");


}

int fdt_position_motor::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("module",
                      SCHED_DEFAULT,
                      SCHED_PRIORITY_MAX -5,
                      1500,
                      (px4_main_t)&run_trampoline,
                      (char *const *)argv);

    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }

    return 0;
}

fdt_position_motor *fdt_position_motor::instantiate(int argc, char *argv[])
{
    int example_param = 0;
    bool example_flag = false;
    bool error_flag = false;

    int myoptind = 1;
    int ch;
    const char *myoptarg = nullptr;

    // parse CLI arguments
    while ((ch = px4_getopt(argc, argv, "p:f", &myoptind,
&myoptarg)) != EOF) {
        switch (ch) {
        case 'p':
            example_param = (int)strtol(myoptarg, nullptr, 10);
            break;

        case 'f':
            example_flag = true;
            break;

        case '?':
            error_flag = true;
            break;

        default:
            PX4_WARN("unrecognized flag");
            error_flag = true;
            break;
        }
    }

    if (error_flag) {
        return nullptr;
    }

    fdt_position_motor *instance = new fdt_position_motor(example_param, example_flag);

    if (instance == nullptr) {
        PX4_ERR("alloc failed");
    }

    return instance;
}


int fdt_position_motor::print_status()
{
    PX4_INFO("Running...");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int fdt_position_motor::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}



void fdt_position_motor::run()
{
    PX4_INFO("Running....");
    start_up();

    fds[0] = { .fd = vehicle_local_position_sub,   .events = POLLIN };
    fds[1] = { .fd = airspeed_sub,   .events = POLLIN };
    fds[2] = { .fd = rc_channel_sub,   .events = POLLIN };
//    fds = {{ .fd = vehicle_local_position_sub,   .events = POLLIN },
//        { .fd = airspeed_sub,   .events = POLLIN },
//        { .fd = rc_channel_sub,   .events = POLLIN }};

     PX4_INFO("Retrieving NED");
    _updateORBMessages();
    //struct vehicle_local_position_s loc_pos = fdt_position_motor::NED();


    PX4_INFO("Retrieving Airspeed");
   //struct airspeed_s loc_airspeed = fdt_position_motor::Airspeed(airspeed_sub, fds[1]);


    PX4_INFO("NED z & vz:\t%8.4f\t%8.4f",
             (double)loc_pos.z,
             (double)loc_pos.vz);
//    PX4_INFO("Airspeed :\t%8.4f",
//             (double)loc_airspeed.true_airspeed_m_s);


     while (!should_exit()) {
         // set _dt in controllib Block - the time difference since the last loop iteration in seconds
         const hrt_abstime time_stamp_now = hrt_absolute_time();
         setDt((time_stamp_now - _time_stamp_last_loop) / 1e6f);
         _time_stamp_last_loop = time_stamp_now;

        _updateORBMessages();
        //loc_pos = fdt_position_motor::NED();
        //loc_airspeed = fdt_position_motor::Airspeed(airspeed_sub, fds[1]);

         _states.position(0) = loc_pos.x; _states.position(1) = loc_pos.y; _states.position(2) = loc_pos.z;
         _states.velocity(0) = loc_pos.vx; _states.velocity(1) = loc_pos.vy; _states.velocity(2) = loc_pos.vz;
         _states.acceleration(0) = loc_pos.ax; _states.acceleration(1) = loc_pos.ay; _states.acceleration(2) = loc_pos.az;
        _control.setState(_states,loc_airspeed.true_airspeed_m_s);
        _control.setInputSetpoint(setpoint);


         _control.update(_dt);
         vehicle_local_position_setpoint_s local_pos_sp{};
         local_pos_sp.timestamp = time_stamp_now;
         _control.getLocalPositionSetpoint(local_pos_sp);


         _aux_act_controls.timestamp = hrt_absolute_time();
         // set the first output to 50% positive (this would rotate a servo halfway into one of its directions)
         _aux_act_controls.control[1] = -_control._thr_sp(2);
         _aux_act_controls.control[2] = -_control._thr_sp(2);

         _actuator_controls_pub2.publish(_aux_act_controls);

//         PX4_INFO("Velocity Setpoint, Velocity Error, Acc Set Point :\t%8.4f\t%8.4f\t%8.4f",
//                  (double)_control.vel_sp_position(2),
//                  (double)_control.vel_error(2),
//                  (double)_control.acc_sp_velocity(2));

//         PX4_INFO("Position :\t%8.4f",
//                  (double)loc_pos.z);

//         PX4_INFO("Thrust Sestpoint :\t%8.4f",
//                  (double)_control._thr_sp(2));
         //px4_usleep(10000);




     }

    PX4_INFO("Stopping Motor");
     _aux_act_controls.timestamp = hrt_absolute_time();
     // set the first output to 50% positive (this would rotate a servo halfway into one of its directions)
     _aux_act_controls.control[1] = 0.0f;
     _aux_act_controls.control[2] = 0.0f;
     _actuator_controls_pub2.publish(_aux_act_controls);
     PX4_INFO("Unsubscribing");

     orb_unsubscribe(vehicle_local_position_sub);
 }

void fdt_position_motor::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
        ModuleParams::updateParams();
        SuperBlock::updateParams();
	}
}



int fdt_position_motor::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

        PRINT_MODULE_USAGE_NAME("fdt_position_motor", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int fdt_position_motor_main(int argc, char *argv[])
{
        return fdt_position_motor::main(argc, argv);
}

void fdt_position_motor::start_up()
{
    PX4_INFO("Starting Up...");
     _time_stamp_last_loop = hrt_absolute_time();


     //Set Position Demand
     setpoint = FlightTask::empty_setpoint;
     setpoint.x = 0.0; setpoint.y = 0.0; setpoint.z = -0.8;
     _control.setInputSetpoint(setpoint);

    //Set PID Gains
    matrix::Vector3f velGainP(0.0,0.0,2.0);
    matrix::Vector3f velGainI(0.0,0.0,2.0);
    matrix::Vector3f velGainD(0.0,0.0,0.0);
    matrix::Vector3f posGainP(0.0,0.0,1.0);

    //Set Hover Thrust
    _control.setHoverThrust(0.5);

    //Set Wing Parameters
    float gain_wing = 1.0;
    float max_CL = 0.8;
    float min_CL = -0.2;

    PX4_INFO("Setting Gains (Velocity");
    _control.setVelocityGains(velGainP,velGainI,velGainD);
    PX4_INFO("Setting Gains (Position");
    _control.setPositionGains(posGainP);
    PX4_INFO("Setting Thrust Limits");
    _control.setThrustLimits(0.2,0.9);
    PX4_INFO("Setting Wing Parameters");
    _control.setWingParameters(gain_wing,max_CL,min_CL);

     PX4_INFO("Arming Motor...");

    //Arm Motor
    _aux_act_controls.timestamp = hrt_absolute_time();
    _aux_act_controls.control[1] = 0.2f;
     _aux_act_controls.control[2] = 0.2f;
    _actuator_controls_pub2.publish(_aux_act_controls);

    PX4_INFO("Pause 3 Seconds... get ready!");
    sleep(3);






}


//struct vehicle_local_position_s fdt_position_motor::NED()
//{
////    int vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));

//    struct vehicle_local_position_s loc_pos;

//    // initialize parameters
//    parameters_update(true);

//    // wait for up to 1000ms for data
//    int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

//    if (pret == 0) {
//        // Timeout: let the loop run anyway, don't do `continue` here
//        PX4_INFO("Timeout NED");

//    } else if (pret < 0) {
//        // this is undesirable but not much we can do
//        PX4_ERR("poll error %d, %d", pret, errno);
//        px4_usleep(50000);


//    } else if (fds[0].revents & POLLIN) {


//        orb_copy(ORB_ID(vehicle_local_position), vehicle_local_position_sub, &loc_pos);




//    }
//    parameters_update();

//    return loc_pos;


//}

//struct airspeed_s fdt_position_motor::Airspeed(int airspeed_sub, const px4_pollfd_struct_t fd)
//{
////    int vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
//    px4_pollfd_struct_t fds[1];
//    fds[0] = fd;
//    struct airspeed_s loc_airspeed;

//    // initialize parameters
//    parameters_update(true);

//    // wait for up to 1000ms for data
//    int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

//    if (pret == 0) {
//        // Timeout: let the loop run anyway, don't do `continue` here
//        PX4_INFO("Timeout Airspeed");

//    } else if (pret < 0) {
//        // this is undesirable but not much we can do
//        PX4_ERR("poll error %d, %d", pret, errno);
//        px4_usleep(50000);


//    } else if (fds[0].revents & POLLIN) {


//        orb_copy(ORB_ID(airspeed), airspeed_sub, &loc_airspeed);


//    }
//    parameters_update();

//    return loc_airspeed;
//}

void fdt_position_motor::_updateORBMessages(){


    // initialize parameters
    parameters_update(true);

    // wait for up to 1000ms for data
    int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

    if (pret == 0) {
        // Timeout: let the loop run anyway, don't do `continue` here
        PX4_INFO("Timeout NED");

    } else if (pret < 0) {
        // this is undesirable but not much we can do
        PX4_ERR("poll error %d, %d", pret, errno);
        px4_usleep(50000);


    } else if (fds[0].revents & POLLIN) {


        orb_copy(ORB_ID(vehicle_local_position), vehicle_local_position_sub, &loc_pos);
        orb_copy(ORB_ID(airspeed), airspeed_sub, &loc_airspeed);
        orb_copy(ORB_ID(rc_channels), rc_channel_sub, &_rc_input);





    }
    parameters_update();

    ;


}


