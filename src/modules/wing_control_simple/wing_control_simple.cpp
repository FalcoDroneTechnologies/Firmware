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

#include "wing_control_simple.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <mathlib/mathlib.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/actuator_outputs.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>



using namespace matrix;
using namespace time_literals;


wing_control_simple::wing_control_simple(int example_param, bool example_flag)
    : ModuleParams(nullptr)

{
    PX4_INFO("Say Something");
}


wing_control_simple::~wing_control_simple()
{
    PX4_INFO("Destruct... setting PWM out to 0");
    orb_unsubscribe(vehicle_attitude_sub);

}

int wing_control_simple::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("module",
                      SCHED_DEFAULT,
                      SCHED_PRIORITY_MAX -5,
                      3000,
                      (px4_main_t)&run_trampoline,
                      (char *const *)argv);

    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }

    return 0;
}


wing_control_simple *wing_control_simple::instantiate(int argc, char *argv[])
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

    wing_control_simple *instance = new wing_control_simple(example_param, example_flag);

    if (instance == nullptr) {
        PX4_ERR("alloc failed");
    }

    return instance;
}


int wing_control_simple::print_status()
{
    PX4_INFO("Running...");
    // TODO: print additional runtime information about the state of the module

    return 0;
}

int wing_control_simple::custom_command(int argc, char *argv[])
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



void wing_control_simple::run()
{

    //Motor Angle Ratio
    float PI = 3.14159;
    float GearRatio = 26.0/24.0;
    float PulseAngle = 2.0f/(((270.0f/2.0f)*GearRatio)*(PI/180.0f))*_param_swc_servo_scale.get();
    float rc_input = 0;
    //Get vehicle attitude(in quats)
    fds[0] = { .fd = vehicle_attitude_sub,   .events = POLLIN };
    fds[1] = { .fd = rc_channel_sub,   .events = POLLIN };
    PX4_INFO("Retrieving ORBs");


    parameters_update(true);

    while (!should_exit()) {
        if (_updateORBMessages()) {

            orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &attitude);
            //Get Euler Angles
            Quatf q_check(attitude.q[0], attitude.q[1], attitude.q[2], attitude.q[3]);
            Eulerf euler_angles(q_check);


            //Set Servo Position
            if (_rc_input.channels[7]<-1 || _rc_input.channels[7]>1){
                rc_input = 0;

            }
            else {
                rc_input = _rc_input.channels[7];
            }


            float command = euler_angles(1)*PulseAngle+_param_swc_control_scale.get()*rc_input;
            _aux_act_controls.timestamp = hrt_absolute_time();
            _aux_act_controls.control[2] = command*_param_swc_servo_scale_R.get()+PulseAngle*_param_swc_offset_R.get();
             _aux_act_controls.control[1] = command*_param_swc_servo_scale_L.get()+PulseAngle*_param_swc_offset_L.get();
            _actuator_controls_pub2.publish(_aux_act_controls);
            px4_usleep(10);

            PX4_INFO("Command :\t%8.4f",
                    (double) command);
//           PX4_INFO("RC Throttle 2,3,4:\t%8.4f\t%8.4f\t%8.4f",
//                                 (double)_rc_input.channels[5],
//                               (double)_rc_input.channels[6],
//                               (double)_rc_input.channels[7]);
        }
         parameters_update();
    }

}


void wing_control_simple::parameters_update(bool force)
{
    // check for parameter updates
    if (_parameter_update_sub.updated() || force) {
        // clear update
        parameter_update_s update;
        _parameter_update_sub.copy(&update);

        // update parameters from storage
        ModuleParams::updateParams();

    }
}



int wing_control_simple::print_usage(const char *reason)
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

        PRINT_MODULE_USAGE_NAME("wing_control_simple", "template");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
    PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int wing_control_simple_main(int argc, char *argv[])
{
        return wing_control_simple::main(argc, argv);
}

bool wing_control_simple::_updateORBMessages(){


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
                return false;


    }
    if (fds[0].revents & POLLIN) {

        orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &attitude);
        //orb_copy(ORB_ID(input_rc), input_rc_channel_sub, &input_rc);
        //return true;

    }

    if (fds[1].revents & POLLIN) {

        orb_copy(ORB_ID(rc_channels), rc_channel_sub, &_rc_input);
    }
    parameters_update();

    return true;

}




