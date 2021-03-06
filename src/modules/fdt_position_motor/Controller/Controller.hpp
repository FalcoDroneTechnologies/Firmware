/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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

/**
 * @file Controller.hpp
 *
 * A cascaded position controller for position/velocity control only.
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_constraints.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

struct PositionControlStates {
	matrix::Vector3f position;
	matrix::Vector3f velocity;
	matrix::Vector3f acceleration;
	float yaw;
    float airpseed;
};

/**
 * 	Core Position-Control for MC.
 * 	This class contains P-controller for position and
 * 	PID-controller for velocity.
 * 	Inputs:
 * 		vehicle position/velocity/yaw
 * 		desired set-point position/velocity/thrust/yaw/yaw-speed
 * 		constraints that are stricter than global limits
 * 	Output
 * 		thrust vector and a yaw-setpoint
 *
 * 	If there is a position and a velocity set-point present, then
 * 	the velocity set-point is used as feed-forward. If feed-forward is
 * 	active, then the velocity component of the P-controller output has
 * 	priority over the feed-forward component.
 *
 * 	A setpoint that is NAN is considered as not set.
 * 	If there is a position/velocity- and thrust-setpoint present, then
 *  the thrust-setpoint is ommitted and recomputed from position-velocity-PID-loop.
 */


class Controller
{
public:

        Controller() = default;
        ~Controller() = default;


    void setState(const PositionControlStates &states, const float current_airspeed);
    bool update(const float dt);
    void printSomething();

    /**
     * Set the velocity control gains
     * @param P 3D vector of proportional gains for x,y,z axis
     * @param I 3D vector of integral gains
     * @param D 3D vector of derivative gains
     */
    void setVelocityGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D);

    /**
     * Set the position control gains
     * @param P 3D vector of proportional gains for x,y,z axis
     */
    void setPositionGains(const matrix::Vector3f &P);

    /**
     * Pass the desired setpoints
     * Note: NAN value means no feed forward/leave state uncontrolled if there's no higher order setpoint.
     * @param setpoint a vehicle_local_position_setpoint_s structure
     */
    void setInputSetpoint(const vehicle_local_position_setpoint_s &setpoint);

    /**
         * Set the paramters for the wing angle control.

         */

    void setWingParameters(const float wing_gain, const float max_CL, const float min_CL);

    /**
         * Set the minimum and maximum collective normalized thrust [0,1] that can be output by the controller
         * @param min minimum thrust e.g. 0.1 or 0
         * @param max maximum thrust e.g. 0.9 or 1
         */
    void setThrustLimits(const float min, const float max);

    /**
         * Get the controllers output local position setpoint
         * These setpoints are the ones which were executed on including PID output and feed-forward.
         * The acceleration or thrust setpoints can be used for attitude control.
         * @param local_position_setpoint reference to struct to fill up
         */
    void getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const;

    /**
     * Set the normalized hover thrust
     * @param thrust [0,1] with which the vehicle hovers not acelerating down or up with level orientation
     */
    void setHoverThrust(const float hover_thrust) { _hover_thrust = hover_thrust; }

    /**
     * Update the hover thrust without immediately affecting the output
     * by adjusting the integrator. This prevents propagating the dynamics
     * of the hover thrust signal directly to the output of the controller.
     */
    void updateHoverThrust(const float hover_thrust_new);

    /**
     * Pass the current vehicle state to the controller
     * @param PositionControlStates structure
     */
    
    void _thrustConverter(); ///< Acceleration setpoint processing
    



    // Gains
    matrix::Vector3f _gain_pos_p; ///< Position control proportional gain
    matrix::Vector3f _gain_vel_p; ///< Velocity control proportional gain
    matrix::Vector3f _gain_vel_i; ///< Velocity control integral gain
    matrix::Vector3f _gain_vel_d; ///< Velocity control derivative gain

    float _gain_wing; ///< Wing Control Constant

    // Limits
    float _lim_vel_up{}; ///< Upwards velocity limit with feed forward and position control
    float _lim_vel_down{}; ///< Downwards velocity limit with feed forward and position control
    float _lim_thr_min{}; ///< Minimum collective thrust allowed as output [-1,0] e.g. -0.9
    float _lim_thr_max{}; ///< Maximum collective thrust allowed as output [-1,0] e.g. -0.1

    float _hover_thrust{}; ///< Thrust [0,1] with which the vehicle hovers not accelerating down or up with level orientation

    float _max_CL; ///< Acgtuator output for maximum CL Value
    float _min_CL; ///< Acgtuator output for maximum CL Value


        // States
    matrix::Vector3f _pos; /**< current position */
    matrix::Vector3f _vel; /**< current velocity */
    matrix::Vector3f _vel_dot; /**< velocity derivative (replacement for acceleration estimate) */
    matrix::Vector3f _vel_int; /**< integral term of the velocity controller */
    float _true_airpseed;

    // Setpoints
    matrix::Vector3f _pos_sp; /**< desired position */
    matrix::Vector3f _vel_sp; /**< desired velocity */
    matrix::Vector3f _acc_sp; /**< desired acceleration */
    matrix::Vector3f _thr_sp; /**< desired thrust */

     // Debug
    matrix::Vector3f vel_error;
    matrix::Vector3f acc_sp_velocity;
    matrix::Vector3f vel_sp_position;






private:
	bool _updateSuccessful();

	void _positionControl(); ///< Position proportional control
	void _velocityControl(const float dt); ///< Velocity PID control
	void _accelerationControl(); ///< Acceleration setpoint processing


};
