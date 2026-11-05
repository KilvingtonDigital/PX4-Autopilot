/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "control_allocation/actuator_effectiveness/ActuatorEffectiveness.hpp"

#include <px4_platform_common/module_params.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/manual_control_switches.h>

/**
 * Side-by-side tandem helicopter: 2 motors and 2 independent 3-servo CCPM swash plates.
 *
 * Actuator order: 0 = left motor, 1 = right motor, 2..4 = left swash servos, 5..7 = right swash servos.
 *
 * Mixing (head 0 = left, head 1 = right):
 *   thrust -> equal collective on both heads (throttle and collective curves, same as the stock helicopter)
 *   roll   -> differential collective (CA_HELI2_ROLL_K)
 *   pitch  -> equal fore-aft cyclic on both heads
 *   yaw    -> opposite fore-aft cyclic (CA_HELI2_YAW_K)
 *
 * UNTESTED DRAFT: signs and gains must be verified in simulation and on the bench before flight.
 */
class ActuatorEffectivenessHelicopterDual : public ModuleParams, public ActuatorEffectiveness
{
public:
	static constexpr int NUM_HEADS = 2;
	static constexpr int SERVOS_PER_HEAD = 3;
	static constexpr int NUM_CURVE_POINTS = 5;

	ActuatorEffectivenessHelicopterDual(ModuleParams *parent);
	virtual ~ActuatorEffectivenessHelicopterDual() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	const char *name() const override { return "HelicopterDual"; }

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index, ActuatorVector &actuator_sp,
			    const ActuatorVector &actuator_min, const ActuatorVector &actuator_max) override;

	void getUnallocatedControl(int matrix_index, control_allocator_status_s &status) override;

private:
	void updateParams() override;
	float throttleSpoolupProgress();
	bool mainMotorEngaged();
	static void setSaturationFlag(float direction, bool &positive_flag, bool &negative_flag);

	struct Servo {
		float angle;
		float arm;
		float trim;
	};

	struct ParamHandles {
		param_t angle[NUM_HEADS][SERVOS_PER_HEAD];
		param_t arm[NUM_HEADS][SERVOS_PER_HEAD];
		param_t trim[NUM_HEADS][SERVOS_PER_HEAD];
		param_t throttle_curve[NUM_CURVE_POINTS];
		param_t pitch_curve[NUM_CURVE_POINTS];
		param_t roll_k;
		param_t yaw_k;
		param_t spoolup_time;
	};
	ParamHandles _ph{};

	Servo _servo[NUM_HEADS][SERVOS_PER_HEAD] {};
	float _throttle_curve[NUM_CURVE_POINTS] {};
	float _pitch_curve[NUM_CURVE_POINTS] {};
	float _roll_k{0.f};
	float _yaw_k{0.f};
	float _spoolup_time{1.f};

	bool _sat_roll_pos{false};
	bool _sat_roll_neg{false};
	bool _sat_pitch_pos{false};
	bool _sat_pitch_neg{false};
	bool _sat_yaw_pos{false};
	bool _sat_yaw_neg{false};

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	bool _armed{false};
	uint64_t _armed_time{0};

	uORB::Subscription _manual_control_switches_sub{ORB_ID(manual_control_switches)};
	bool _main_motor_engaged{true};

	int _first_servo_index{2};
};
