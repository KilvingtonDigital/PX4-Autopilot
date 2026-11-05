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

#include "ActuatorEffectivenessHelicopterDual.hpp"
#include <lib/mathlib/mathlib.h>

using namespace matrix;
using namespace time_literals;

ActuatorEffectivenessHelicopterDual::ActuatorEffectivenessHelicopterDual(ModuleParams *parent)
	: ModuleParams(parent)
{
	char name[32];

	for (int h = 0; h < NUM_HEADS; ++h) {
		for (int i = 0; i < SERVOS_PER_HEAD; ++i) {
			snprintf(name, sizeof(name), "CA_SP%d_ANG%d", h, i);
			_ph.angle[h][i] = param_find(name);

			snprintf(name, sizeof(name), "CA_SP%d_ARM_L%d", h, i);
			_ph.arm[h][i] = param_find(name);

			if (h == 0) {
				snprintf(name, sizeof(name), "CA_SV_CS%d_TRIM", i);

			} else {
				snprintf(name, sizeof(name), "CA_SP%d_TRIM%d", h, i);
			}

			_ph.trim[h][i] = param_find(name);
		}
	}

	for (int i = 0; i < NUM_CURVE_POINTS; ++i) {
		snprintf(name, sizeof(name), "CA_HELI_THR_C%d", i);
		_ph.throttle_curve[i] = param_find(name);
		snprintf(name, sizeof(name), "CA_HELI_PITCH_C%d", i);
		_ph.pitch_curve[i] = param_find(name);
	}

	_ph.roll_k = param_find("CA_HELI2_ROLL_K");
	_ph.yaw_k = param_find("CA_HELI2_YAW_K");
	_ph.spoolup_time = param_find("COM_SPOOLUP_TIME");

	updateParams();
}

void ActuatorEffectivenessHelicopterDual::updateParams()
{
	ModuleParams::updateParams();

	for (int h = 0; h < NUM_HEADS; ++h) {
		for (int i = 0; i < SERVOS_PER_HEAD; ++i) {
			float angle_deg = 0.f;
			param_get(_ph.angle[h][i], &angle_deg);
			_servo[h][i].angle = math::radians(angle_deg);
			param_get(_ph.arm[h][i], &_servo[h][i].arm);
			param_get(_ph.trim[h][i], &_servo[h][i].trim);
		}
	}

	for (int i = 0; i < NUM_CURVE_POINTS; ++i) {
		param_get(_ph.throttle_curve[i], &_throttle_curve[i]);
		param_get(_ph.pitch_curve[i], &_pitch_curve[i]);
	}

	param_get(_ph.roll_k, &_roll_k);
	param_get(_ph.yaw_k, &_yaw_k);
	param_get(_ph.spoolup_time, &_spoolup_time);
}

bool ActuatorEffectivenessHelicopterDual::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// The allocation is non-linear, so everything happens in updateSetpoint(); the matrix stays empty.
	configuration.addActuator(ActuatorType::MOTORS, Vector3f{}, Vector3f{}); // 0: left motor
	configuration.addActuator(ActuatorType::MOTORS, Vector3f{}, Vector3f{}); // 1: right motor

	_first_servo_index = configuration.num_actuators_matrix[0];

	// 3 servos per head, left head first. Trims are added directly in updateSetpoint().
	for (int i = 0; i < NUM_HEADS * SERVOS_PER_HEAD; ++i) {
		configuration.addActuator(ActuatorType::SERVOS, Vector3f{}, Vector3f{});
	}

	return true;
}

void ActuatorEffectivenessHelicopterDual::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const ActuatorVector &actuator_min, const ActuatorVector &actuator_max)
{
	_sat_roll_pos = _sat_roll_neg = false;
	_sat_pitch_pos = _sat_pitch_neg = false;
	_sat_yaw_pos = _sat_yaw_neg = false;

	const float spoolup = throttleSpoolupProgress();
	const float thrust = -control_sp(ControlAxis::THRUST_Z);
	const float throttle = math::interpolateN(thrust, _throttle_curve) * spoolup;
	const float collective = math::interpolateN(thrust, _pitch_curve);

	const float motor = mainMotorEngaged() ? throttle : NAN;
	actuator_sp(0) = motor;
	actuator_sp(1) = motor;

	const float roll = control_sp(ControlAxis::ROLL);
	const float pitch = control_sp(ControlAxis::PITCH);
	const float yaw = control_sp(ControlAxis::YAW);

	// head 0 = left, head 1 = right
	// roll demand  -> differential collective (positive roll = right side down = left up, right down)
	// yaw demand   -> opposite fore-aft cyclic (positive yaw = nose right = left head forward, right head aft)
	const float head_collective[NUM_HEADS] = {collective + _roll_k *roll, collective - _roll_k *roll};
	const float head_pitch[NUM_HEADS] = {pitch - _yaw_k *yaw, pitch + _yaw_k *yaw};

	// How one unit of roll / yaw demand moves a head (used only for the saturation flags)
	const float roll_dir = (_roll_k >= 0.f) ? 1.f : -1.f;
	const float yaw_dir = (_yaw_k >= 0.f) ? 1.f : -1.f;
	const float roll_effect[NUM_HEADS] = {roll_dir, -roll_dir};
	const float yaw_effect[NUM_HEADS] = {-yaw_dir, yaw_dir};

	for (int h = 0; h < NUM_HEADS; ++h) {
		for (int i = 0; i < SERVOS_PER_HEAD; ++i) {
			const Servo &s = _servo[h][i];
			const int idx = _first_servo_index + h * SERVOS_PER_HEAD + i;
			const float pitch_coeff = cosf(s.angle) * s.arm;

			actuator_sp(idx) = head_collective[h] + head_pitch[h] * pitch_coeff + s.trim;

			const bool too_high = actuator_sp(idx) > actuator_max(idx);
			const bool too_low = actuator_sp(idx) < actuator_min(idx);

			if (too_high || too_low) {
				// Direction of each axis demand that pushes this servo further out of range
				const float sign = too_high ? 1.f : -1.f;
				setSaturationFlag(sign * roll_effect[h], _sat_roll_pos, _sat_roll_neg);
				setSaturationFlag(sign * pitch_coeff, _sat_pitch_pos, _sat_pitch_neg);
				setSaturationFlag(sign * yaw_effect[h] * pitch_coeff, _sat_yaw_pos, _sat_yaw_neg);
			}
		}
	}
}

void ActuatorEffectivenessHelicopterDual::setSaturationFlag(float direction, bool &positive_flag, bool &negative_flag)
{
	if (direction > 0.f) {
		positive_flag = true;

	} else if (direction < 0.f) {
		negative_flag = true;
	}
}

bool ActuatorEffectivenessHelicopterDual::mainMotorEngaged()
{
	manual_control_switches_s sw;

	if (_manual_control_switches_sub.update(&sw)) {
		_main_motor_engaged = sw.engage_main_motor_switch == manual_control_switches_s::SWITCH_POS_NONE
				      || sw.engage_main_motor_switch == manual_control_switches_s::SWITCH_POS_ON;
	}

	return _main_motor_engaged;
}

float ActuatorEffectivenessHelicopterDual::throttleSpoolupProgress()
{
	vehicle_status_s vs;

	if (_vehicle_status_sub.update(&vs)) {
		_armed = vs.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
		_armed_time = vs.armed_time;
	}

	const float spoolup_time = (_spoolup_time > 0.01f) ? _spoolup_time : 0.01f;
	const float progress = ((hrt_absolute_time() - _armed_time) / 1e6f) / spoolup_time;

	if (_armed && progress < 1.f) {
		return progress;
	}

	return 1.f;
}

void ActuatorEffectivenessHelicopterDual::getUnallocatedControl(int matrix_index, control_allocator_status_s &status)
{
	// -1 / 0 / +1 only indicate the saturated direction to the rate controller
	status.unallocated_torque[0] = _sat_roll_pos ? 1.f : (_sat_roll_neg ? -1.f : 0.f);
	status.unallocated_torque[1] = _sat_pitch_pos ? 1.f : (_sat_pitch_neg ? -1.f : 0.f);
	status.unallocated_torque[2] = _sat_yaw_pos ? 1.f : (_sat_yaw_neg ? -1.f : 0.f);
	status.unallocated_thrust[2] = 0.f;
}
