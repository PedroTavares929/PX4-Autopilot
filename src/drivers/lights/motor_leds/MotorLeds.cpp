/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "MotorLeds.hpp"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <px4_arch/io_timer.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <uORB/topics/battery_status.h>

ModuleBase::Descriptor MotorLeds::desc{task_spawn, custom_command, print_usage};

MotorLeds::MotorLeds() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

void MotorLeds::Run()
{
	const hrt_abstime now = hrt_absolute_time();

	if (should_exit()) {
		stop(now);
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s parameter_update;
		_parameter_update_sub.copy(&parameter_update);
		updateParams();
		_publish_all = true;
		_send_frame = true;
	}

	if (!_led_control_pub.advertised()) {
		// A subscription made after a publication only gets the last queued message: advertise first and let
		// the LedController subscribe, so that it receives every arm state published below.
		_led_control_pub.advertise();
		_led_controller.update(_control);
	}

	update_vehicle_state();
	publish_arm_states(now);

	// apply what was just published, the commander's state and any priority 2 override
	_led_controller.update(_control);
	send_frame(now);

	ScheduleDelayed(_led_controller.maximum_update_interval());
}

void MotorLeds::update_vehicle_state()
{
	actuator_armed_s actuator_armed;

	if (_actuator_armed_sub.update(&actuator_armed)) {
		_prearmed = actuator_armed.prearmed;
	}

	failsafe_flags_s failsafe_flags;

	if (_failsafe_flags_sub.update(&failsafe_flags)) {
		_battery_warning = failsafe_flags.battery_warning;
	}

	vehicle_status_s vehicle_status;

	if (_vehicle_status_sub.update(&vehicle_status)) {
		_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
		_failsafe = vehicle_status.failsafe;
		_preflight_ok = vehicle_status.pre_flight_checks_pass;
		_calibrating = vehicle_status.calibration_enabled;
	}
}

void MotorLeds::publish_arm_states(hrt_abstime now)
{
	const unsigned arms = _hw.outputs();
	const int32_t mode = _param_motled_mode.get();
	const int32_t colors[ARMS] {_param_motled_col1.get(), _param_motled_col2.get(), _param_motled_col3.get(), _param_motled_col4.get()};
	const int32_t warn_arms = _param_motled_warn.get();
	const int32_t strobe_arms = _param_motled_strb.get();

	const bool own_colors = (mode == 3) || ((mode == 1) && _armed) || ((mode == 2) && (_armed || _prearmed));

	// the conditions under which the commander's colour is not the normal one
	const bool warning = _failsafe || (_battery_warning >= battery_status_s::WARNING_LOW) || (!_armed && !_preflight_ok);

	ArmState desired[ARMS] {};

	for (unsigned k = 0; k < arms; k++) {
		if (!own_colors || (colors[k] < 0) || (colors[k] > led_control_s::COLOR_WHITE) || (warning && (warn_arms & (1 << k)))) {
			desired[k] = {led_control_s::COLOR_OFF, led_control_s::MODE_DISABLED}; // the PX4 status shows through

		} else if (colors[k] == led_control_s::COLOR_OFF) {
			desired[k] = {led_control_s::COLOR_OFF, led_control_s::MODE_OFF};

		} else {
			const bool strobe = _armed && (strobe_arms & (1 << k));
			desired[k] = {(uint8_t)colors[k], strobe ? led_control_s::MODE_FLASH : led_control_s::MODE_ON};
		}
	}

	// Commander calibration blinks all LEDs at this priority and leaves it disabled afterwards: re-assert the
	// steady states once per second. Not the others: it would restart a flash, or the commander's blink/breathe
	// phase on an arm that shows the PX4 status.
	const bool refresh = !_calibrating && (now - _last_refresh >= 1_s);

	if (refresh) {
		_last_refresh = now;
	}

	unsigned pending = 0;

	for (unsigned k = 0; k < arms; k++) {
		const bool steady = (desired[k].mode == led_control_s::MODE_ON) || (desired[k].mode == led_control_s::MODE_OFF);

		if (_publish_all || (desired[k] != _published[k]) || (refresh && steady)) {
			pending |= 1u << k;
		}
	}

	_publish_all = false;

	// one message per distinct state (the led_control queue has 8 entries, shared with the commander)
	for (unsigned k = 0; k < arms; k++) {
		if (pending & (1u << k)) {
			unsigned arm_mask = 0;

			for (unsigned j = k; j < arms; j++) {
				if ((pending & (1u << j)) && (desired[j] == desired[k])) {
					arm_mask |= 1u << j;
					_published[j] = desired[j];
				}
			}

			pending &= ~arm_mask;
			publish((uint8_t)(arm_mask << 1), desired[k]); // led_control index = motor number
		}
	}
}

void MotorLeds::publish(uint8_t led_mask, const ArmState &state)
{
	led_control_s led_control{};
	led_control.led_mask = led_mask;
	led_control.color = state.color;
	led_control.mode = state.mode;
	led_control.num_blinks = 0; // flash forever
	led_control.priority = PRIORITY;
	led_control.timestamp = hrt_absolute_time();
	_led_control_pub.publish(led_control);
}

void MotorLeds::send_frame(hrt_abstime now)
{
	const float brightness = math::constrain(_param_motled_brt.get(), 0.f, 1.f);
	ws2812::Rgb rgb[ARMS] {};

	for (unsigned k = 0; k < _hw.outputs(); k++) {
		const LedControlDataSingle &led = _control.leds[k + 1];
		const uint8_t b = (uint8_t)(led.brightness * brightness + 0.5f);

		switch (led.color) {
		case led_control_s::COLOR_RED:    rgb[k] = {b, 0, 0}; break;

		case led_control_s::COLOR_GREEN:  rgb[k] = {0, b, 0}; break;

		case led_control_s::COLOR_BLUE:   rgb[k] = {0, 0, b}; break;

		case led_control_s::COLOR_AMBER: // same as yellow, as in the neopixel and rgbled drivers
		case led_control_s::COLOR_YELLOW: rgb[k] = {b, b, 0}; break;

		case led_control_s::COLOR_PURPLE: rgb[k] = {b, 0, b}; break;

		case led_control_s::COLOR_CYAN:   rgb[k] = {0, b, b}; break;

		case led_control_s::COLOR_WHITE:  rgb[k] = {b, b, b}; break;

		default: break; // COLOR_OFF
		}
	}

	// also resent once per second, so that a LED upset by noise on its line recovers
	if (_send_frame || (memcmp(rgb, _sent, sizeof(rgb)) != 0) || (now - _last_frame >= 1_s)) {
		// on -EBUSY or -EAGAIN this is retried in the next run
		if (_hw.send(rgb) == PX4_OK) {
			memcpy(_sent, rgb, sizeof(rgb));
			_last_frame = now;
			_send_frame = false;
		}
	}
}

void MotorLeds::stop(hrt_abstime now)
{
	// The LEDs keep their colour while powered: send one dark frame first, without blocking the work queue
	if (_stop_deadline == 0) {
		_stop_deadline = now + 20_ms;
	}

	if (!_stop_frame_sent) {
		const ws2812::Rgb off[ARMS] {};
		_stop_frame_sent = (_hw.send(off) == PX4_OK);
	}

	if ((_stop_frame_sent && !_hw.busy()) || (now > _stop_deadline)) {
		// hand the arms back to the PX4 status
		publish((uint8_t)(((1u << _hw.outputs()) - 1) << 1), ArmState{led_control_s::COLOR_OFF, led_control_s::MODE_DISABLED});

		ScheduleClear();
		exit_and_cleanup(desc); // deletes this: ~Ws2812TimerBurst() releases the timer, its pins and the DMA stream
		return;
	}

	ScheduleDelayed(1_ms);
}

bool MotorLeds::check_output_config(unsigned timer)
{
	char name[17];
	int32_t value = 0;

	// DShot on this timer would reprogram it and take a second DMA stream for the same request: leave it to dshot
	snprintf(name, sizeof(name), PARAM_PREFIX "_TIM%u", timer);
	param_t handle = param_find(name);

	if ((handle != PARAM_INVALID) && (param_get(handle, &value) == PX4_OK) && (value >= -8) && (value <= -3)) {
		PX4_ERR("%s=%" PRId32 " (DShot): not starting", name, value);
		return false;
	}

	if (timer < MAX_IO_TIMERS) {
		const io_timers_channel_mapping_element_t &mapping = io_timers_channel_mapping.element[timer];

		for (unsigned k = 0; k < mapping.channel_count; k++) {
			const unsigned output = mapping.first_channel_index + k + 1;
			snprintf(name, sizeof(name), PARAM_PREFIX "_FUNC%u", output);
			handle = param_find(name);

			if ((handle != PARAM_INVALID) && (param_get(handle, &value) == PX4_OK) && (value != 0)) {
				PX4_WARN("%s=%" PRId32 " ignored: output %u drives the LED of motor %u", name, value, output, k + 1);
			}
		}
	}

	return true;
}

int MotorLeds::task_spawn(int argc, char *argv[])
{
	unsigned timer = 0;
	unsigned leds = 1;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "t:n:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 't':
			timer = strtoul(myoptarg, nullptr, 10);
			break;

		case 'n':
			leds = strtoul(myoptarg, nullptr, 10);
			break;

		default:
			print_usage("unrecognized option");
			return PX4_ERROR;
		}
	}

	if (!check_output_config(timer)) {
		return PX4_ERROR;
	}

	MotorLeds *instance = new MotorLeds();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;

	const int ret = instance->_hw.init(timer, leds);

	if (ret == PX4_OK) {
		instance->ScheduleNow();
		return PX4_OK;
	}

	PX4_ERR("init failed (%d)", ret);
	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;
	return PX4_ERROR;
}

int MotorLeds::print_status()
{
	_hw.print_status();
	PX4_INFO("armed %d, prearmed %d, failsafe %d, battery warning %u, pre-flight checks pass %d, MOTLED_MODE %" PRId32,
		 _armed, _prearmed, _failsafe, _battery_warning, _preflight_ok, _param_motled_mode.get());

	for (unsigned k = 0; k < _hw.outputs(); k++) {
		PX4_INFO("motor %u, output %u: %s (color %u, mode %u), rgb %u %u %u", k + 1, _hw.first_channel() + k + 1,
			 (_published[k].mode == led_control_s::MODE_DISABLED) ? "PX4 status" : "own color",
			 _published[k].color, _published[k].mode, _sent[k].r, _sent[k].g, _sent[k].b);
	}

	return 0;
}

int MotorLeds::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MotorLeds::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
WS2812B LEDs on the motor arms: one output per arm, all outputs of one timer driven together
by a single DMA burst on the timer update event (STM32H7, TIM2-TIM5).

Output k of the timer is the arm of motor k: led_control index k (index 0 stays the status LED,
the board needs -DBOARD_MAX_LEDS=5). Each arm shows its own colour (MOTLED_COLx) at priority 1,
or the PX4 status when configured so (MOTLED_MODE, MOTLED_WARN). led_control at priority 2
(ROS 2 /fmu/in/led_control, led_control command) overrides it.

The timer is reserved: start this before pwm_out and dshot (rc.additional_init), and leave the
functions of its outputs disabled.

### Examples
$ motor_leds start
$ led_control on -c red -l 1
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("motor_leds", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_INT('t', 0, 0, MAX_IO_TIMERS - 1, "io_timer index", true);
	PRINT_MODULE_USAGE_PARAM_INT('n', 1, 1, Ws2812TimerBurst::MAX_LEDS, "LEDs per arm", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int motor_leds_main(int argc, char *argv[])
{
	return ModuleBase::main(MotorLeds::desc, argc, argv);
}
