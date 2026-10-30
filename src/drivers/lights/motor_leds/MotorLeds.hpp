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

/**
 * @file MotorLeds.hpp
 *
 * WS2812B LEDs on the motor arms, one output (one LED or a short string) per arm.
 *
 * led_control index n (1..4) is the arm of motor n; index 0 stays the status LED.
 * The own colour of each arm is published at priority 1. Where an arm shows the PX4 status,
 * priority 1 is disabled and the commander (priority 0) shows through. Priority 2 (ROS 2,
 * led_control command) overrides both.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/led/led.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/failsafe_flags.h>
#include <uORB/topics/led_control.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>

#include "Ws2812TimerBurst.hpp"

using namespace time_literals;

class MotorLeds : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	MotorLeds();
	~MotorLeds() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:
	static constexpr unsigned ARMS = Ws2812TimerBurst::MAX_OUTPUTS;
	static constexpr uint8_t PRIORITY = 1;

	static_assert(BOARD_MAX_LEDS >= 1 + (int)ARMS,
		      "motor_leds: needs CONFIG_BOARD_COMPILE_DEFINITIONS=\"-DBOARD_MAX_LEDS=5\" (0: status LED, 1-4: arms)");

	struct ArmState {
		uint8_t color;
		uint8_t mode;

		bool operator==(const ArmState &other) const { return (color == other.color) && (mode == other.mode); }
		bool operator!=(const ArmState &other) const { return !(*this == other); }
	};

	void Run() override;

	static bool check_output_config(unsigned timer);

	void update_vehicle_state();
	void publish_arm_states(hrt_abstime now);
	void publish(uint8_t led_mask, const ArmState &state);
	void send_frame(hrt_abstime now);
	void stop(hrt_abstime now);

	Ws2812TimerBurst _hw;

	LedController _led_controller;
	LedControlData _control{}; ///< kept between runs: LedController::update() only fills it on changes

	uORB::Publication<led_control_s> _led_control_pub{ORB_ID(led_control)};

	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _failsafe_flags_sub{ORB_ID(failsafe_flags)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	bool _armed{false};
	bool _prearmed{false};
	bool _failsafe{false};
	bool _preflight_ok{true};
	bool _calibrating{false};
	uint8_t _battery_warning{0};

	ArmState _published[ARMS] {};
	bool _publish_all{true};
	hrt_abstime _last_refresh{0};

	ws2812::Rgb _sent[ARMS] {};
	bool _send_frame{true};
	hrt_abstime _last_frame{0};

	bool _stop_frame_sent{false};
	hrt_abstime _stop_deadline{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MOTLED_MODE>) _param_motled_mode,
		(ParamInt<px4::params::MOTLED_COL1>) _param_motled_col1,
		(ParamInt<px4::params::MOTLED_COL2>) _param_motled_col2,
		(ParamInt<px4::params::MOTLED_COL3>) _param_motled_col3,
		(ParamInt<px4::params::MOTLED_COL4>) _param_motled_col4,
		(ParamInt<px4::params::MOTLED_WARN>) _param_motled_warn,
		(ParamInt<px4::params::MOTLED_STRB>) _param_motled_strb,
		(ParamFloat<px4::params::MOTLED_BRT>) _param_motled_brt
	)
};
