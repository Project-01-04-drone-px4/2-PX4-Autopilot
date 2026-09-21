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

#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/SubscriptionInterval.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/log_message.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

#include "MspV1.hpp"
#include "MessageDisplay/MessageDisplay.hpp"
#include "uorb_to_msp.hpp"

using namespace time_literals;

#define POWER_LEVEL_COUNT 5
#define BAND_COUNT 7

struct PerformanceData {
	bool initialization_problems{false};
	long unsigned int successful_sends{0};
	long unsigned int unsuccessful_sends{0};
};

// mapping from symbol name to bit in the parameter bitmask
//  @TODO investigate params; it seems like this should be available directly?
enum SymbolIndex : uint8_t {
	CRAFT_NAME		= 0,
	DISARMED		= 1,
	GPS_LAT			= 2,
	GPS_LON			= 3,
	GPS_SATS		= 4,
	GPS_SPEED		= 5,
	HOME_DIST		= 6,
	HOME_DIR		= 7,
	MAIN_BATT_VOLTAGE	= 8,
	CURRENT_DRAW		= 9,
	MAH_DRAWN		= 10,
	RSSI_VALUE		= 11,
	ALTITUDE		= 12,
	NUMERICAL_VARIO		= 13,
	FLYMODE			= 14,
	ESC_TMP			= 15,
	PITCH_ANGLE		= 16,
	ROLL_ANGLE		= 17,
	CROSSHAIRS		= 18,
	AVG_CELL_VOLTAGE	= 19,
	HORIZON_SIDEBARS	= 20,
	POWER			= 21
};

class MspOsd : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	MspOsd(const char *device);

	~MspOsd() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	/** @see ModuleBase::print_status() */
	int print_status() override;

	int set_channel(char *new_channel);

private:
	void Run() override;

	// update a single display element in the display
	void Send(const unsigned int message_type, const void *payload);
	void Send(const unsigned int message_type, const void *payload, int32_t payload_size);

	static constexpr int32_t DISPLAY_MAX_COLUMN = 59;
	static constexpr int32_t DISPLAY_MAX_ROW = 21;

	// DisplayPort uses zero-based character coordinates on a 60 x 22 canvas.
	template<typename T>
	void SendDisplay(T &message, int64_t column, int64_t row)
	{
		message.screenXPosition = math::constrain(column, int64_t{0}, int64_t{DISPLAY_MAX_COLUMN});
		message.screenYPosition = math::constrain(row, int64_t{0}, int64_t{DISPLAY_MAX_ROW});
		Send(MSP_CMD_DISPLAYPORT, &message, sizeof(message));
	}

	// receive vtx data
	void Receive();

	void SendTelemetry();

	// perform actions required for local updates
	void parameters_update();

	// convenience function to check if a given symbol is enabled
	bool enabled(const SymbolIndex &symbol);

	MspV1 _msp{0};
	int _msp_fd{-1};

	msp_osd::MessageDisplay _display{};

	bool _is_initialized{false};

	// subscriptions to desired vehicle display information
	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _home_position_sub{ORB_ID(home_position)};
	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};
	uORB::Subscription _log_message_sub{ORB_ID(log_message)};
	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// local heartbeat
	bool _heartbeat{false};

	// parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::OSD_SYMBOLS>) _param_osd_symbols,
		(ParamInt<px4::params::OSD_CH_POS_VER>) _param_osd_ch_pos_ver,
		(ParamInt<px4::params::OSD_CH_POS_HOR>) _param_osd_ch_pos_hor,
		(ParamInt<px4::params::OSD_SCROLL_RATE>) _param_osd_scroll_rate,
		(ParamInt<px4::params::OSD_DWELL_TIME>) _param_osd_dwell_time,
		(ParamInt<px4::params::OSD_LOG_LEVEL>) _param_osd_log_level,
		(ParamInt<px4::params::OSD_RC_STICK>) _param_osd_rc_stick,
		(ParamInt<px4::params::OSD_MSG_X>) _param_osd_msg_x,
		(ParamInt<px4::params::OSD_MSG_Y>) _param_osd_msg_y,
		(ParamInt<px4::params::OSD_RSSI_X>) _param_osd_rssi_x,
		(ParamInt<px4::params::OSD_RSSI_Y>) _param_osd_rssi_y,
		(ParamInt<px4::params::OSD_CELL_X>) _param_osd_cell_x,
		(ParamInt<px4::params::OSD_CELL_Y>) _param_osd_cell_y,
		(ParamInt<px4::params::OSD_CURR_X>) _param_osd_curr_x,
		(ParamInt<px4::params::OSD_CURR_Y>) _param_osd_curr_y,
		(ParamInt<px4::params::OSD_MAH_X>) _param_osd_mah_x,
		(ParamInt<px4::params::OSD_MAH_Y>) _param_osd_mah_y,
		(ParamInt<px4::params::OSD_LAT_X>) _param_osd_lat_x,
		(ParamInt<px4::params::OSD_LAT_Y>) _param_osd_lat_y,
		(ParamInt<px4::params::OSD_LON_X>) _param_osd_lon_x,
		(ParamInt<px4::params::OSD_LON_Y>) _param_osd_lon_y,
		(ParamInt<px4::params::OSD_SAT_X>) _param_osd_sat_x,
		(ParamInt<px4::params::OSD_SAT_Y>) _param_osd_sat_y,
		(ParamInt<px4::params::OSD_SPD_X>) _param_osd_spd_x,
		(ParamInt<px4::params::OSD_SPD_Y>) _param_osd_spd_y,
		(ParamInt<px4::params::OSD_HOME_X>) _param_osd_home_x,
		(ParamInt<px4::params::OSD_HOME_Y>) _param_osd_home_y,
		(ParamInt<px4::params::OSD_PITCH_X>) _param_osd_pitch_x,
		(ParamInt<px4::params::OSD_PITCH_Y>) _param_osd_pitch_y,
		(ParamInt<px4::params::OSD_ROLL_X>) _param_osd_roll_x,
		(ParamInt<px4::params::OSD_ROLL_Y>) _param_osd_roll_y,
		(ParamInt<px4::params::OSD_ALT_X>) _param_osd_alt_x,
		(ParamInt<px4::params::OSD_ALT_Y>) _param_osd_alt_y,
		(ParamInt<px4::params::OSD_CH_X>) _param_osd_ch_x,
		(ParamInt<px4::params::OSD_CH_Y>) _param_osd_ch_y
	)

	// metadata
	char _device[64] {};
	PerformanceData _performance_data{};

	msp_set_vtx_config_t vtx_config;
	msp_set_vtxtable_powerlevel_t power_levels[POWER_LEVEL_COUNT];
	msp_set_vtxtable_band_t vtx_bands[BAND_COUNT] {};
	bool has_vtx_config {false};
	bool has_power_config {false};
	bool has_vtx_bands {false};
	bool change_channel {false};
};
