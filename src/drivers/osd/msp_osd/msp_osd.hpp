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

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/SubscriptionInterval.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/estimator_status.h>
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

// location to "hide" unused display elements
#define LOCATION_HIDDEN 234

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

class MspOsd : public ModuleBase<MspOsd>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
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

private:
	void Run() override;

	// update a single display element in the display
	void Send(const unsigned int message_type, const void *payload);

	// send full configuration to MSP (triggers the actual update)
	void SendConfig();
	void SendDisplayPort();
	bool SendDisplayPortText(uint8_t x, uint8_t y, const char *text, uint8_t attributes = 0);
	void SendTelemetry();

	// perform actions required for local updates
	void parameters_update();

	// convenience function to check if a given symbol is enabled
	bool enabled(const SymbolIndex &symbol);
	uint16_t position(int32_t x, int32_t y) const;

	MspV1 _msp{0};
	int _msp_fd{-1};

	msp_osd::MessageDisplay _display{};

	bool _is_initialized{false};

	// subscriptions to desired vehicle display information
	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _estimator_status_sub{ORB_ID(estimator_status)};
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
	bool _displayport_needs_clear{true};
	hrt_abstime _last_displayport_update{0};
	hrt_abstime _last_config_update{0};
	hrt_abstime _last_telemetry_update{0};

	// parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::OSD_SYMBOLS>) _param_osd_symbols,
		(ParamInt<px4::params::OSD_CH_HEIGHT>) _param_osd_ch_height,
		(ParamInt<px4::params::OSD_SCROLL_RATE>) _param_osd_scroll_rate,
		(ParamInt<px4::params::OSD_DWELL_TIME>) _param_osd_dwell_time,
		(ParamInt<px4::params::OSD_LOG_LEVEL>) _param_osd_log_level,
		(ParamInt<px4::params::OSD_CRAFT_X>) _param_osd_craft_x,
		(ParamInt<px4::params::OSD_CRAFT_Y>) _param_osd_craft_y,
		(ParamInt<px4::params::OSD_DISARMED_X>) _param_osd_disarmed_x,
		(ParamInt<px4::params::OSD_DISARMED_Y>) _param_osd_disarmed_y,
		(ParamInt<px4::params::OSD_GPS_LAT_X>) _param_osd_gps_lat_x,
		(ParamInt<px4::params::OSD_GPS_LAT_Y>) _param_osd_gps_lat_y,
		(ParamInt<px4::params::OSD_GPS_LON_X>) _param_osd_gps_lon_x,
		(ParamInt<px4::params::OSD_GPS_LON_Y>) _param_osd_gps_lon_y,
		(ParamInt<px4::params::OSD_GPS_SATS_X>) _param_osd_gps_sats_x,
		(ParamInt<px4::params::OSD_GPS_SATS_Y>) _param_osd_gps_sats_y,
		(ParamInt<px4::params::OSD_GPS_SPEED_X>) _param_osd_gps_speed_x,
		(ParamInt<px4::params::OSD_GPS_SPEED_Y>) _param_osd_gps_speed_y,
		(ParamInt<px4::params::OSD_HOME_DIST_X>) _param_osd_home_dist_x,
		(ParamInt<px4::params::OSD_HOME_DIST_Y>) _param_osd_home_dist_y,
		(ParamInt<px4::params::OSD_HOME_DIR_X>) _param_osd_home_dir_x,
		(ParamInt<px4::params::OSD_HOME_DIR_Y>) _param_osd_home_dir_y,
		(ParamInt<px4::params::OSD_BATT_VOLT_X>) _param_osd_batt_volt_x,
		(ParamInt<px4::params::OSD_BATT_VOLT_Y>) _param_osd_batt_volt_y,
		(ParamInt<px4::params::OSD_CURRENT_X>) _param_osd_current_x,
		(ParamInt<px4::params::OSD_CURRENT_Y>) _param_osd_current_y,
		(ParamInt<px4::params::OSD_MAH_DRAWN_X>) _param_osd_mah_drawn_x,
		(ParamInt<px4::params::OSD_MAH_DRAWN_Y>) _param_osd_mah_drawn_y,
		(ParamInt<px4::params::OSD_RSSI_X>) _param_osd_rssi_x,
		(ParamInt<px4::params::OSD_RSSI_Y>) _param_osd_rssi_y,
		(ParamInt<px4::params::OSD_ALTITUDE_X>) _param_osd_altitude_x,
		(ParamInt<px4::params::OSD_ALTITUDE_Y>) _param_osd_altitude_y,
		(ParamInt<px4::params::OSD_CROSSHAIR_X>) _param_osd_crosshair_x,
		(ParamInt<px4::params::OSD_CROSSHAIR_Y>) _param_osd_crosshair_y,
		(ParamInt<px4::params::OSD_CELL_VOLT_X>) _param_osd_cell_volt_x,
		(ParamInt<px4::params::OSD_CELL_VOLT_Y>) _param_osd_cell_volt_y,
		(ParamInt<px4::params::OSD_POWER_X>) _param_osd_power_x,
		(ParamInt<px4::params::OSD_POWER_Y>) _param_osd_power_y
	)

	// metadata
	char _device[64] {};
	PerformanceData _performance_data{};
};
