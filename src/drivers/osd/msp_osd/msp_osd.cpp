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

/* Notes:
 *  - Currently there's a lot of wasted processing here if certain displays are enabled.
 *    A relatively low-hanging fruit would be figuring out which display elements require
 *    information from what UORB topics and disable if the information isn't displayed.
 * 	(this is complicated by the fact that it's not a one-to-one mapping...)
 */

#include "msp_osd.hpp"

#include "msp_defines.h"

#include <fcntl.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <stdio.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <lib/mathlib/mathlib.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/power_monitor.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/vehicle_air_data.h>

#include <lib/geo/geo.h>

#include "MspV1.hpp"

namespace
{
constexpr hrt_abstime DISPLAYPORT_FRAME_INTERVAL = 200_ms;
constexpr hrt_abstime CONFIG_FRAME_INTERVAL = 1_s;
constexpr hrt_abstime TELEMETRY_FRAME_INTERVAL = 200_ms;
constexpr hrt_abstime SERIAL_STARTUP_DELAY = 1500_ms;
constexpr hrt_abstime SERIAL_SEND_TIMEOUT = 3_s;
constexpr hrt_abstime DISPLAYPORT_OPTIONS_INTERVAL = 1_s;
constexpr uint8_t SERIAL_SEND_FAILURE_LIMIT = 10;

const char *flight_mode_name(uint8_t nav_state)
{
	switch (nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		return "MANUAL";

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		return "ALTCTL";

	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		return "POSCTL";

	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
		return "MISSION";

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
		return "LOITER";

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
		return "RTL";

	case vehicle_status_s::NAVIGATION_STATE_ACRO:
		return "ACRO";

	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
		return "DESCEND";

	case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
		return "TERM";

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		return "OFFBOARD";

	case vehicle_status_s::NAVIGATION_STATE_STAB:
		return "STAB";

	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
		return "TAKEOFF";

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
		return "LAND";

	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
		return "FOLLOW";

	case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
		return "PRECLAND";

	case vehicle_status_s::NAVIGATION_STATE_ORBIT:
		return "ORBIT";

	case vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF:
		return "VTOL_TKOF";

	default:
		return "UNKNOWN";
	}
}
}

// OSD elements positions.
// The legacy MSP OSD position fields use a 26 x 15 SD grid. The actual
// DisplayPort output below uses the 53 x 20 HD canvas.

// Currently working elements positions (hardcoded)

/* center col

Speed Power Alt
Rssi cell_voltage mah
craft name

*/

MspOsd::MspOsd(const char *device) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	_display.set_period(_param_osd_scroll_rate.get() * 1000ULL);
	_display.set_dwell(_param_osd_dwell_time.get() * 1000ULL);

	// back up device name for connection later
	strcpy(_device, device);

	// _is_initialized = true;
	PX4_INFO("MSP OSD running on %s", _device);
}

MspOsd::~MspOsd()
{
	close_serial();
}

bool MspOsd::init()
{
	ScheduleOnInterval(100_ms);

	return true;
}

void MspOsd::SendConfig()
{
	// Initialize the complete MSP configuration. Several protocol fields are
	// not currently configurable by PX4 but must still be deterministic.
	msp_osd_config_t msp_osd_config{};

	msp_osd_config.units = 0;
	msp_osd_config.video_system = 2; // VIDEO_SYSTEM_HD
	msp_osd_config.osd_item_count = 56;
	msp_osd_config.osd_stat_count = 24;
	msp_osd_config.osd_timer_count = 2;
	msp_osd_config.osd_warning_count = 16;              // 16
	msp_osd_config.osd_profile_count = 1;              // 1
	msp_osd_config.osdprofileindex = 1;                // 1
	msp_osd_config.overlay_radio_mode = 0;             //  0

	// display conditional elements
	msp_osd_config.osd_craft_name_pos = enabled(SymbolIndex::CRAFT_NAME) ?
					    position(_param_osd_craft_x.get(), _param_osd_craft_y.get()) : LOCATION_HIDDEN;
	msp_osd_config.osd_disarmed_pos = enabled(SymbolIndex::DISARMED) ?
					   position(_param_osd_disarmed_x.get(), _param_osd_disarmed_y.get()) : LOCATION_HIDDEN;
	msp_osd_config.osd_gps_lat_pos = enabled(SymbolIndex::GPS_LAT) ?
					  position(_param_osd_gps_lat_x.get(), _param_osd_gps_lat_y.get()) : LOCATION_HIDDEN;
	msp_osd_config.osd_gps_lon_pos = enabled(SymbolIndex::GPS_LON) ?
					  position(_param_osd_gps_lon_x.get(), _param_osd_gps_lon_y.get()) : LOCATION_HIDDEN;
	msp_osd_config.osd_gps_sats_pos = enabled(SymbolIndex::GPS_SATS) ?
					   position(_param_osd_gps_sats_x.get(), _param_osd_gps_sats_y.get()) : LOCATION_HIDDEN;
	msp_osd_config.osd_gps_speed_pos = enabled(SymbolIndex::GPS_SPEED) ?
					    position(_param_osd_gps_speed_x.get(), _param_osd_gps_speed_y.get()) : LOCATION_HIDDEN;
	msp_osd_config.osd_home_dist_pos = enabled(SymbolIndex::HOME_DIST) ?
					    position(_param_osd_home_dist_x.get(), _param_osd_home_dist_y.get()) : LOCATION_HIDDEN;
	msp_osd_config.osd_home_dir_pos = enabled(SymbolIndex::HOME_DIR) ?
					   position(_param_osd_home_dir_x.get(), _param_osd_home_dir_y.get()) : LOCATION_HIDDEN;
	msp_osd_config.osd_main_batt_voltage_pos = enabled(SymbolIndex::MAIN_BATT_VOLTAGE) ?
			position(_param_osd_batt_volt_x.get(), _param_osd_batt_volt_y.get()) : LOCATION_HIDDEN;
	msp_osd_config.osd_current_draw_pos = enabled(SymbolIndex::CURRENT_DRAW) ?
					      position(_param_osd_current_x.get(), _param_osd_current_y.get()) : LOCATION_HIDDEN;
	msp_osd_config.osd_mah_drawn_pos = enabled(SymbolIndex::MAH_DRAWN) ?
					    position(_param_osd_mah_drawn_x.get(), _param_osd_mah_drawn_y.get()) : LOCATION_HIDDEN;
	msp_osd_config.osd_rssi_value_pos = enabled(SymbolIndex::RSSI_VALUE) ?
					    position(_param_osd_rssi_x.get(), _param_osd_rssi_y.get()) : LOCATION_HIDDEN;
	msp_osd_config.osd_altitude_pos = enabled(SymbolIndex::ALTITUDE) ?
					  position(_param_osd_altitude_x.get(), _param_osd_altitude_y.get()) : LOCATION_HIDDEN;
	msp_osd_config.osd_numerical_vario_pos = LOCATION_HIDDEN;

	msp_osd_config.osd_power_pos = enabled(SymbolIndex::POWER) ?
				       position(_param_osd_power_x.get(), _param_osd_power_y.get()) : LOCATION_HIDDEN;
	msp_osd_config.osd_avg_cell_voltage_pos = enabled(SymbolIndex::AVG_CELL_VOLTAGE) ?
			position(_param_osd_cell_volt_x.get(), _param_osd_cell_volt_y.get()) : LOCATION_HIDDEN;

	// the location of our crosshairs can change
	msp_osd_config.osd_crosshairs_pos = LOCATION_HIDDEN;

	if (enabled(SymbolIndex::CROSSHAIRS)) {
		const int32_t crosshair_y = _param_osd_crosshair_y.get() - _param_osd_ch_height.get();
		msp_osd_config.osd_crosshairs_pos = position(_param_osd_crosshair_x.get(), crosshair_y);
	}

	// possibly available, but not currently used
	msp_osd_config.osd_flymode_pos = enabled(SymbolIndex::FLYMODE) ?
					 position(_param_osd_flymode_x.get(), _param_osd_flymode_y.get()) : LOCATION_HIDDEN;
	msp_osd_config.osd_esc_tmp_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_pitch_angle_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_roll_angle_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_horizon_sidebars_pos = 		LOCATION_HIDDEN;

	// Not implemented or not available
	msp_osd_config.osd_artificial_horizon_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_item_timer_1_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_item_timer_2_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_throttle_pos_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_vtx_channel_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_roll_pids_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_pitch_pids_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_yaw_pids_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_pidrate_profile_pos =		LOCATION_HIDDEN;
	msp_osd_config.osd_warnings_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_debug_pos = 				LOCATION_HIDDEN;
	msp_osd_config.osd_main_batt_usage_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_numerical_heading_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_compass_bar_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_esc_rpm_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_remaining_time_estimate_pos = 	LOCATION_HIDDEN;
	msp_osd_config.osd_rtc_datetime_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_adjustment_range_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_core_temperature_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_anti_gravity_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_g_force_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_motor_diag_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_log_status_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_flip_arrow_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_link_quality_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_flight_dist_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_stick_overlay_left_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_stick_overlay_right_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_display_name_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_esc_rpm_freq_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_rate_profile_name_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_pid_profile_name_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_profile_name_pos = 			LOCATION_HIDDEN;
	msp_osd_config.osd_rssi_dbm_value_pos = 		LOCATION_HIDDEN;
	msp_osd_config.osd_rc_channels_pos = 			LOCATION_HIDDEN;

	Send(MSP_OSD_CONFIG, &msp_osd_config);
}

bool MspOsd::initialize_serial()
{
	_msp_fd = open(_device, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_msp_fd < 0) {
		_performance_data.initialization_problems = true;
		return false;
	}

	struct termios t {};

	if (tcgetattr(_msp_fd, &t) != 0) {
		close_serial();
		_performance_data.initialization_problems = true;
		return false;
	}

	cfsetspeed(&t, B115200);
	t.c_cflag |= (CS8 | CLOCAL | CREAD);
	t.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
	t.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	t.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
	t.c_oflag = 0;

	if (tcsetattr(_msp_fd, TCSANOW, &t) != 0) {
		close_serial();
		_performance_data.initialization_problems = true;
		return false;
	}

	// Discard stale bytes left by the bootloader or another owner of the UART.
	tcflush(_msp_fd, TCIOFLUSH);

	_msp = MspV1(_msp_fd);
	_is_initialized = true;
	_displayport_needs_clear = true;
	_displayport_session_reset_pending = true;
	_consecutive_unsuccessful_sends = 0;
	_last_displayport_update = 0;
	_last_config_update = 0;
	_last_telemetry_update = 0;
	_last_displayport_options_update = 0;

	const hrt_abstime now = hrt_absolute_time();
	_serial_startup_time = now + SERIAL_STARTUP_DELAY;
	_last_successful_send = now;

	PX4_INFO("MSP OSD serial ready on %s", _device);
	return true;
}

void MspOsd::close_serial()
{
	if (_msp_fd >= 0) {
		close(_msp_fd);
		_msp_fd = -1;
	}

	_msp = MspV1(-1);
	_is_initialized = false;
	_displayport_needs_clear = true;
	_displayport_session_reset_pending = true;
	_last_displayport_update = 0;
	_last_config_update = 0;
	_last_telemetry_update = 0;
	_last_displayport_options_update = 0;
	_consecutive_unsuccessful_sends = 0;
}

void MspOsd::register_send_result(bool success)
{
	if (success) {
		_performance_data.successful_sends++;
		_consecutive_unsuccessful_sends = 0;
		_last_successful_send = hrt_absolute_time();

	} else {
		_performance_data.unsuccessful_sends++;
		_last_failed_send = hrt_absolute_time();

		if (_consecutive_unsuccessful_sends < UINT8_MAX) {
			_consecutive_unsuccessful_sends++;
		}
	}
}

void MspOsd::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
			_parameter_update_sub.copy(&param_update);
			updateParams(); // update module parameters (in DEFINE_PARAMETERS)
			parameters_update();
			_displayport_needs_clear = true;
		}

	// perform first time initialization, if needed
	if (!_is_initialized) {
		if (!initialize_serial()) {
			return;
		}
	}

	const hrt_abstime now = hrt_absolute_time();

	// Give the external OSD/video device time to finish its own boot before
	// sending the first MSP frame.
	if (now < _serial_startup_time) {
		return;
	}

	// A UART can remain open after its peer has disappeared or was powered up
	// later. Reopen it so the next periodic frame starts a clean session.
	if (_consecutive_unsuccessful_sends >= SERIAL_SEND_FAILURE_LIMIT
	    || now - _last_successful_send > SERIAL_SEND_TIMEOUT) {
		close_serial();
		return;
	}

	// avoid premature pessimization; if skip processing if we're effectively disabled
	if (_param_osd_symbols.get() == 0) {
		return;
	}

	if (now - _last_telemetry_update >= TELEMETRY_FRAME_INTERVAL) {
		_last_telemetry_update = now;

		// update display message
		{
		vehicle_status_s vehicle_status{};
		_vehicle_status_sub.copy(&vehicle_status);

		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);

		log_message_s log_message{};
		_log_message_sub.copy(&log_message);

		const auto display_message = msp_osd::construct_display_message(
						     vehicle_status,
						     vehicle_attitude,
						     log_message,
						     _param_osd_log_level.get(),
						     _display);
		this->Send(MSP_NAME, &display_message);
		}

		// MSP_FC_VARIANT
		{
		const auto msg = msp_osd::construct_FC_VARIANT();
		this->Send(MSP_FC_VARIANT, &msg);
		}

		// MSP_STATUS
		{
		vehicle_status_s vehicle_status{};
		_vehicle_status_sub.copy(&vehicle_status);

		const auto msg = msp_osd::construct_STATUS(vehicle_status);
		this->Send(MSP_STATUS, &msg);
		}

		// MSP_ANALOG
		{
		battery_status_s battery_status{};
		_battery_status_sub.copy(&battery_status);

		input_rc_s input_rc{};
		_input_rc_sub.copy(&input_rc);

		const auto msg = msp_osd::construct_ANALOG(
					 battery_status,
					 input_rc);
		this->Send(MSP_ANALOG, &msg);
		}

		// MSP_BATTERY_STATE
		{
		battery_status_s battery_status{};
		_battery_status_sub.copy(&battery_status);

		const auto msg = msp_osd::construct_BATTERY_STATE(battery_status);
		this->Send(MSP_BATTERY_STATE, &msg);
		}

		// MSP_RAW_GPS
		{
		sensor_gps_s vehicle_gps_position{};
		_vehicle_gps_position_sub.copy(&vehicle_gps_position);

		airspeed_validated_s airspeed_validated{};
		_airspeed_validated_sub.copy(&airspeed_validated);

		const auto msg = msp_osd::construct_RAW_GPS(
					 vehicle_gps_position,
					 airspeed_validated);
		this->Send(MSP_RAW_GPS, &msg);
		}

		// MSP_COMP_GPS
		{
		// update heartbeat
		_heartbeat = !_heartbeat;

		home_position_s home_position{};
		_home_position_sub.copy(&home_position);

		estimator_status_s estimator_status{};
		_estimator_status_sub.copy(&estimator_status);

		vehicle_global_position_s vehicle_global_position{};
		_vehicle_global_position_sub.copy(&vehicle_global_position);

		// construct and send message
		const auto msg = msp_osd::construct_COMP_GPS(
					 home_position,
					 estimator_status,
					 vehicle_global_position,
					 _heartbeat);
		this->Send(MSP_COMP_GPS, &msg);
		}

		// MSP_ATTITUDE
		{
		vehicle_attitude_s vehicle_attitude{};
		_vehicle_attitude_sub.copy(&vehicle_attitude);

		const auto msg = msp_osd::construct_ATTITUDE(vehicle_attitude);
		this->Send(MSP_ATTITUDE, &msg);
		}

		// MSP_ALTITUDE
		{
		sensor_gps_s vehicle_gps_position{};
		_vehicle_gps_position_sub.copy(&vehicle_gps_position);

		estimator_status_s estimator_status{};
		_estimator_status_sub.copy(&estimator_status);

		vehicle_local_position_s vehicle_local_position{};
		_vehicle_local_position_sub.copy(&vehicle_local_position);

		// construct and send message
		const auto msg = msp_osd::construct_ALTITUDE(
					 vehicle_gps_position,
					 estimator_status,
					 vehicle_local_position);
		this->Send(MSP_ALTITUDE, &msg);
		}

		// MSP_MOTOR_TELEMETRY
		{
		const auto msg = msp_osd::construct_ESC_SENSOR_DATA();
		this->Send(MSP_ESC_SENSOR_DATA, &msg);
		}
	}

	if (now - _last_config_update >= CONFIG_FRAME_INTERVAL) {
		SendConfig();
		_last_config_update = now;
	}

	if (now - _last_displayport_update >= DISPLAYPORT_FRAME_INTERVAL) {
		// Send the actual character-based DisplayPort frame used by modern
		// digital video systems (DJI, Walksnail and HDZero).
		SendDisplayPort();
		_last_displayport_update = now;
	}
}

void MspOsd::Send(const unsigned int message_type, const void *payload)
{
	register_send_result(_msp.Send(message_type, payload));
}

bool MspOsd::SendDisplayPortText(uint8_t x, uint8_t y, const char *text, uint8_t attributes)
{
	if (text == nullptr) {
		return false;
	}

	const size_t text_length = strnlen(text, DISPLAYPORT_CANVAS_COLUMNS);
	uint8_t payload[4 + DISPLAYPORT_CANVAS_COLUMNS] {};
	payload[0] = MSP_DP_WRITE_STRING;
	payload[1] = y;
	payload[2] = x;
	payload[3] = attributes;
	memcpy(&payload[4], text, text_length);

	_displayport_write_count++;
	const bool result = _msp.SendPayload(MSP_DISPLAYPORT, payload, text_length + 4);

	register_send_result(result);

	return result;
}

void MspOsd::SendDisplayPort()
{
	const uint8_t heartbeat[] = {MSP_DP_HEARTBEAT};
	const uint8_t release[] = {MSP_DP_RELEASE};
	const uint8_t options[] = {MSP_DP_OPTIONS, 0, MSP_DP_CANVAS_HD_5320};
	const uint8_t clear_screen[] = {MSP_DP_CLEAR_SCREEN};
	const uint8_t draw_screen[] = {MSP_DP_DRAW_SCREEN};

	auto send_command = [this](const uint8_t *payload, size_t size) {
		if (size > 0) {
			switch (payload[0]) {
			case MSP_DP_RELEASE:
				_displayport_release_count++;
				break;

			case MSP_DP_HEARTBEAT:
				_displayport_heartbeat_count++;
				break;

			case MSP_DP_OPTIONS:
				_displayport_options_count++;
				break;

			case MSP_DP_CLEAR_SCREEN:
				_displayport_clear_count++;
				break;

			case MSP_DP_DRAW_SCREEN:
				_displayport_draw_count++;
				break;
			}
		}

		const bool result = _msp.SendPayload(MSP_DISPLAYPORT, payload, size);
		register_send_result(result);
		return result;
	};
	auto x_coord = [](int32_t value) {
		return static_cast<uint8_t>(math::constrain(value, static_cast<int32_t>(0),
				static_cast<int32_t>(DISPLAYPORT_CANVAS_COLUMNS - 1)));
	};
	auto y_coord = [](int32_t value) {
		return static_cast<uint8_t>(math::constrain(value, static_cast<int32_t>(0),
				static_cast<int32_t>(DISPLAYPORT_CANVAS_ROWS - 1)));
	};

	const hrt_abstime now = hrt_absolute_time();
	const bool session_reset = _displayport_session_reset_pending;
	bool session_reset_success = true;
	_displayport_frame_count++;

	// A powered video receiver can keep the previous DisplayPort session while
	// the flight controller reboots. Explicitly release that session before
	// grabbing it again, otherwise the receiver may continue showing a stale
	// session and ignore the new flight controller stream.
	if (session_reset) {
		session_reset_success = send_command(release, sizeof(release));
	}

	session_reset_success = send_command(heartbeat, sizeof(heartbeat)) && session_reset_success;

	const bool options_update = session_reset
				     || _displayport_needs_clear
				     || now - _last_displayport_options_update >= DISPLAYPORT_OPTIONS_INTERVAL;

	// Tell the goggles which HD canvas is in use. This is separate from the
	// legacy MSP_OSD_CONFIG video_system field and is required by many HD
	// DisplayPort receivers before they accept 53x20 coordinates.
	if (options_update) {
		const bool options_success = send_command(options, sizeof(options));
		session_reset_success = options_success && session_reset_success;

		if (options_success) {
			_last_displayport_options_update = now;
		}
	}

	if (_displayport_needs_clear) {
		const bool clear_success = send_command(clear_screen, sizeof(clear_screen));
		session_reset_success = clear_success && session_reset_success;

		if (clear_success) {
			_displayport_needs_clear = false;
		}
	}

	if (session_reset && session_reset_success) {
		_displayport_session_reset_pending = false;
	}

	vehicle_status_s vehicle_status{};
	_vehicle_status_sub.copy(&vehicle_status);

	char message[FULL_MSG_BUFFER] {};
	_display.get(message, hrt_absolute_time());

	if (enabled(SymbolIndex::CRAFT_NAME)) {
		SendDisplayPortText(x_coord(_param_osd_craft_x.get()), y_coord(_param_osd_craft_y.get()), message);
	}

	if (enabled(SymbolIndex::DISARMED)) {
		const char *arming = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED ? "ARM" : "DISARM";
		char text[8];
		snprintf(text, sizeof(text), "%-6s", arming);
		SendDisplayPortText(x_coord(_param_osd_disarmed_x.get()), y_coord(_param_osd_disarmed_y.get()), text);
	}

	if (enabled(SymbolIndex::FLYMODE)) {
		// The value itself identifies the flight mode; avoid spending five
		// columns on a redundant "MODE:" prefix.
		char text[16];
		snprintf(text, sizeof(text), "%-8s", flight_mode_name(vehicle_status.nav_state));
		SendDisplayPortText(x_coord(_param_osd_flymode_x.get()), y_coord(_param_osd_flymode_y.get()), text);
	}

	if (enabled(SymbolIndex::GPS_SATS)) {
		sensor_gps_s gps{};
		_vehicle_gps_position_sub.copy(&gps);
		char text[12];
		snprintf(text, sizeof(text), "S:%02u", static_cast<unsigned>(gps.satellites_used));
		SendDisplayPortText(x_coord(_param_osd_gps_sats_x.get()), y_coord(_param_osd_gps_sats_y.get()), text);
	}

	if (enabled(SymbolIndex::ALTITUDE)) {
		vehicle_local_position_s local_position{};
		_vehicle_local_position_sub.copy(&local_position);
		char text[16];
		const float altitude = local_position.z_valid ? -local_position.z : 0.f;
		snprintf(text, sizeof(text), "%+7.1fm", static_cast<double>(altitude));
		SendDisplayPortText(x_coord(_param_osd_altitude_x.get()), y_coord(_param_osd_altitude_y.get()), text);
	}

	if (enabled(SymbolIndex::MAIN_BATT_VOLTAGE) || enabled(SymbolIndex::CURRENT_DRAW)
	    || enabled(SymbolIndex::MAH_DRAWN) || enabled(SymbolIndex::AVG_CELL_VOLTAGE)
	    || enabled(SymbolIndex::POWER)) {
		battery_status_s battery{};
		_battery_status_sub.copy(&battery);

		if (enabled(SymbolIndex::MAIN_BATT_VOLTAGE)) {
			char text[12];
			snprintf(text, sizeof(text), "%5.2fV", static_cast<double>(battery.voltage_v));
			SendDisplayPortText(x_coord(_param_osd_batt_volt_x.get()), y_coord(_param_osd_batt_volt_y.get()), text);
		}

		if (enabled(SymbolIndex::AVG_CELL_VOLTAGE)) {
			char text[12];
			const float average_cell_voltage = battery.cell_count > 0
							   ? battery.voltage_v / battery.cell_count
							   : 0.f;
			snprintf(text, sizeof(text), "C%4.2fV", static_cast<double>(average_cell_voltage));
			SendDisplayPortText(x_coord(_param_osd_cell_volt_x.get()), y_coord(_param_osd_cell_volt_y.get()), text);
		}

		if (enabled(SymbolIndex::CURRENT_DRAW)) {
			char text[12];
			snprintf(text, sizeof(text), "%5.1fA", static_cast<double>(battery.current_a));
			SendDisplayPortText(x_coord(_param_osd_current_x.get()), y_coord(_param_osd_current_y.get()), text);
		}

		if (enabled(SymbolIndex::MAH_DRAWN)) {
			char text[16];
			snprintf(text, sizeof(text), "%5umAh", static_cast<unsigned>(battery.discharged_mah));
			SendDisplayPortText(x_coord(_param_osd_mah_drawn_x.get()), y_coord(_param_osd_mah_drawn_y.get()), text);
		}

		if (enabled(SymbolIndex::POWER)) {
			char text[12];
			snprintf(text, sizeof(text), "%5.0fW",
				 static_cast<double>(battery.voltage_v * battery.current_a));
			SendDisplayPortText(x_coord(_param_osd_power_x.get()), y_coord(_param_osd_power_y.get()), text);
		}
	}

	if (enabled(SymbolIndex::RSSI_VALUE)) {
		input_rc_s input_rc{};
		_input_rc_sub.copy(&input_rc);
		char text[12];
		snprintf(text, sizeof(text), "%3u%%", static_cast<unsigned>(input_rc.link_quality));
		SendDisplayPortText(x_coord(_param_osd_rssi_x.get()), y_coord(_param_osd_rssi_y.get()), text);
	}

	send_command(draw_screen, sizeof(draw_screen));
}

void MspOsd::parameters_update()
{
	// update our display rate and dwell time
	_display.set_period(hrt_abstime(_param_osd_scroll_rate.get() * 1000ULL));
	_display.set_dwell(hrt_abstime(_param_osd_dwell_time.get() * 1000ULL));
}

bool MspOsd::enabled(const SymbolIndex &symbol)
{
	return _param_osd_symbols.get() & (1u << symbol);
}

uint16_t MspOsd::position(int32_t x, int32_t y) const
{
	x = math::constrain(x, static_cast<int32_t>(0), static_cast<int32_t>(25));
	y = math::constrain(y, static_cast<int32_t>(0), static_cast<int32_t>(14));
	return static_cast<uint16_t>(2048 + x + 32 * y);
}

int MspOsd::task_spawn(int argc, char *argv[])
{
	// initialize device
	const char *device = nullptr;
	bool error_flag = false;

	// loop through input arguments
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return PX4_ERROR;
	}

	if (!device) {
		PX4_ERR("Missing device");
		return PX4_ERROR;
	}

	MspOsd *instance = new MspOsd(device);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MspOsd::print_status()
{
	const hrt_abstime now = hrt_absolute_time();
	const auto age_ms = [now](hrt_abstime timestamp) -> unsigned long long {
		return timestamp > 0 && now >= timestamp
		       ? static_cast<unsigned long long>((now - timestamp) / 1000)
		       : 0;
	};

	PX4_INFO("Running on %s", _device);
	PX4_INFO("\tinitialized: %d", _is_initialized);
	PX4_INFO("\tinitialization issues: %d", _performance_data.initialization_problems);
	PX4_INFO("\tserial: 115200 8N1, MSP direction: $M>");
	PX4_INFO("\tscroll rate: %d", static_cast<int>(_param_osd_scroll_rate.get()));
	PX4_INFO("\tsuccessful sends: %lu", _performance_data.successful_sends);
	PX4_INFO("\tunsuccessful sends: %lu", _performance_data.unsuccessful_sends);
	PX4_INFO("\tconsecutive send failures: %u", _consecutive_unsuccessful_sends);
	PX4_INFO("\tlast successful send: %llu ms ago", age_ms(_last_successful_send));
	PX4_INFO("\tlast failed send: %llu ms ago", age_ms(_last_failed_send));
	PX4_INFO("\tstartup delay pending: %d", now < _serial_startup_time);
	PX4_INFO("\tDP session reset pending: %d", _displayport_session_reset_pending);
	PX4_INFO("\tDP clear pending: %d", _displayport_needs_clear);
	PX4_INFO("\tDP frames: %lu, release: %lu, heartbeat: %lu",
		 static_cast<unsigned long>(_displayport_frame_count),
		 static_cast<unsigned long>(_displayport_release_count),
		 static_cast<unsigned long>(_displayport_heartbeat_count));
	PX4_INFO("\tDP options: %lu, clear: %lu, write: %lu, draw: %lu",
		 static_cast<unsigned long>(_displayport_options_count),
		 static_cast<unsigned long>(_displayport_clear_count),
		 static_cast<unsigned long>(_displayport_write_count),
		 static_cast<unsigned long>(_displayport_draw_count));
	PX4_INFO("\tDP canvas: %ux%u, options mode: %u",
		 DISPLAYPORT_CANVAS_COLUMNS,
		 DISPLAYPORT_CANVAS_ROWS,
		 MSP_DP_CANVAS_HD_5320);

	// print current display string
	char msg[FULL_MSG_BUFFER];
	_display.get(msg, hrt_absolute_time());
	PX4_INFO("Current message: \n\t%s", msg);

	return 0;
}

int MspOsd::custom_command(int argc, char *argv[])
{
	return 0;
}

int MspOsd::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
MSP telemetry streamer

### Implementation
Converts uORB messages to MSP telemetry packets

### Examples
CLI usage example:
$ msp_osd

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("msp_osd", "driver");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int msp_osd_main(int argc, char *argv[])
{
	return MspOsd::main(argc, argv);
}
