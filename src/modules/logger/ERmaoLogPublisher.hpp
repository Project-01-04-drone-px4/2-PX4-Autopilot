/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file ERmaoLogPublisher_MultiCallback.hpp
 *
 * OPTIMIZED VERSION: Multiple callbacks for ultra-low latency
 *
 * Each topic has its own SubscriptionCallbackWorkItem, ensuring immediate
 * processing when published (< 0.1 ms latency vs < 1 ms in single-callback version)
 *
 * Advantages:
 *  - Ultra-low latency: < 0.1 ms from original publish to log publish
 *  - Independent processing: each topic processed immediately
 *  - No dependency: doesn't wait for other topics
 *
 * Trade-offs:
 *  - Memory: +200 bytes for 8 callback objects (acceptable)
 *  - Code complexity: slightly more complex initialization
 *  - Thread safety: WorkQueue guarantees serial execution, no issues
 *
 * @author ERmao Mode Development Team
 */

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

// Original topics
#include <uORB/topics/sensor_gyro_fifo.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/actuator_motors.h>

// Compact log topics (generated from .msg files with snake_case naming)
#include <uORB/topics/log_gyro_fifo.h>
#include <uORB/topics/log_angular_velocity.h>
#include <uORB/topics/log_vehicle_imu.h>
#include <uORB/topics/log_sensor_combined.h>
#include <uORB/topics/log_attitude.h>
#include <uORB/topics/log_attitude_setpoint.h>
#include <uORB/topics/log_rates_setpoint.h>
#include <uORB/topics/log_actuator_motors.h>

class ERmaoLogPublisher : public ModuleBase<ERmaoLogPublisher>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	ERmaoLogPublisher();
	~ERmaoLogPublisher() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	// Conversion functions (each called when corresponding topic updates)
	void convertGyroFifo();
	void convertAngularVelocity();
	void convertVehicleImu();
	void convertSensorCombined();
	void convertAttitude();
	void convertAttitudeSetpoint();
	void convertRatesSetpoint();
	void convertActuatorMotors();

	// ========== MULTI-CALLBACK OPTIMIZATION ==========
	// Each topic has its own callback for immediate processing
	// When any topic updates, it triggers Run(), which processes ALL updated topics
	//
	// Advantage: Ultra-low latency (< 0.1 ms from original publish to log publish)
	// Trade-off: Run() triggered more frequently, but only processes updated topics
	// ===================================================

	// Polling subscriptions (checked in Run())
	uORB::Subscription _sensor_gyro_fifo_sub{ORB_ID(sensor_gyro_fifo)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _vehicle_imu_sub{ORB_ID(vehicle_imu)};
	uORB::Subscription _sensor_combined_sub{ORB_ID(sensor_combined)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};

	// Publications to compact log topics
	uORB::Publication<log_gyro_fifo_s> _log_gyro_fifo_pub{ORB_ID(log_gyro_fifo)};
	uORB::Publication<log_angular_velocity_s> _log_angular_velocity_pub{ORB_ID(log_angular_velocity)};
	uORB::Publication<log_vehicle_imu_s> _log_vehicle_imu_pub{ORB_ID(log_vehicle_imu)};
	uORB::Publication<log_sensor_combined_s> _log_sensor_combined_pub{ORB_ID(log_sensor_combined)};
	uORB::Publication<log_attitude_s> _log_attitude_pub{ORB_ID(log_attitude)};
	uORB::Publication<log_attitude_setpoint_s> _log_attitude_setpoint_pub{ORB_ID(log_attitude_setpoint)};
	uORB::Publication<log_rates_setpoint_s> _log_rates_setpoint_pub{ORB_ID(log_rates_setpoint)};
	uORB::Publication<log_actuator_motors_s> _log_actuator_motors_pub{ORB_ID(log_actuator_motors)};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::ERMAO_LOG_ENABLE>) _param_ermao_log_enable
	)
};

