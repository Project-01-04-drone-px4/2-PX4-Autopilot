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
 * @file gg_imu_logger.hpp
 *
 * GG IMU Logger - 订阅vehicle_imu并记录到独立的ulog文件
 *
 * 功能：
 * - 订阅vehicle_imu单例或多实例数据
 * - 将数据记录到独立的ulog文件
 * - 支持通过参数控制启动和配置
 *
 * @author GG
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/parameter_update.h>

#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>

#include <fcntl.h>
#include <unistd.h>

extern "C" __EXPORT int gg_imu_logger_main(int argc, char *argv[]);

class GgImuLogger : public ModuleBase<GgImuLogger>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	GgImuLogger();
	~GgImuLogger() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();

private:
	void Run() override;

	bool ParametersUpdate(bool force = false);

	/**
	 * 打开日志文件
	 */
	bool OpenLogFile();

	/**
	 * 关闭日志文件
	 */
	void CloseLogFile();

	/**
	 * 写入ulog文件头
	 */
	bool WriteULogHeader();

	/**
	 * 写入IMU数据到日志
	 */
	void LogImuData(const vehicle_imu_s &imu_data, uint8_t instance);

	// 参数
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::GG_IMU_INSTANCE>) _param_imu_instance,  ///< IMU实例选择: 0=全部, 1=实例1, 2=实例2
		(ParamInt<px4::params::GG_LOG_RATE>) _param_log_rate           ///< 日志记录频率(Hz)
	)

	// uORB订阅
	uORB::SubscriptionMultiArray<vehicle_imu_s, 3> _vehicle_imu_subs{ORB_ID::vehicle_imu};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// 日志文件
	int _log_fd{-1};
	char _log_filename[64]{};
	uint32_t _log_sequence{0};
	hrt_abstime _log_start_time{0};
	uint64_t _log_write_count{0};

	// 性能计数器
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
	perf_counter_t _write_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": write")};

	// 运行状态
	bool _initialized{false};
	hrt_abstime _last_run{0};
};

