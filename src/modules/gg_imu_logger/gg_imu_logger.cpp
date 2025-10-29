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

#include "gg_imu_logger.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>

#include <sys/stat.h>
#include <time.h>

// ulog文件格式定义
#define ULOG_MSG_HEADER_LEN 3
#define ULOG_MSG_TYPE_FLAG_BITS 1
#define ULOG_MSG_TYPE_DATA 'D'
#define ULOG_MSG_TYPE_INFO 'I'
#define ULOG_MSG_TYPE_FORMAT 'F'
#define ULOG_MSG_TYPE_PARAMETER 'P'

GgImuLogger::GgImuLogger() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

GgImuLogger::~GgImuLogger()
{
	CloseLogFile();

	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
	perf_free(_write_perf);
}

bool GgImuLogger::init()
{
	ParametersUpdate(true);

	// 打开日志文件
	if (!OpenLogFile()) {
		PX4_ERR("Failed to open log file");
		return false;
	}

	_initialized = true;

	// 根据配置的频率调度运行
	int rate_hz = _param_log_rate.get();
	if (rate_hz <= 0) {
		rate_hz = 100; // 默认100Hz
	}

	uint32_t interval_us = 1000000 / rate_hz;
	ScheduleOnInterval(interval_us);

	return true;
}

void GgImuLogger::Run()
{
	if (should_exit()) {
		ScheduleClear();
		CloseLogFile();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// 检查参数更新
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		ParametersUpdate();
	}

	// 获取IMU数据并记录
	const int imu_instance = _param_imu_instance.get();

	if (imu_instance == 0) {
		// 记录所有实例
		for (uint8_t i = 0; i < _vehicle_imu_subs.size(); i++) {
			vehicle_imu_s imu_data;
			if (_vehicle_imu_subs[i].update(&imu_data)) {
				LogImuData(imu_data, i);
			}
		}
	} else {
		// 记录指定实例
		uint8_t instance_index = imu_instance - 1;
		if (instance_index < _vehicle_imu_subs.size()) {
			vehicle_imu_s imu_data;
			if (_vehicle_imu_subs[instance_index].update(&imu_data)) {
				LogImuData(imu_data, instance_index);
			}
		}
	}

	_last_run = hrt_absolute_time();
	perf_end(_loop_perf);
}

bool GgImuLogger::ParametersUpdate(bool force)
{
	bool updated = force;

	if (_parameter_update_sub.updated() || force) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		updated = true;
	}

	return updated;
}

bool GgImuLogger::OpenLogFile()
{
	// 确保日志目录存在
	const char *log_dir = "/fs/microsd/log_gg_imu";
	mkdir(log_dir, S_IRWXU | S_IRWXG | S_IRWXO);

	// 生成文件名（带时间戳）
	time_t now = time(nullptr);
	struct tm *timeinfo = localtime(&now);

	const char *instance_str = "";
	int imu_instance = _param_imu_instance.get();
	if (imu_instance == 1) {
		instance_str = "_imu1";
	} else if (imu_instance == 2) {
		instance_str = "_imu2";
	} else {
		instance_str = "_all";
	}

	snprintf(_log_filename, sizeof(_log_filename),
	         "%s/%04d%02d%02d_%02d%02d%02d%s_%03d.ulg",
	         log_dir,
	         timeinfo->tm_year + 1900,
	         timeinfo->tm_mon + 1,
	         timeinfo->tm_mday,
	         timeinfo->tm_hour,
	         timeinfo->tm_min,
	         timeinfo->tm_sec,
	         instance_str,
	         _log_sequence);

	// 打开文件
	_log_fd = ::open(_log_filename, O_CREAT | O_WRONLY | O_TRUNC, PX4_O_MODE_666);

	if (_log_fd < 0) {
		PX4_ERR("Failed to open log file: %s (errno=%d)", _log_filename, errno);
		return false;
	}

	_log_start_time = hrt_absolute_time();
	_log_write_count = 0;

	// 写入ulog文件头
	if (!WriteULogHeader()) {
		::close(_log_fd);
		_log_fd = -1;
		return false;
	}

	PX4_INFO("Log file opened: %s", _log_filename);
	return true;
}

void GgImuLogger::CloseLogFile()
{
	if (_log_fd >= 0) {
		::close(_log_fd);
		_log_fd = -1;

		PX4_INFO("Log file closed: %s (wrote %llu entries)", _log_filename, _log_write_count);
		_log_sequence++;
	}
}

bool GgImuLogger::WriteULogHeader()
{
	// ULog文件魔术字节
	const uint8_t magic[] = {'U', 'L', 'o', 'g', 0x01, 0x12, 0x35};
	if (::write(_log_fd, magic, sizeof(magic)) != sizeof(magic)) {
		return false;
	}

	// 写入时间戳
	uint64_t timestamp = hrt_absolute_time();
	if (::write(_log_fd, &timestamp, sizeof(timestamp)) != sizeof(timestamp)) {
		return false;
	}

	// 写入格式定义（简化版）
	const char *format_msg = "F\x00\x00vehicle_imu uint64_t timestamp;uint64_t timestamp_sample;"
	                         "uint32_t accel_device_id;uint32_t gyro_device_id;"
	                         "float[3] delta_angle;float[3] delta_velocity;"
	                         "uint16_t delta_angle_dt;uint16_t delta_velocity_dt;"
	                         "uint8_t delta_velocity_clipping;uint8_t delta_angle_clipping;";

	uint16_t msg_size = strlen(format_msg + 3);
	format_msg[1] = msg_size & 0xFF;
	format_msg[2] = (msg_size >> 8) & 0xFF;

	if (::write(_log_fd, format_msg, msg_size + 3) != (ssize_t)(msg_size + 3)) {
		return false;
	}

	return true;
}

void GgImuLogger::LogImuData(const vehicle_imu_s &imu_data, uint8_t instance)
{
	if (_log_fd < 0) {
		return;
	}

	perf_begin(_write_perf);

	// 准备数据缓冲区
	struct __attribute__((packed)) {
		uint8_t msg_type;
		uint16_t msg_size;
		uint16_t msg_id;
		vehicle_imu_s data;
	} log_msg;

	log_msg.msg_type = ULOG_MSG_TYPE_DATA;
	log_msg.msg_size = sizeof(vehicle_imu_s);
	log_msg.msg_id = instance; // 使用实例号作为消息ID
	log_msg.data = imu_data;

	// 写入数据
	ssize_t written = ::write(_log_fd, &log_msg, sizeof(log_msg));

	if (written == sizeof(log_msg)) {
		_log_write_count++;
	} else {
		PX4_ERR("Write failed: %d (errno=%d)", (int)written, errno);
	}

	perf_end(_write_perf);
}

int GgImuLogger::task_spawn(int argc, char *argv[])
{
	GgImuLogger *instance = new GgImuLogger();

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

int GgImuLogger::print_status()
{
	PX4_INFO("Running");
	PX4_INFO("Log file: %s", _log_filename);
	PX4_INFO("Entries written: %llu", _log_write_count);
	PX4_INFO("IMU instance: %d (0=all, 1=imu1, 2=imu2)", _param_imu_instance.get());
	PX4_INFO("Log rate: %d Hz", _param_log_rate.get());

	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	perf_print_counter(_write_perf);

	return 0;
}

int GgImuLogger::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GgImuLogger::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
GG IMU Logger - 订阅并记录vehicle_imu数据到独立的ulog文件

该模块可以订阅vehicle_imu的单个或多个实例，并将数据记录到SD卡上的独立日志文件中。
日志文件存储在 /fs/microsd/log_gg_imu/ 目录下。

### Implementation
模块运行在lp_default work queue中，周期性读取vehicle_imu数据并写入日志文件。
记录频率可通过GG_LOG_RATE参数配置。

### Examples
启动并记录所有IMU实例：
$ gg_imu_logger start

启动并只记录IMU1：
$ param set GG_IMU_INSTANCE 1
$ gg_imu_logger start

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gg_imu_logger", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int gg_imu_logger_main(int argc, char *argv[])
{
	return GgImuLogger::main(argc, argv);
}

