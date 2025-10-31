/****************************************************************************
 *
 *   Copyright (c) 2014-2015 PX4 Development Team. All rights reserved.
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
 * @file matlab_csv_serial.c
 *
 * Matlab CSV / ASCII format interface at 921600 baud, 8 data bits,
 * 1 stop bit, no parity
 *

 * Modified to subscribe to fake_imu sensor data and output via serial port
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Modified for fake_imu integration
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/log.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>
#include <float.h>
#include <inttypes.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <poll.h>

#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_gyro_fifo.h>

__EXPORT int matlab_csv_serial_main(int argc, char *argv[]);
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

int matlab_csv_serial_thread_main(int argc, char *argv[]);
static void usage(const char *reason);

static void usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: daemon {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to px4_task_spawn_cmd().
 */
int matlab_csv_serial_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
	}

	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			warnx("already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("matlab_csv_serial",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 5,
						 2000,
						 matlab_csv_serial_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("running");

		} else {
			warnx("stopped");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int matlab_csv_serial_thread_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_ERR("need a serial port name as argument");
		PX4_ERR("usage: matlab_csv_serial start <serial_port>");
		return -1;
	}

	const char *uart_name = argv[1];

	PX4_INFO("opening port %s", uart_name);

	int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

	unsigned speed = 921600;

	if (serial_fd < 0) {
		PX4_ERR("failed to open port: %s", uart_name);
		return -1;
	}

	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;

	/* Back up the original uart configuration to restore it after exit */
	if ((termios_state = tcgetattr(serial_fd, &uart_config)) < 0) {
		PX4_WARN("ERR GET CONF %s: %d", uart_name, termios_state);
		close(serial_fd);
		return -1;
	}

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* USB serial is indicated by /dev/ttyACM0*/
	if (strcmp(uart_name, "/dev/ttyACM0") != 0 && strcmp(uart_name, "/dev/ttyACM1") != 0) {

		/* Set baud rate */
		if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
			PX4_WARN("ERR SET BAUD %s: %d", uart_name, termios_state);
			close(serial_fd);
			return -1;
		}
	}

	if ((termios_state = tcsetattr(serial_fd, TCSANOW, &uart_config)) < 0) {
		PX4_WARN("ERR SET CONF %s", uart_name);
		close(serial_fd);
		return -1;
	}

	PX4_INFO("Serial port configured successfully");
	PX4_INFO("Searching for fake_imu sensor (device ID 1310988)...");

	/* fake_imu uses device ID 1310988 (DRV_IMU_DEVTYPE_SIM) */
	/* In FakeImu.cpp: accel is published before gyro, so we only need to subscribe to gyro */
	struct sensor_accel_s accel;
	struct sensor_gyro_s gyro;

	#define MAX_SENSOR_INSTANCES 8
	#define FAKE_IMU_DEVICE_ID 1310988

	int gyro_instance = -1;
	int accel_instance = -1;

	/* Search for fake_imu gyro instance (primary trigger) */
	for (int i = 0; i < MAX_SENSOR_INSTANCES; i++) {
		int sub = orb_subscribe_multi(ORB_ID(sensor_gyro), i);

		if (sub >= 0) {
			/* Use poll to wait for data (up to 100ms) */
			struct pollfd fds[1];
			fds[0].fd = sub;
			fds[0].events = POLLIN;

			int poll_ret = poll(fds, 1, 100);  // Wait up to 100ms for data

			if (poll_ret > 0 && (fds[0].revents & POLLIN)) {
				if (orb_copy(ORB_ID(sensor_gyro), sub, &gyro) == 0) {
					if (gyro.device_id == FAKE_IMU_DEVICE_ID) {
						gyro_instance = i;
						orb_unsubscribe(sub);
						PX4_INFO("Found fake_imu gyro on instance %d", i);
						break;
					}
				}
			}

			orb_unsubscribe(sub);
		}
	}

	/* Search for fake_imu accel instance (for reading, not triggering) */
	for (int i = 0; i < MAX_SENSOR_INSTANCES; i++) {
		int sub = orb_subscribe_multi(ORB_ID(sensor_accel), i);

		if (sub >= 0) {
			/* Use poll to wait for data (up to 100ms) */
			struct pollfd fds[1];
			fds[0].fd = sub;
			fds[0].events = POLLIN;

			int poll_ret = poll(fds, 1, 100);  // Wait up to 100ms for data

			if (poll_ret > 0 && (fds[0].revents & POLLIN)) {
				if (orb_copy(ORB_ID(sensor_accel), sub, &accel) == 0) {
					if (accel.device_id == FAKE_IMU_DEVICE_ID) {
						accel_instance = i;
						orb_unsubscribe(sub);
						PX4_INFO("Found fake_imu accel on instance %d", i);
						break;
					}
				}
			}

			orb_unsubscribe(sub);
		}
	}

	/* Check if fake_imu was found */
	if (gyro_instance < 0 || accel_instance < 0) {
		PX4_ERR("fake_imu not found! Is it running?");
		PX4_ERR("  gyro_instance: %d, accel_instance: %d", gyro_instance, accel_instance);
		PX4_ERR("  Run 'fake_imu start' first!");
		close(serial_fd);
		return -1;
	}

	/* Subscribe to gyro (primary trigger, like VehicleIMU does) */
	/* Subscribe to accel (for reading only) */
	int gyro_sub = orb_subscribe_multi(ORB_ID(sensor_gyro), gyro_instance);
	int accel_sub = orb_subscribe_multi(ORB_ID(sensor_accel), accel_instance);

	if (gyro_sub < 0 || accel_sub < 0) {
		PX4_ERR("Failed to subscribe to fake_imu instances");
		close(serial_fd);
		return -1;
	}

	PX4_INFO("Subscribed to fake_imu: gyro (trigger) on inst %d, accel (read) on inst %d",
	         gyro_instance, accel_instance);

	thread_running = true;

	PX4_INFO("Started! Writing CSV data to serial port...");
	PX4_INFO("CSV format: timestamp_us,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z");
	PX4_INFO("Strategy: Poll on gyro (like VehicleIMU), accel is always published before gyro");

	// Write CSV header to serial port
	dprintf(serial_fd, "# timestamp_us,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z\n");

	uint32_t sample_count = 0;
	uint64_t start_time = hrt_absolute_time();
	uint64_t last_print_time = start_time;

	/* Setup poll for ONLY gyro subscription (like VehicleIMU does) */
	/* Accel is published before gyro in FakeImu::Run(), so when gyro arrives, accel is ready */
	struct pollfd fds[1];
	fds[0].fd = gyro_sub;
	fds[0].events = POLLIN;

	PX4_INFO("Entering main loop - polling on gyro only (zero latency)");

	while (!thread_should_exit) {

		/* Block waiting for gyro data with poll() */
		/* When gyro arrives, accel is already published (see FakeImu.cpp line 118, 122) */
		int ret = poll(fds, 1, 500);  // 500ms timeout to check exit condition

		if (ret < 0) {
			/* Poll error */
			if (errno != EINTR) {  // Ignore interrupted system call
				PX4_ERR("poll error: %d", errno);
			}

			continue;

		} else if (ret == 0) {
			/* Timeout - no data for 500ms */
			PX4_WARN("No gyro data for 500ms - is fake_imu still running?");
			continue;

		} else {
			/* Gyro data available - this is our trigger (like VehicleIMU) */

			if (fds[0].revents & POLLIN) {
				/* Read gyro first (our trigger) */
				orb_copy(ORB_ID(sensor_gyro), gyro_sub, &gyro);

				/* Verify it's fake_imu data */
				if (gyro.device_id == FAKE_IMU_DEVICE_ID) {

					/* Read corresponding accel (always ready since published before gyro) */
					orb_copy(ORB_ID(sensor_accel), accel_sub, &accel);

					/* Verify accel is also from fake_imu */
					if (accel.device_id == FAKE_IMU_DEVICE_ID) {

						/* Write CSV line with gyro timestamp as reference */
						/* Like VehicleIMU: gyro drives the timing */
						dprintf(serial_fd, "%llu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
						        (unsigned long long)gyro.timestamp,
						        (double)accel.x,
						        (double)accel.y,
						        (double)accel.z,
						        (double)gyro.x,
						        (double)gyro.y,
						        (double)gyro.z);

						sample_count++;
					}
				}
			}

			/* Print status every second */
			uint64_t now = hrt_absolute_time();

			if (now - last_print_time > 1000000) {  // 1 second
				float elapsed_sec = (float)(now - start_time) / 1e6f;
				float rate = (float)sample_count / elapsed_sec;

				PX4_INFO("Samples: %" PRIu32 " (%.1f Hz)", sample_count, (double)rate);

				last_print_time = now;
			}
		}
	}

	PX4_INFO("Exiting... Total samples: %" PRIu32, sample_count);
	thread_running = false;

	close(serial_fd);
	fflush(stdout);
	return 0;
}
