/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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
 * @file fake_imu_params.c
 * Parameters for fake IMU chirp signal generator
 */

/**
 * Fake IMU X axis start frequency
 *
 * Start frequency for X axis chirp signal
 *
 * @unit Hz
 * @min 0
 * @max 1000
 * @decimal 1
 * @group Fake IMU
 */
PARAM_DEFINE_FLOAT(FAKE_IMU_X_F0, 0.0f);

/**
 * Fake IMU X axis stop frequency
 *
 * Stop frequency for X axis chirp signal
 *
 * @unit Hz
 * @min 0
 * @max 1000
 * @decimal 1
 * @group Fake IMU
 */
PARAM_DEFINE_FLOAT(FAKE_IMU_X_F1, 10.0f);

/**
 * Fake IMU Y axis start frequency
 *
 * Start frequency for Y axis chirp signal
 *
 * @unit Hz
 * @min 0
 * @max 2000
 * @decimal 1
 * @group Fake IMU
 */
PARAM_DEFINE_FLOAT(FAKE_IMU_Y_F0, 0.0f);

/**
 * Fake IMU Y axis stop frequency
 *
 * Stop frequency for Y axis chirp signal
 *
 * @unit Hz
 * @min 0
 * @max 2000
 * @decimal 1
 * @group Fake IMU
 */
PARAM_DEFINE_FLOAT(FAKE_IMU_Y_F1, 100.0f);

/**
 * Fake IMU Z axis start frequency
 *
 * Start frequency for Z axis chirp signal
 *
 * @unit Hz
 * @min 0
 * @max 4000
 * @decimal 1
 * @group Fake IMU
 */
PARAM_DEFINE_FLOAT(FAKE_IMU_Z_F0, 0.0f);

/**
 * Fake IMU Z axis stop frequency
 *
 * Stop frequency for Z axis chirp signal
 *
 * @unit Hz
 * @min 0
 * @max 4000
 * @decimal 1
 * @group Fake IMU
 */
PARAM_DEFINE_FLOAT(FAKE_IMU_Z_F1, 1000.0f);

/**
 * Fake IMU sweep period
 *
 * Time period for one complete frequency sweep
 *
 * @unit s
 * @min 1
 * @max 60
 * @decimal 1
 * @group Fake IMU
 */
PARAM_DEFINE_FLOAT(FAKE_IMU_PERIOD, 10.0f);

