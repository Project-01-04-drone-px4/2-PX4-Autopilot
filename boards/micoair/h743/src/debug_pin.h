/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file debug_pin.h
 *
 * Debug pin management for micoair H743 board
 * Uses motor5-10 pins (PB1, PB0, PD12, PD13, PD14, PD15) as debug pins
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <stm32_gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef BOARD_ENABLE_DEBUG_PIN

/**
 * Debug pin line definitions
 * Line0-5 correspond to motor5-10 pins
 */
#define DEBUG_PIN_LINE0  0  // Motor 5: PB1
#define DEBUG_PIN_LINE1  1  // Motor 6: PB0
#define DEBUG_PIN_LINE2  2  // Motor 7: PD12
#define DEBUG_PIN_LINE3  3  // Motor 8: PD13
#define DEBUG_PIN_LINE4  4  // Motor 9: PD14
#define DEBUG_PIN_LINE5  5  // Motor 10: PD15

#define DEBUG_PIN_MAX_LINES 6

/**
 * Initialize debug pins
 * Must be called before using any debug pin functions
 */
void debug_pin_init(void);

/**
 * Set debug pin to high
 * @param line Debug pin line (0-5)
 */
void debug_pin_set_high(uint8_t line);

/**
 * Set debug pin to low
 * @param line Debug pin line (0-5)
 */
void debug_pin_set_low(uint8_t line);

/**
 * Toggle debug pin
 * @param line Debug pin line (0-5)
 */
void debug_pin_toggle(uint8_t line);

#else // BOARD_ENABLE_DEBUG_PIN

// Empty macros when debug pin is disabled
#define debug_pin_init()
#define debug_pin_set_high(line) ((void)(line))
#define debug_pin_set_low(line) ((void)(line))
#define debug_pin_toggle(line) ((void)(line))

#endif // BOARD_ENABLE_DEBUG_PIN

#ifdef __cplusplus
}
#endif

