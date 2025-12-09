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
 * @file debug_pin.cpp
 *
 * Debug pin management implementation for micoair H743 board
 */

#include "board_config.h"
#include "debug_pin.h"

#ifdef BOARD_ENABLE_DEBUG_PIN

#include <px4_platform_common/px4_config.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#ifdef __cplusplus
extern "C" {
#endif

// GPIO definitions for motor5-10 pins used as debug pins
// Motor 5: PB1 (Timer3 Channel4)
// Motor 6: PB0 (Timer3 Channel3)
// Motor 7: PD12 (Timer4 Channel1)
// Motor 8: PD13 (Timer4 Channel2)
// Motor 9: PD14 (Timer4 Channel3)
// Motor 10: PD15 (Timer4 Channel4)

#define GPIO_DEBUG_PIN_LINE0  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN1)   // PB1
#define GPIO_DEBUG_PIN_LINE1  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN0)   // PB0
#define GPIO_DEBUG_PIN_LINE2  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN12)  // PD12
#define GPIO_DEBUG_PIN_LINE3  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN13)  // PD13
#define GPIO_DEBUG_PIN_LINE4  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN14)  // PD14
#define GPIO_DEBUG_PIN_LINE5  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN15)  // PD15

static const uint32_t debug_pin_gpios[DEBUG_PIN_MAX_LINES] = {
	GPIO_DEBUG_PIN_LINE0,
	GPIO_DEBUG_PIN_LINE1,
	GPIO_DEBUG_PIN_LINE2,
	GPIO_DEBUG_PIN_LINE3,
	GPIO_DEBUG_PIN_LINE4,
	GPIO_DEBUG_PIN_LINE5,
};

static bool debug_pin_initialized = false;

void debug_pin_init(void)
{
	// Avoid duplicate initialization
	if (debug_pin_initialized) {
		return;
	}

	// Configure all debug pins as outputs
	for (int i = 0; i < DEBUG_PIN_MAX_LINES; i++) {
		stm32_configgpio(debug_pin_gpios[i]);
		// Initialize to low
		stm32_gpiowrite(debug_pin_gpios[i], false);
	}

	debug_pin_initialized = true;
}

void debug_pin_set_high(uint8_t line)
{
	if (line < DEBUG_PIN_MAX_LINES) {
		stm32_gpiowrite(debug_pin_gpios[line], true);
	}
}

void debug_pin_set_low(uint8_t line)
{
	if (line < DEBUG_PIN_MAX_LINES) {
		stm32_gpiowrite(debug_pin_gpios[line], false);
	}
}

void debug_pin_toggle(uint8_t line)
{
	if (line < DEBUG_PIN_MAX_LINES) {
		bool current_state = stm32_gpioread(debug_pin_gpios[line]);
		stm32_gpiowrite(debug_pin_gpios[line], !current_state);
	}
}

#ifdef __cplusplus
}
#endif

#endif // BOARD_ENABLE_DEBUG_PIN

