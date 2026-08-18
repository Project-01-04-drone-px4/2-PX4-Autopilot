/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <px4_arch/micro_hal.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>

#include <board_config.h>

#include <cstdint>

namespace
{
constexpr uint32_t kOutputHigh = GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_SET;

struct TestLine {
	uint32_t gpio;
	const char *name;
};

constexpr TestLine kTestLines[] {
	{kOutputHigh | GPIO_PORTA | GPIO_PIN5,  "PA5/SPI1_SCK"},
	{kOutputHigh | GPIO_PORTA | GPIO_PIN6,  "PA6/SPI1_MISO"},
	{kOutputHigh | GPIO_PORTA | GPIO_PIN7,  "PA7/SPI1_MOSI"},
	{kOutputHigh | GPIO_PORTD | GPIO_PIN14, "PD14/M9/IMU1_CS"},
	{kOutputHigh | GPIO_PORTD | GPIO_PIN15, "PD15/M10/IMU2_CS"},
};

constexpr unsigned kAllHighTimeUs = 100000;
constexpr unsigned kLowTimeUs = 500000;
}

class GPIOLineTest : public ModuleBase<GPIOLineTest>
{
public:
	static int task_spawn(int argc, char *argv[]);
	static GPIOLineTest *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	void run() override;
	int print_status() override;

private:
	void set_all(bool high);

	volatile unsigned _active_line{0};
};

void GPIOLineTest::set_all(bool high)
{
	for (const TestLine &line : kTestLines) {
		px4_arch_gpiowrite(line.gpio, high);
	}
}

void GPIOLineTest::run()
{
	for (const TestLine &line : kTestLines) {
		px4_arch_configgpio(line.gpio);
	}

	set_all(true);
	PX4_INFO("started: 100 ms all-high, then each line low for 500 ms");

	while (!should_exit()) {
		for (unsigned i = 0; i < sizeof(kTestLines) / sizeof(kTestLines[0]) && !should_exit(); ++i) {
			set_all(true);
			px4_usleep(kAllHighTimeUs);

			_active_line = i;
			px4_arch_gpiowrite(kTestLines[i].gpio, false);
			px4_usleep(kLowTimeUs);
		}
	}

	set_all(true);
}

int GPIOLineTest::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("gpio_line_test",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      PX4_STACK_ADJUSTED(1400),
				      run_trampoline,
				      reinterpret_cast<char *const *>(argv));

	return _task_id < 0 ? PX4_ERROR : PX4_OK;
}

GPIOLineTest *GPIOLineTest::instantiate(int argc, char *argv[])
{
	return new GPIOLineTest();
}

int GPIOLineTest::print_status()
{
	PX4_INFO("running, active low: %s", kTestLines[_active_line].name);
	return PX4_OK;
}

int GPIOLineTest::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GPIOLineTest::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION("Cycle SPI1 SCK/MISO/MOSI and the M9/M10 chip-select pins for wiring tests.");
	PRINT_MODULE_USAGE_NAME("gpio_line_test", "command");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return PX4_OK;
}

extern "C" __EXPORT int gpio_line_test_main(int argc, char *argv[])
{
	return GPIOLineTest::main(argc, argv);
}
