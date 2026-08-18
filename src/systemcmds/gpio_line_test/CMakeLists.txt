############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
#
############################################################################

px4_add_module(
	MODULE systemcmds__gpio_line_test
	MAIN gpio_line_test
	SRCS
		gpio_line_test.cpp
)
