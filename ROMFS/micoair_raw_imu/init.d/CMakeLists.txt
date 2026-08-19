############################################################################
#
# Minimal startup files for the MicoAir H743 dual raw-IMU data logger.
#
############################################################################

px4_add_romfs_files(
	rc.fifo_logger
	rcS
)
