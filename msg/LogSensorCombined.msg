# Compact sensor fusion data for ERmao logging mode
# Optimized for EKF input analysis
# Original: sensor_combined (52 bytes) → LogSensorCombined (32 bytes) = -38% size reduction

uint64 timestamp          # time since system start (microseconds)

# Calibrated sensor readings
float32[3] gyro_rad       # average angular rate in rad/s (FRD body frame)
float32[3] accelerometer_m_s2  # average acceleration in m/s² (FRD body frame)

# Removed fields (not critical for signal chain analysis):
# - gyro_integral_dt, accelerometer_integral_dt: can be derived
# - accelerometer_clipping, gyro_clipping: not critical for normal operation
# - accel_calibration_count, gyro_calibration_count: not needed

# Queue depth increased for full-rate logging
uint8 ORB_QUEUE_LENGTH = 32  # vs 8 in original sensor_combined
