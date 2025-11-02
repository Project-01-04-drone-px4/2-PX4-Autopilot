# Compact integrated IMU data for ERmao logging mode
# Optimized for attitude estimation input analysis
# Original: vehicle_imu (60 bytes) → LogVehicleImu (40 bytes) = -33% size reduction

uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample   # timestamp of the data sample (microseconds)

# Integrated delta angles (rad) and delta velocities (m/s)
float32[3] delta_angle    # gyro integral dt [x, y, z] (rad)
float32[3] delta_velocity # accel integral dt [x, y, z] (m/s)

# Removed fields (not critical for signal chain analysis):
# - accel_device_id, gyro_device_id: not needed for single IMU
# - delta_angle_dt, delta_velocity_dt: can be derived from timestamps
# - clip_counter: not critical for normal operation analysis
# - calibration_count: not needed for post-processing

# Queue depth increased for full-rate logging
uint8 ORB_QUEUE_LENGTH = 16  # vs 8 in original vehicle_imu
