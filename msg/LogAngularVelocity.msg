# Compact angular velocity data for ERmao logging mode
# Optimized for inner loop control analysis
# Original: vehicle_angular_velocity (40 bytes) → LogAngularVelocity (28 bytes) = -30% size reduction

uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample   # timestamp of the data sample (microseconds)

# Bias-corrected angular velocity in FRD body frame (rad/s)
float32[3] xyz            # [roll_rate, pitch_rate, yaw_rate] in rad/s

# Removed fields (can be calculated in post-processing):
# - xyz_derivative: angular acceleration, can be calculated from xyz differences

# Queue depth increased for full-rate logging
uint8 ORB_QUEUE_LENGTH = 32  # vs 8 in original vehicle_angular_velocity
