# Compact attitude data for ERmao logging mode
# Optimized for outer loop control analysis
# Original: vehicle_attitude (65 bytes) → LogAttitude (28 bytes) = -57% size reduction

uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample   # timestamp of the raw data (microseconds)

# Euler angles in rad (FRD body frame to NED earth frame)
float32 roll              # Roll angle (rad)
float32 pitch             # Pitch angle (rad)
float32 yaw               # Yaw angle (rad)

# Removed fields (not critical for control analysis):
# - q[4]: quaternion, can be reconstructed from Euler angles if needed
# - delta_q_reset[4]: only needed for estimator switching
# - quat_reset_counter: only needed for estimator switching

# Queue depth increased for full-rate logging
uint8 ORB_QUEUE_LENGTH = 16  # vs 8 in original vehicle_attitude
