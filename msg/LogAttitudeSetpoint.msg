# Compact attitude setpoint for ERmao logging mode
# Optimized for outer loop control analysis
# Original: vehicle_attitude_setpoint (56 bytes) → LogAttitudeSetpoint (20 bytes) = -64% size reduction

uint64 timestamp          # time since system start (microseconds)

# Euler angle setpoints in rad (body frame)
float32 roll_body         # Roll angle setpoint (rad)
float32 pitch_body        # Pitch angle setpoint (rad)
float32 yaw_body          # Yaw angle setpoint (rad)

# Removed fields (not critical for control analysis):
# - yaw_sp_move_rate: can be calculated from yaw_body differences
# - q_d[4]: quaternion setpoint, redundant with Euler angles
# - thrust_body[3]: not needed for attitude tracking analysis

# Queue depth increased for full-rate logging
uint8 ORB_QUEUE_LENGTH = 16  # vs 8 in original vehicle_attitude_setpoint
