# Compact angular rate setpoint for ERmao logging mode
# Optimized for inner loop control analysis
# Original: vehicle_rates_setpoint (37 bytes) → LogRatesSetpoint (20 bytes) = -46% size reduction

uint64 timestamp          # time since system start (microseconds)

# Angular rate setpoints in FRD body frame (rad/s)
float32 roll              # Roll rate setpoint (rad/s)
float32 pitch             # Pitch rate setpoint (rad/s)
float32 yaw               # Yaw rate setpoint (rad/s)

# Removed fields (not critical for rate tracking analysis):
# - thrust_body[3]: not needed for rate tracking analysis
# - reset_integral: only needed for real-time control

# Queue depth increased for full-rate logging
uint8 ORB_QUEUE_LENGTH = 16  # vs 8 in original vehicle_rates_setpoint
