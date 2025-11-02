# Compact motor output for ERmao logging mode
# Optimized for quadcopter (4 motors) control output analysis
# Original: actuator_motors (81 bytes) → LogActuatorMotors (32 bytes) = -60% size reduction

uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample   # timestamp of the data sample (microseconds)

# Normalized motor control values for quadcopter
# Range: [-1, 1] where 1 = full throttle, -1 = full reverse (if supported)
float32[4] control        # Motor outputs [motor1, motor2, motor3, motor4]

# Removed fields (not critical for signal chain analysis):
# - reversible_flags: most multicopters don't use reversible motors
# - Additional 12 motor slots (only 4 needed for quadcopter)

# Queue depth increased for full-rate logging
uint8 ORB_QUEUE_LENGTH = 16  # vs 8 in original actuator_motors
