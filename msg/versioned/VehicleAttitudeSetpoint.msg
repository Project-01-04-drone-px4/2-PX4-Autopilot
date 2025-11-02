uint32 MESSAGE_VERSION = 2

uint64 timestamp		# time since system start (microseconds)

float32 yaw_sp_move_rate	# rad/s (commanded by user)

# For quaternion-based attitude control
float32[4] q_d			# Desired quaternion for quaternion control

# Euler angles (converted from quaternion for convenience in logging and analysis)
float32 roll_body		# Roll angle setpoint (rad)
float32 pitch_body		# Pitch angle setpoint (rad)
float32 yaw_body		# Yaw angle setpoint (rad)

# For clarification: For multicopters thrust_body[0] and thrust[1] are usually 0 and thrust[2] is the negative throttle demand.
# For fixed wings thrust_x is the throttle demand and thrust_y, thrust_z will usually be zero.
float32[3] thrust_body		# Normalized thrust command in body FRD frame [-1,1]

# TOPICS vehicle_attitude_setpoint mc_virtual_attitude_setpoint fw_virtual_attitude_setpoint
