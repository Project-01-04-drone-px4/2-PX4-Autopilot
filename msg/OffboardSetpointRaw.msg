# Raw offboard setpoint received from MAVLink SET_POSITION_TARGET_LOCAL_NED message
# This topic records the original received setpoint data at the rate it was received
# Used for logging and analysis of offboard control commands

uint64 timestamp # time since system start (microseconds)

# Original MAVLink message fields
uint8 coordinate_frame # MAV_FRAME enum value (e.g., MAV_FRAME_LOCAL_NED=1, MAV_FRAME_BODY_NED=8)
uint16 type_mask # Position target type mask (bit flags)

# Position in NED frame (meters)
float32 x # North position
float32 y # East position
float32 z # Down position (positive down, negative up)

# Velocity in NED frame (m/s)
float32 vx # North velocity
float32 vy # East velocity
float32 vz # Down velocity

# Acceleration/Force in NED frame (m/s^2 or N/kg)
float32 afx # North acceleration/force
float32 afy # East acceleration/force
float32 afz # Down acceleration/force

# Yaw and yaw rate
float32 yaw # Yaw angle in radians (-PI..+PI)
float32 yaw_rate # Yaw rate in rad/s

# Target system and component IDs
uint8 target_system # System ID of target
uint8 target_component # Component ID of target


