# Ultra-compact gyroscope FIFO data for ERmao logging mode
# Optimized for signal chain analysis with MINIMAL data size
# Original: sensor_gyro_fifo (224 bytes) → LogGyroFifo (36 bytes) = -84% size reduction!

uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample   # timestamp of first sample (microseconds)

# Raw int16 data (NOT pre-scaled) - use scale factor to convert to rad/s
# Only 2 samples per message (BMI270 @ 1600Hz → 800Hz effective logging rate)
int16[2] x                # X-axis angular velocity (raw)
int16[2] y                # Y-axis angular velocity (raw)
int16[2] z                # Z-axis angular velocity (raw)

float32 scale             # scale factor to convert int16 to rad/s

uint8 samples             # number of valid samples (max 2)

# Removed fields:
# - device_id: not needed for single IMU
# - dt: fixed at 625us (1600Hz / 2)
# - x[32],y[32],z[32]: only need 2 samples

# Queue depth increased for full-rate logging without data loss
uint8 ORB_QUEUE_LENGTH = 32  # vs 4 in original sensor_gyro_fifo
