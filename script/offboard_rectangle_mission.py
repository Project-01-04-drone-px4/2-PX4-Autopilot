#!/usr/bin/env python3
"""
Offboard Control Script for PX4
Controls the drone to:
1. Enter offboard mode
2. Take off to 1.5m
3. Fly a 2m x 2m rectangle trajectory
4. Land

Usage:
    python3 offboard_rectangle_mission.py

Requirements:
    pip install pymavlink
"""

import time
import math
from pymavlink import mavutil

# Connection settings
CONNECTION_STRING = 'udp:127.0.0.1:14540'  # Default PX4 SITL connection
CONNECTION_TIMEOUT = 30  # seconds

# Mission parameters
TAKEOFF_ALTITUDE = 1.5  # meters (NED frame, negative is up)
RECTANGLE_SIZE = 2.0  # meters
POSITION_TOLERANCE = 0.3  # meters
SETPOINT_RATE = 20  # Hz (must be > 2Hz for offboard)

# Rectangle waypoints (relative to start position, in NED frame)
# NED: North=X, East=Y, Down=Z (positive Z is down, negative Z is up)
RECTANGLE_WAYPOINTS = [
    (0.0, 0.0, -TAKEOFF_ALTITUDE),      # Start position (hover at 1.5m)
    (RECTANGLE_SIZE, 0.0, -TAKEOFF_ALTITUDE),  # North 2m
    (RECTANGLE_SIZE, RECTANGLE_SIZE, -TAKEOFF_ALTITUDE),  # North-East corner
    (0.0, RECTANGLE_SIZE, -TAKEOFF_ALTITUDE),  # East 2m
    (0.0, 0.0, -TAKEOFF_ALTITUDE),      # Return to start
    (0.0, 0.0, 0.0),                     # Land (z=0 is ground level)
]


class OffboardController:
    def __init__(self, connection_string=CONNECTION_STRING):
        """Initialize MAVLink connection"""
        print(f"Connecting to PX4 at {connection_string}...")
        self.master = mavutil.mavlink_connection(connection_string)

        # Wait for heartbeat
        print("Waiting for heartbeat...")
        self.master.wait_heartbeat(timeout=CONNECTION_TIMEOUT)
        print(f"Heartbeat received! System: {self.master.target_system}, Component: {self.master.target_component}")

        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.armed = False
        self.mode = None
        self.current_setpoint = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        self.offboard_active = False

    def wait_for_position(self):
        """Wait for local position estimate"""
        print("Waiting for local position estimate...")
        while True:
            msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=5)
            if msg:
                self.current_position['x'] = msg.x
                self.current_position['y'] = msg.y
                self.current_position['z'] = msg.z
                print(f"Position received: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}")
                return True
            time.sleep(0.1)

    def get_position(self):
        """Get current position"""
        msg = self.master.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if msg:
            self.current_position['x'] = msg.x
            self.current_position['y'] = msg.y
            self.current_position['z'] = msg.z
        return self.current_position.copy()

    def get_state(self):
        """Get current vehicle state"""
        msg = self.master.recv_match(type='HEARTBEAT', blocking=False)
        if msg:
            self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            # Decode PX4 custom mode
            # PX4 custom_mode is a 32-bit integer with structure:
            # - bits 0-7: sub_mode
            # - bits 8-15: reserved
            # - bits 16-23: main_mode
            # - bits 24-31: reserved_hl
            main_mode = (msg.custom_mode >> 16) & 0xFF

            # PX4 main mode definitions
            PX4_CUSTOM_MAIN_MODE_MANUAL = 1
            PX4_CUSTOM_MAIN_MODE_ALTCTL = 2
            PX4_CUSTOM_MAIN_MODE_POSCTL = 3
            PX4_CUSTOM_MAIN_MODE_AUTO = 4
            PX4_CUSTOM_MAIN_MODE_ACRO = 5
            PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
            PX4_CUSTOM_MAIN_MODE_STABILIZED = 7

            if main_mode == PX4_CUSTOM_MAIN_MODE_MANUAL:
                self.mode = "MANUAL"
            elif main_mode == PX4_CUSTOM_MAIN_MODE_ALTCTL:
                self.mode = "ALTCTL"
            elif main_mode == PX4_CUSTOM_MAIN_MODE_POSCTL:
                self.mode = "POSCTL"
            elif main_mode == PX4_CUSTOM_MAIN_MODE_AUTO:
                self.mode = "AUTO"
            elif main_mode == PX4_CUSTOM_MAIN_MODE_ACRO:
                self.mode = "ACRO"
            elif main_mode == PX4_CUSTOM_MAIN_MODE_OFFBOARD:
                self.mode = "OFFBOARD"
            elif main_mode == PX4_CUSTOM_MAIN_MODE_STABILIZED:
                self.mode = "STABILIZED"
            else:
                self.mode = f"UNKNOWN(main={main_mode}, custom={msg.custom_mode})"
        return self.armed, self.mode

    def send_position_setpoint(self, x, y, z, yaw=0.0):
        """Send position setpoint (NED frame)"""
        # Update current setpoint
        self.current_setpoint = {'x': x, 'y': y, 'z': z, 'yaw': yaw}

        # MAV_FRAME_LOCAL_NED = 1
        # type_mask: bit 1 = x, bit 2 = y, bit 3 = z, bit 4 = vx, bit 5 = vy, bit 6 = vz
        # bit 7 = ax, bit 8 = ay, bit 9 = az, bit 10 = yaw, bit 11 = yaw_rate
        # We want to control position (x, y, z) and yaw, so:
        # Ignore velocity: bit 4, 5, 6 = 1
        # Ignore acceleration: bit 7, 8, 9 = 1
        # Ignore yaw_rate: bit 11 = 1
        # Use position: bit 1, 2, 3 = 0
        # Use yaw: bit 10 = 0
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        self.master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            type_mask,
            x, y, z,  # position (NED)
            0, 0, 0,  # velocity (NED, ignored)
            0, 0, 0,  # acceleration (NED, ignored)
            yaw, 0  # yaw, yaw_rate
        )

    def set_mode_offboard(self):
        """Set vehicle to OFFBOARD mode"""
        print("Setting mode to OFFBOARD...")

        # Get current position for setpoint
        pos = self.get_position()
        current_x, current_y, current_z = pos['x'], pos['y'], pos['z']

        # First, send setpoints for a few seconds to enable offboard mode
        print("Sending initial setpoints (required before entering offboard)...")
        start_time = time.time()
        while time.time() - start_time < 2.0:
            self.send_position_setpoint(current_x, current_y, current_z)
            time.sleep(1.0 / SETPOINT_RATE)

        # Now request offboard mode
        # PX4 custom_mode structure: main_mode (bits 16-23) | sub_mode (bits 0-7)
        # PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6, sub_mode = 0
        PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
        PX4_CUSTOM_SUB_MODE_OFFBOARD = 0
        custom_mode = (PX4_CUSTOM_MAIN_MODE_OFFBOARD << 16) | PX4_CUSTOM_SUB_MODE_OFFBOARD

        # Base mode flags for offboard: CUSTOM_MODE_ENABLED | AUTO_ENABLED | STABILIZE_ENABLED | GUIDED_ENABLED
        base_mode = (
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
            mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED |
            mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED |
            mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED
        )

        print(f"Requesting OFFBOARD mode (custom_mode={custom_mode}, base_mode={base_mode})...")
        self.master.mav.set_mode_send(
            self.master.target_system,
            base_mode,
            custom_mode
        )

        # Wait for mode change - CRITICAL: Must continue sending setpoints!
        print("Waiting for OFFBOARD mode (continuing to send setpoints)...")
        timeout = 10
        start_time = time.time()
        last_request_time = time.time()
        last_print_time = start_time

        while time.time() - start_time < timeout:
            # CONTINUOUSLY send setpoints while waiting for mode change
            # This is required - PX4 needs continuous offboard signal (>2Hz)
            self.send_position_setpoint(current_x, current_y, current_z)

            # Re-request offboard mode every second (in case first request was missed)
            if time.time() - last_request_time > 1.0:
                PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
                PX4_CUSTOM_SUB_MODE_OFFBOARD = 0
                custom_mode = (PX4_CUSTOM_MAIN_MODE_OFFBOARD << 16) | PX4_CUSTOM_SUB_MODE_OFFBOARD
                base_mode = (
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
                    mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED |
                    mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED |
                    mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED
                )
                self.master.mav.set_mode_send(
                    self.master.target_system,
                    base_mode,
                    custom_mode
                )
                last_request_time = time.time()

            armed, mode = self.get_state()
            if mode == "OFFBOARD":
                print("Successfully entered OFFBOARD mode!")
                self.offboard_active = True
                return True

            # Print progress every 2 seconds
            if time.time() - last_print_time >= 2.0:
                elapsed = time.time() - start_time
                print(f"  Waiting... Current mode: {mode}, Elapsed: {elapsed:.1f}s")
                last_print_time = time.time()

            time.sleep(1.0 / SETPOINT_RATE)  # Maintain setpoint rate (>2Hz)

        armed, mode = self.get_state()
        print(f"Failed to enter OFFBOARD mode. Current mode: {mode}")
        print("Make sure setpoints are being sent continuously at >2Hz")
        return False

    def arm(self):
        """Arm the vehicle"""
        print("Arming vehicle...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # arm
            0, 0, 0, 0, 0, 0
        )

        # Wait for arming
        timeout = 10
        start_time = time.time()
        while time.time() - start_time < timeout:
            armed, mode = self.get_state()
            if armed:
                print("Vehicle armed!")
                return True
            time.sleep(0.1)

        print("Failed to arm vehicle")
        return False

    def disarm(self):
        """Disarm the vehicle"""
        print("Disarming vehicle...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,  # disarm
            0, 0, 0, 0, 0, 0
        )

    def wait_for_position_reached(self, target_x, target_y, target_z, timeout=30):
        """Wait until vehicle reaches target position"""
        print(f"Waiting to reach position: x={target_x:.2f}, y={target_y:.2f}, z={target_z:.2f}")
        start_time = time.time()
        distance = float('inf')

        while time.time() - start_time < timeout:
            # Keep sending setpoints while waiting (CRITICAL: must be >2Hz)
            self.send_position_setpoint(target_x, target_y, target_z)

            pos = self.get_position()
            distance = math.sqrt(
                (pos['x'] - target_x)**2 +
                (pos['y'] - target_y)**2 +
                (pos['z'] - target_z)**2
            )

            if distance < POSITION_TOLERANCE:
                print(f"Position reached! Distance: {distance:.2f}m")
                return True

            time.sleep(1.0 / SETPOINT_RATE)

        print(f"Timeout waiting for position. Current distance: {distance:.2f}m")
        return False

    def fly_to_position(self, x, y, z, yaw=0.0, timeout=30):
        """Fly to a specific position"""
        print(f"\nFlying to: x={x:.2f}m, y={y:.2f}m, z={z:.2f}m")

        start_time = time.time()
        distance = float('inf')

        while time.time() - start_time < timeout:
            # Send setpoint continuously (CRITICAL: must be >2Hz to maintain offboard mode)
            self.send_position_setpoint(x, y, z, yaw)

            # Check if reached
            pos = self.get_position()
            distance = math.sqrt(
                (pos['x'] - x)**2 +
                (pos['y'] - y)**2 +
                (pos['z'] - z)**2
            )

            if distance < POSITION_TOLERANCE:
                print(f"Position reached! Distance: {distance:.2f}m")
                return True

            # Check if still in offboard mode
            armed, mode = self.get_state()
            if mode != "OFFBOARD" and self.offboard_active:
                print(f"WARNING: Exited OFFBOARD mode! Current mode: {mode}")
                print("This usually means setpoints stopped being received")
                # Try to re-enter offboard
                if self.set_mode_offboard():
                    continue
                else:
                    return False

            time.sleep(1.0 / SETPOINT_RATE)

        print(f"Timeout flying to position. Distance: {distance:.2f}m")
        return False

    def run_mission(self):
        """Run the complete mission"""
        try:
            # Step 1: Wait for position estimate
            self.wait_for_position()
            initial_pos = self.get_position()
            print(f"Initial position: x={initial_pos['x']:.2f}, y={initial_pos['y']:.2f}, z={initial_pos['z']:.2f}")

            # Step 2: Set mode to OFFBOARD
            if not self.set_mode_offboard():
                print("Failed to enter OFFBOARD mode. Aborting.")
                return False

            # Step 3: Arm vehicle
            if not self.arm():
                print("Failed to arm vehicle. Aborting.")
                return False

            # Step 4: Take off to 1.5m
            print("\n=== TAKEOFF ===")
            takeoff_x = initial_pos['x']
            takeoff_y = initial_pos['y']
            takeoff_z = -TAKEOFF_ALTITUDE  # Negative Z is up in NED

            if not self.fly_to_position(takeoff_x, takeoff_y, takeoff_z, timeout=20):
                print("Takeoff failed!")
                return False

            # Hover for a moment
            print("Hovering at takeoff altitude...")
            hover_start = time.time()
            while time.time() - hover_start < 2.0:
                self.send_position_setpoint(takeoff_x, takeoff_y, takeoff_z)
                time.sleep(1.0 / SETPOINT_RATE)

            # Step 5: Fly rectangle trajectory
            print("\n=== RECTANGLE TRAJECTORY ===")
            for i, (dx, dy, dz) in enumerate(RECTANGLE_WAYPOINTS[1:], 1):  # Skip first waypoint (already there)
                waypoint_x = initial_pos['x'] + dx
                waypoint_y = initial_pos['y'] + dy
                waypoint_z = initial_pos['z'] + dz

                print(f"\nWaypoint {i}/{len(RECTANGLE_WAYPOINTS)-1}")
                if not self.fly_to_position(waypoint_x, waypoint_y, waypoint_z, timeout=30):
                    print(f"Failed to reach waypoint {i}")
                    # Continue anyway

                # Brief hover at waypoint
                hover_start = time.time()
                while time.time() - hover_start < 1.0:
                    self.send_position_setpoint(waypoint_x, waypoint_y, waypoint_z)
                    time.sleep(1.0 / SETPOINT_RATE)

            # Step 6: Land
            print("\n=== LANDING ===")
            land_x = initial_pos['x']
            land_y = initial_pos['y']
            land_z = 0.0  # Ground level

            if not self.fly_to_position(land_x, land_y, land_z, timeout=30):
                print("Landing failed!")

            # Wait for landing
            print("Waiting for landing...")
            landing_start = time.time()
            while time.time() - landing_start < 10.0:
                pos = self.get_position()
                if pos['z'] > -0.2:  # Close to ground
                    print("Landed!")
                    break
                self.send_position_setpoint(land_x, land_y, land_z)
                time.sleep(1.0 / SETPOINT_RATE)

            # Step 7: Disarm
            time.sleep(2)
            self.disarm()

            print("\n=== MISSION COMPLETE ===")
            return True

        except KeyboardInterrupt:
            print("\nMission interrupted by user")
            # Try to land
            try:
                pos = self.get_position()
                self.fly_to_position(pos['x'], pos['y'], 0.0, timeout=10)
                self.disarm()
            except:
                pass
            return False
        except Exception as e:
            print(f"\nError during mission: {e}")
            import traceback
            traceback.print_exc()
            return False


def main():
    """Main function"""
    print("=" * 60)
    print("PX4 Offboard Rectangle Mission")
    print("=" * 60)
    print(f"Takeoff altitude: {TAKEOFF_ALTITUDE}m")
    print(f"Rectangle size: {RECTANGLE_SIZE}m x {RECTANGLE_SIZE}m")
    print("=" * 60)

    controller = OffboardController()
    success = controller.run_mission()

    if success:
        print("\nMission completed successfully!")
    else:
        print("\nMission failed!")

    return 0 if success else 1


if __name__ == "__main__":
    exit(main())

