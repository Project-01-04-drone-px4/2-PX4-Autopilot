#!/usr/bin/env python3
"""
Publish synthetic MAVLink LANDING_TARGET observations for SITL learning.

PX4 converts LANDING_TARGET messages with position_valid=false into
irlock_report uORB messages. The landing_target_estimator then consumes those
reports and publishes landing_target_pose and landing_target_innovations.
"""

import argparse
import math
import random
import time

from pymavlink import mavutil


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rx-udp", default="udpin:0.0.0.0:14540",
                        help="MAVLink endpoint that receives PX4 telemetry")
    parser.add_argument("--tx-udp", default="udpout:127.0.0.1:14580",
                        help="MAVLink endpoint that sends observations to PX4")
    parser.add_argument("--rate", type=float, default=20.0,
                        help="LANDING_TARGET publish rate in Hz")
    parser.add_argument("--target-x", type=float, default=0.0,
                        help="Target local NED x/north position in meters")
    parser.add_argument("--target-y", type=float, default=0.0,
                        help="Target local NED y/east position in meters")
    parser.add_argument("--angle-x", type=float, default=0.0,
                        help="Fallback body-frame x angle when telemetry is unavailable")
    parser.add_argument("--angle-y", type=float, default=0.0,
                        help="Fallback body-frame y angle when telemetry is unavailable")
    parser.add_argument("--noise", type=float, default=0.002,
                        help="Gaussian angular noise in tan(angle) units")
    parser.add_argument("--size", type=float, default=0.05,
                        help="Synthetic target angular size")
    return parser.parse_args()


def main():
    args = parse_args()
    rx_mav = mavutil.mavlink_connection(args.rx_udp, source_system=245, source_component=190)
    tx_mav = mavutil.mavlink_connection(args.tx_udp, source_system=245, source_component=190)

    print(
        f"Publishing LANDING_TARGET to {args.tx_udp}; reading telemetry from {args.rx_udp}.",
        flush=True,
    )

    period = 1.0 / max(args.rate, 0.1)
    next_send = time.monotonic()
    next_heartbeat = 0.0
    local_position = None
    attitude = None
    last_print = 0.0

    while True:
        msg = rx_mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE"], blocking=False)

        if msg is not None and msg.get_type() == "LOCAL_POSITION_NED":
            local_position = msg

        elif msg is not None and msg.get_type() == "ATTITUDE":
            attitude = msg

        now = time.monotonic()

        if now >= next_heartbeat:
            tx_mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                0,
            )
            next_heartbeat = now + 1.0

        if now >= next_send:
            body_x = math.nan
            body_y = math.nan
            body_z = math.nan
            rel_z_down = 1.0
            angle_x = args.angle_x + random.gauss(0.0, args.noise)
            angle_y = args.angle_y + random.gauss(0.0, args.noise)

            # NED: z is negative above the origin. A ground target at z=0 has
            # positive down distance relative to a flying vehicle.
            if local_position is not None and attitude is not None:
                rel_n = args.target_x - float(local_position.x)
                rel_e = args.target_y - float(local_position.y)
                rel_d = -float(local_position.z)

                # Rotate local NED target vector into body FRD using ATTITUDE.
                roll = float(attitude.roll)
                pitch = float(attitude.pitch)
                yaw = float(attitude.yaw)
                cr = math.cos(roll)
                sr = math.sin(roll)
                cp = math.cos(pitch)
                sp = math.sin(pitch)
                cy = math.cos(yaw)
                sy = math.sin(yaw)

                body_x = cp * cy * rel_n + cp * sy * rel_e - sp * rel_d
                body_y = (sr * sp * cy - cr * sy) * rel_n + (sr * sp * sy + cr * cy) * rel_e + sr * cp * rel_d
                body_z = (cr * sp * cy + sr * sy) * rel_n + (cr * sp * sy - sr * cy) * rel_e + cr * cp * rel_d

                rel_z_down = max(0.5, body_z)
                angle_x = body_x / rel_z_down + random.gauss(0.0, args.noise)
                angle_y = body_y / rel_z_down + random.gauss(0.0, args.noise)

            time_usec = int(time.time() * 1_000_000)
            tx_mav.mav.landing_target_send(
                time_usec,
                0,
                mavutil.mavlink.MAV_FRAME_BODY_FRD,
                angle_x,
                angle_y,
                rel_z_down,
                args.size,
                args.size,
            )

            if now - last_print > 1.0:
                print(
                    "LANDING_TARGET "
                    f"angle_x={angle_x:+.4f} angle_y={angle_y:+.4f} "
                    f"body=({body_x:+.2f}, {body_y:+.2f}, {body_z:+.2f})",
                    flush=True,
                )
                last_print = now

            next_send += period

        time.sleep(0.005)


if __name__ == "__main__":
    main()
