#!/usr/bin/env python3
"""
Move a PX4 SITL vehicle a small local-NED offset using MAVLink Offboard setpoints.
"""

import argparse
import math
import time

from pymavlink import mavutil


PX4_MODE_OFFBOARD = (29, 6, 0)
PX4_MODE_LOITER = (29, 4, 3)

POSITION_ONLY_TYPE_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rx-udp", default=None,
                        help="MAVLink endpoint that receives PX4 telemetry. Defaults to auto: 14550, then 14540.")
    parser.add_argument("--tx-udp", default="udpout:127.0.0.1:18570",
                        help="MAVLink endpoint that sends setpoints to PX4")
    parser.add_argument("--dx", type=float, default=1.0,
                        help="Relative local NED x/north movement in meters")
    parser.add_argument("--dy", type=float, default=0.5,
                        help="Relative local NED y/east movement in meters")
    parser.add_argument("--dz", type=float, default=0.0,
                        help="Relative local NED z/down movement in meters")
    parser.add_argument("--alt", type=float, default=None,
                        help="Target altitude above the local origin in meters. Overrides --dz.")
    parser.add_argument("--duration", type=float, default=8.0,
                        help="Move duration in seconds")
    parser.add_argument("--hold", type=float, default=6.0,
                        help="Hold duration after movement in seconds")
    parser.add_argument("--rate", type=float, default=20.0,
                        help="Setpoint publish rate in Hz")
    parser.add_argument("--timeout", type=float, default=30.0,
                        help="Seconds to wait for LOCAL_POSITION_NED")
    parser.add_argument("--fallback-alt", type=float, default=2.0,
                        help="Assumed altitude in meters if telemetry is unavailable")
    parser.add_argument("--require-telemetry", action="store_true",
                        help="Fail instead of using the fallback position when telemetry is unavailable")
    parser.add_argument("--target-system", type=int, default=1,
                        help="PX4 MAVLink target system id")
    parser.add_argument("--target-component", type=int, default=1,
                        help="PX4 MAVLink target component id")
    parser.add_argument("--no-final-loiter", action="store_true",
                        help="Do not switch to LOITER after the movement")
    return parser.parse_args()


def send_heartbeat(mav):
    mav.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        0,
    )


def send_px4_mode(mav, target_system, target_component, mode_tuple):
    base_mode, custom_mode, custom_sub_mode = mode_tuple
    mav.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        base_mode,
        custom_mode,
        custom_sub_mode,
        0,
        0,
        0,
        0,
    )


def send_position_setpoint(mav, target_system, target_component, x, y, z):
    mav.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        target_system,
        target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        POSITION_ONLY_TYPE_MASK,
        x,
        y,
        z,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def request_local_position_stream(mav, target_system, target_component):
    mav.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        50000,
        0,
        0,
        0,
        0,
        0,
    )


def wait_for_local_position_on_endpoint(rx_endpoint, tx_mav, args, timeout):
    deadline = time.monotonic() + timeout
    next_heartbeat = 0.0
    next_request = 0.0
    last_position = None
    rx_mav = mavutil.mavlink_connection(rx_endpoint, source_system=246, source_component=191)

    print(f"Waiting for LOCAL_POSITION_NED on {rx_endpoint}...")

    while time.monotonic() < deadline:
        now = time.monotonic()

        if now >= next_heartbeat:
            send_heartbeat(tx_mav)
            next_heartbeat = now + 1.0

        if now >= next_request:
            request_local_position_stream(tx_mav, args.target_system, args.target_component)
            next_request = now + 2.0

        msg = rx_mav.recv_match(type=["HEARTBEAT", "LOCAL_POSITION_NED"], blocking=True, timeout=0.2)

        if msg is None:
            continue

        if msg.get_type() == "LOCAL_POSITION_NED":
            last_position = msg
            break

    rx_mav.close()

    return last_position


def wait_for_local_position(tx_mav, args):
    endpoints = [args.rx_udp] if args.rx_udp else [
        "udpin:0.0.0.0:14550",
        "udpin:0.0.0.0:14540",
    ]
    timeout_per_endpoint = max(5.0, args.timeout / len(endpoints))

    for endpoint in endpoints:
        try:
            local_position = wait_for_local_position_on_endpoint(endpoint, tx_mav, args, timeout_per_endpoint)

        except OSError as exc:
            print(f"Could not open {endpoint}: {exc}")
            continue

        if local_position is not None:
            print(f"Telemetry received from {endpoint}.")
            return local_position

    raise TimeoutError(
        "No LOCAL_POSITION_NED received. Start jMAVSim/PX4 first, wait for pxh>, then run this script. "
        "If QGroundControl is using UDP 14550, try: .\\move_sitl_local.cmd --rx-udp udpin:0.0.0.0:14540"
    )


def publish_for_duration(tx_mav, args, start, target, duration, label):
    period = 1.0 / max(args.rate, 1.0)
    deadline = time.monotonic() + max(duration, 0.0)
    begin = time.monotonic()
    next_print = 0.0

    while time.monotonic() < deadline:
        now = time.monotonic()
        elapsed = now - begin
        alpha = 1.0 if duration <= 0.0 else min(1.0, elapsed / duration)

        x = start[0] + (target[0] - start[0]) * alpha
        y = start[1] + (target[1] - start[1]) * alpha
        z = start[2] + (target[2] - start[2]) * alpha

        send_heartbeat(tx_mav)
        send_position_setpoint(tx_mav, args.target_system, args.target_component, x, y, z)

        if now >= next_print:
            print(f"{label}: setpoint x={x:+.2f} y={y:+.2f} z={z:+.2f}")
            next_print = now + 1.0

        time.sleep(period)


def main():
    args = parse_args()

    tx_mav = mavutil.mavlink_connection(args.tx_udp, source_system=246, source_component=191)

    try:
        local_position = wait_for_local_position(tx_mav, args)
        start = [float(local_position.x), float(local_position.y), float(local_position.z)]
        using_fallback = False

    except TimeoutError:
        if args.require_telemetry:
            raise

        fallback_alt = abs(args.alt) if args.alt is not None else abs(args.fallback_alt)
        start = [0.0, 0.0, -fallback_alt]
        using_fallback = True
        print(
            "Warning: LOCAL_POSITION_NED was not received. "
            f"Falling back to assumed local NED x=0.00 y=0.00 z={start[2]:+.2f}."
        )

    target_z = -abs(args.alt) if args.alt is not None else start[2] + args.dz
    target = [start[0] + args.dx, start[1] + args.dy, target_z]

    if not all(math.isfinite(value) for value in start + target):
        raise ValueError(f"Invalid local position/setpoint: start={start}, target={target}")

    print(f"Current local NED: x={start[0]:+.2f} y={start[1]:+.2f} z={start[2]:+.2f}")
    print(f"Target local NED:  x={target[0]:+.2f} y={target[1]:+.2f} z={target[2]:+.2f}")

    if using_fallback:
        print("Fallback mode sends absolute local setpoints. Run after 'commander takeoff' for best results.")

    elif start[2] > -0.5 and args.alt is None:
        print("Warning: vehicle does not look airborne. Run 'commander takeoff' first, or pass --alt 2.0.")

    print("Priming Offboard setpoints...")
    publish_for_duration(tx_mav, args, start, start, 2.5, "prime")

    print("Switching to Offboard...")
    send_px4_mode(tx_mav, args.target_system, args.target_component, PX4_MODE_OFFBOARD)
    publish_for_duration(tx_mav, args, start, target, args.duration, "move")

    print("Holding target...")
    publish_for_duration(tx_mav, args, target, target, args.hold, "hold")

    if not args.no_final_loiter:
        print("Switching to LOITER so you can run 'commander land' from pxh>.")
        send_px4_mode(tx_mav, args.target_system, args.target_component, PX4_MODE_LOITER)

    print("Done.")


if __name__ == "__main__":
    main()
