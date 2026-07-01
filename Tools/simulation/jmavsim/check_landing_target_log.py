#!/usr/bin/env python3
"""
Check a PX4 ULog for landing_target_estimator learning topics.
"""

import argparse
import sys
from pathlib import Path

try:
    from pyulog import ULog
except ImportError as exc:
    raise SystemExit(
        "pyulog is not installed for this Python. Install it with: python -m pip install pyulog"
    ) from exc


REQUIRED_TOPICS = [
    "irlock_report",
    "landing_target_pose",
    "landing_target_innovations",
]

SUPPORT_TOPICS = [
    "vehicle_local_position",
    "vehicle_attitude",
    "vehicle_acceleration",
]

DIAGNOSTIC_PARAMETERS = [
    "SYS_AUTOSTART",
    "VTE_EN",
    "LTEST_MODE",
    "LTEST_SIM_EN",
    "LTEST_SIM_X",
    "LTEST_SIM_Y",
    "LTEST_Z_FALLBACK",
]

FIELD_SUMMARY = {
    "irlock_report": ["pos_x", "pos_y", "size_x", "size_y"],
    "landing_target_pose": [
        "x_rel",
        "y_rel",
        "z_rel",
        "vx_rel",
        "vy_rel",
        "cov_x_rel",
        "cov_y_rel",
        "cov_vx_rel",
        "cov_vy_rel",
    ],
    "landing_target_innovations": ["innov_x", "innov_y", "innov_cov_x", "innov_cov_y"],
    "vehicle_local_position": ["x", "y", "z", "dist_bottom", "dist_bottom_valid"],
}


def repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def parse_args() -> argparse.Namespace:
    default_log_dir = repo_root() / "build" / "px4_sitl_cygwin39" / "rootfs" / "log"

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "log",
        nargs="?",
        type=Path,
        help="Specific .ulg file to check. If omitted, the newest .ulg is used.",
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=default_log_dir,
        help="Directory to search recursively for .ulg files when no log file is specified.",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List all topics in the selected log.",
    )
    return parser.parse_args()


def newest_log(log_dir: Path) -> Path:
    logs = sorted(log_dir.rglob("*.ulg"), key=lambda path: path.stat().st_mtime, reverse=True)

    if not logs:
        raise FileNotFoundError(f"No .ulg files found under {log_dir}")

    return logs[0]


def sample_count(dataset) -> int:
    if not dataset.data:
        return 0

    return len(next(iter(dataset.data.values())))


def print_field_summary(name: str, dataset) -> None:
    fields = FIELD_SUMMARY.get(name, [])

    for field in fields:
        data = dataset.data.get(field)

        if data is None or len(data) == 0:
            continue

        try:
            first = data[0]
            last = data[-1]
            print(f"    {field}: first={first} last={last}")

        except Exception:
            print(f"    {field}: samples={len(data)}")


def main() -> int:
    args = parse_args()
    log_path = args.log if args.log else newest_log(args.log_dir)
    log_path = log_path.resolve()

    if not log_path.exists():
        print(f"ERROR: log file does not exist: {log_path}", file=sys.stderr)
        return 1

    if log_path.suffix.lower() != ".ulg":
        print(f"ERROR: expected a .ulg file: {log_path}", file=sys.stderr)
        return 1

    ulog = ULog(str(log_path))
    datasets = {dataset.name: dataset for dataset in ulog.data_list}
    topic_names = sorted(datasets)
    missing_required = [topic for topic in REQUIRED_TOPICS if topic not in datasets]

    print(f"Log: {log_path}")
    print(f"Size: {log_path.stat().st_size / (1024 * 1024):.2f} MiB")
    print(f"Topics: {len(topic_names)}")
    print()

    print("Landing target estimator topics:")

    for topic in REQUIRED_TOPICS:
        dataset = datasets.get(topic)

        if dataset is None:
            print(f"  [NO ] {topic}")

        else:
            print(f"  [YES] {topic}: samples={sample_count(dataset)}")
            print_field_summary(topic, dataset)

    print()
    print("Support topics:")

    for topic in SUPPORT_TOPICS:
        dataset = datasets.get(topic)

        if dataset is None:
            print(f"  [NO ] {topic}")

        else:
            print(f"  [YES] {topic}: samples={sample_count(dataset)}")
            print_field_summary(topic, dataset)

    landing_like = [name for name in topic_names if "landing" in name or "irlock" in name]

    print()
    print("Landing/IRLOCK-like topics:")
    print("  " + (", ".join(landing_like) if landing_like else "(none)"))

    print()
    print("Relevant parameters:")

    for name in DIAGNOSTIC_PARAMETERS:
        print(f"  {name}: {ulog.initial_parameters.get(name, '<missing>')}")

    interesting_messages = [
        message for message in ulog.logged_messages
        if any(
            key in message.message.lower()
            for key in ["landing_target", "irlock", "vision target", "lost sight", "marker"]
        )
    ]

    if interesting_messages:
        print()
        print("Relevant log messages:")

        for message in interesting_messages[-10:]:
            print(f"  {message.timestamp}: {message.message}")

    if args.list:
        print()
        print("All topics:")

        for name in topic_names:
            print(f"  {name}")

    print()

    if missing_required:
        print("Result: MISSING landing_target_estimator learning topics")
        print("Missing: " + ", ".join(missing_required))

        if interesting_messages:
            print(
                "Hint: landing_target_estimator produced log messages, so the module likely ran. "
                "Use a log from a run after the logger topics were forced on."
            )

        print("Action: rebuild/restart SITL, fly again, then run this checker on the new log.")
        return 0

    print("Result: OK - this log contains the landing_target_estimator learning topics.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
