#!/usr/bin/env python3
"""Publish the sealed readonly snapshot; this script has one ROS output only."""
from __future__ import annotations

import argparse
import json
import sys

import rospy
from std_msgs.msg import String

from raicom_bringup.readonly_runtime import (
    ReadonlyRuntimeError,
    create_readonly_runtime,
    load_runtime_config,
    sealed_snapshot_to_mission_display,
)


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RAICOM sealed readonly runtime")
    parser.add_argument("--config", required=True)
    parser.add_argument("--artifact-root", required=True)
    return parser.parse_args()


def main() -> int:
    args = _arguments()
    try:
        config = load_runtime_config(args.config)
        runtime = create_readonly_runtime(args.artifact_root, config)
        snapshot = json.dumps(sealed_snapshot_to_mission_display(runtime.snapshot_json()), sort_keys=True, separators=(",", ":"))
    except (ReadonlyRuntimeError, TypeError, ValueError) as error:
        rospy.logerr("SIM development readonly runtime failed closed: %s", error)
        return 2
    rospy.init_node("raicom_sim_readonly_runtime", anonymous=False, disable_signals=False)
    publisher = rospy.Publisher("/raicom/mission/snapshot", String, queue_size=1, latch=True)
    rate = rospy.Rate(config.snapshot_rate_hz)
    while not rospy.is_shutdown():
        publisher.publish(String(data=snapshot))
        rate.sleep()
    return 0


if __name__ == "__main__":
    sys.exit(main())
