#!/usr/bin/env python3
"""Manage one fixed SIM-only guarded tangent run; never publishes Twist."""
import argparse
import json
import time

import rospy
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool, Empty, String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout-s", type=float, default=45.0)
    args = parser.parse_args(rospy.myargv()[1:])
    if not 5.0 <= args.timeout_s <= 60.0:
        parser.error("timeout must be within 5..60 seconds")
    rospy.init_node("round_table_managed_run", anonymous=True)
    adapter_ack = rospy.Publisher("/raicom/round_table_docking/ack", Bool, queue_size=1, latch=True)
    safety_ack = rospy.Publisher("/raicom/safety/ack", Empty, queue_size=1)
    soft_estop = rospy.Publisher("/raicom/safety/soft_estop", Bool, queue_size=1, latch=True)
    try:
        if rospy.get_param("/raicom/backend", "") != "SIM":
            raise RuntimeError("REAL_BACKEND_REJECTED")
        rospy.set_param("/round_table_docking_adapter/enabled", False)
        adapter_ack.publish(Bool(True))
        rospy.sleep(0.2)
        safety_ack.publish(Empty())
        rospy.sleep(0.3)
        safety = json.loads(rospy.wait_for_message("/raicom/safety/state", String, timeout=3.0).data)
        if (safety.get("soft_estop_latched") or not safety.get("localization_valid")
                or not safety.get("watchdog_healthy")):
            raise RuntimeError("SAFETY_REQUALIFICATION_FAILED:" + json.dumps(safety, sort_keys=True))
        # Fetch the stable pair only after the slower safety handshake, then
        # enable immediately. The adapter independently rechecks its snapshot.
        observation_deadline = time.monotonic() + 8.0
        while time.monotonic() < observation_deadline:
            rospy.wait_for_message("/round_table/circle", Vector3Stamped, timeout=2.0)
            valid = rospy.wait_for_message("/round_table/valid", Bool, timeout=2.0)
            if valid.data:
                break
        else:
            raise RuntimeError("DETECTOR_NOT_STABLY_VALID")
        started = time.monotonic()
        latest = {}
        samples = 0
        rospy.set_param("/round_table_docking_adapter/enabled", True)
        deadline = started + args.timeout_s
        while time.monotonic() < deadline and not rospy.is_shutdown():
            message = rospy.wait_for_message("/raicom/round_table_docking/status", String, timeout=2.0)
            latest = json.loads(message.data)
            samples += 1
            phase = latest.get("phase")
            if phase == "COMPLETE":
                radial = latest.get("radial_observation", {})
                print(json.dumps({"ok": True, "simulation_only": True, "motion_issued": True,
                                  "measured_front_gap_window_achieved": True,
                                  "exact_3cm_not_established": True,
                                  "attested_front_gap_interval_m": [
                                      radial.get("conservative_front_gap_lower_m"),
                                      radial.get("conservative_front_gap_upper_m")],
                                  "boundary_limited": False, "terminal": latest, "status_samples": samples,
                                  "wall_time_s": round(time.monotonic() - started, 3)}, sort_keys=True))
                return
            if latest.get("state") == "BOUNDARY_LIMIT":
                raise RuntimeError("SAFE_STOP_BOUNDARY_LIMITED:" + json.dumps(latest, sort_keys=True))
            if latest.get("state") == "SOFT_ESTOP" or phase == "STOP":
                raise RuntimeError("ADAPTER_FAIL_CLOSED:" + json.dumps(latest, sort_keys=True))
        raise RuntimeError("ADAPTER_TIMEOUT:" + json.dumps(latest, sort_keys=True))
    finally:
        rospy.set_param("/round_table_docking_adapter/enabled", False)
        # Latched publisher plus bounded connection/send opportunity makes this
        # component safe even when invoked directly rather than via raicomctl.
        deadline = time.monotonic() + 0.5
        while soft_estop.get_num_connections() == 0 and time.monotonic() < deadline:
            rospy.sleep(0.02)
        soft_estop.publish(Bool(True))
        rospy.sleep(0.3)


if __name__ == "__main__":
    main()
