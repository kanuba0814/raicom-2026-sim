#!/usr/bin/env python3
"""Verify the SIM pre-dock pose before AR fine docking; never publishes motion."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import rospy
import tf
from tf.transformations import euler_from_quaternion
import yaml

from raicom_charging_sim.docking import DockProfile, coarse_navigation_error


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", required=True)
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    profile = DockProfile.from_mapping(
        yaml.safe_load(Path(args.profile).read_text(encoding="utf-8"))
    )
    rospy.init_node("raicom_sim_charging_coarse_gate", anonymous=False)
    if rospy.get_param("/raicom/backend", "") != "SIM":
        raise SystemExit("REAL_BACKEND_REJECTED")

    listener = tf.TransformListener()
    try:
        listener.waitForTransform(profile.pre_dock_frame, "base_link", rospy.Time(0), rospy.Duration(3.0))
        translation, rotation = listener.lookupTransform(
            profile.pre_dock_frame, "base_link", rospy.Time(0)
        )
        yaw_rad = euler_from_quaternion(rotation)[2]
        xy_error_m, yaw_error_rad, accepted = coarse_navigation_error(
            profile, translation[0], translation[1], yaw_rad
        )
        report = {
            "ok": accepted,
            "simulation_only": True,
            "motion_issued": False,
            "stage": "COARSE_NAVIGATION_ACCURACY",
            "measured_pose": {
                "frame": profile.pre_dock_frame,
                "x_m": translation[0],
                "y_m": translation[1],
                "yaw_rad": yaw_rad,
            },
            "xy_error_m": xy_error_m,
            "yaw_error_rad": yaw_error_rad,
            "xy_tolerance_m": profile.coarse_xy_tolerance_m,
            "yaw_tolerance_rad": profile.coarse_yaw_tolerance_rad,
            "next": "AR_FINE_DOCKING" if accepted else "RVIZ_MANUAL_CALIBRATION_REQUIRED",
        }
    except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
        report = {
            "ok": False,
            "simulation_only": True,
            "motion_issued": False,
            "stage": "COARSE_NAVIGATION_ACCURACY",
            "reason": "COARSE_POSE_UNAVAILABLE: {}".format(exc),
            "next": "RVIZ_MANUAL_CALIBRATION_REQUIRED",
        }
    print(json.dumps(report, sort_keys=True), flush=True)
    raise SystemExit(0 if report["ok"] else 2)


if __name__ == "__main__":
    main()
