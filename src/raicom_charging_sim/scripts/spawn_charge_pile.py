#!/usr/bin/env python3
"""Spawn the team-owned static SIM wrapper and return observed Gazebo pose."""
from __future__ import annotations
import json
import math
from pathlib import Path
import sys

import rospy
import yaml
from gazebo_msgs.srv import GetModelProperties, GetModelState, SpawnModel
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def main() -> None:
    arguments = rospy.myargv(argv=sys.argv)[1:]
    if len(arguments) != 1:
        raise SystemExit("one fixed profile path is required")
    profile_path = Path(arguments[0])
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    if profile.get("metadata", {}).get("scope") != "SIM_ONLY":
        raise SystemExit("SIM_ONLY_PROFILE_REQUIRED")
    dock = profile["dock"]
    model_sdf = (Path(__file__).resolve().parents[1] / "models" / "raicom_sim_charge_pile" / "model.sdf").read_text(encoding="utf-8")
    if "<static>true</static>" not in model_sdf or "libChargeContact" in model_sdf:
        raise SystemExit("STATIC_OVERLAY_CONTRACT_INVALID")

    rospy.init_node("raicom_sim_charge_pile_spawner", anonymous=True)
    if rospy.get_param("/raicom/backend", "") != "SIM":
        print(json.dumps({"ok": False, "reason": "REAL_BACKEND_REJECTED"}))
        raise SystemExit(2)
    rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=8)
    rospy.wait_for_service("/gazebo/get_model_state", timeout=8)
    rospy.wait_for_service("/gazebo/get_model_properties", timeout=8)
    pose = Pose()
    expected = dock["pose"]
    pose.position.x, pose.position.y = float(expected["x_m"]), float(expected["y_m"])
    quaternion = quaternion_from_euler(0, 0, float(expected["yaw_rad"]))
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion
    model = dock["model_name"]
    spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    response = spawn(model, model_sdf, "", pose, "world")
    if not response.success and "exist" not in response.status_message.lower():
        print(json.dumps({"ok": False, "reason": "SPAWN_FAILED", "detail": response.status_message}))
        raise SystemExit(2)
    state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)(model, "world")
    properties = rospy.ServiceProxy("/gazebo/get_model_properties", GetModelProperties)(model)
    observed = state.pose
    yaw = euler_from_quaternion((observed.orientation.x, observed.orientation.y,
                                 observed.orientation.z, observed.orientation.w))[2]
    pose_matches = (state.success and properties.success and properties.is_static and
                    "charging_contact_pad" in properties.body_names and
                    math.hypot(observed.position.x - pose.position.x,
                                                observed.position.y - pose.position.y) <= 0.005 and
                    abs(math.atan2(math.sin(yaw - float(expected["yaw_rad"])),
                                   math.cos(yaw - float(expected["yaw_rad"])))) <= math.radians(0.5))
    report = {
        "ok": bool(pose_matches), "simulation_only": True,
        "asset": "small_marker_charge_pile_visual_in_static_team_wrapper",
        "model_name": model, "static": bool(properties.is_static),
        "contact_pad_present": "charging_contact_pad" in properties.body_names,
        "assumption_status": "SIM_WORKING_ASSUMPTION",
        "gazebo_pose_evidence": {"frame": "world", "x_m": observed.position.x,
                                 "y_m": observed.position.y, "z_m": observed.position.z,
                                 "yaw_rad": yaw},
        "pose_matches_profile": bool(pose_matches),
    }
    print(json.dumps(report, sort_keys=True))
    raise SystemExit(0 if report["ok"] else 2)


if __name__ == "__main__":
    main()
