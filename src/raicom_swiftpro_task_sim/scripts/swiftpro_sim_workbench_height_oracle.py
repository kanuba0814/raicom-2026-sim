#!/usr/bin/env python3
"""Read-only SIM oracle for calibrating camera workbench height and clearance."""
from __future__ import annotations

import argparse
import json
import math

import rospy
from gazebo_msgs.srv import GetLinkState
from raicom_msgs.msg import WorkbenchSlots
from sensor_msgs.msg import JointState

from raicom_swiftpro_task_sim.kinematics import forward_position

# Derived from industrial_group_workbench/meshes/workbench.dae:
# raw mesh Y=[0,220], node transform maps Y to Gazebo Z with scale 0.0011.
WORKBENCH_MESH_TOP_OFFSET_M = 220.0 * 0.0011
WORKBENCH_LINK = "industrial_group::workbench_1::link_1"
END_EFFECTOR_LINK = "oryxbot::virtual_end_effector"
OBSERVATION_JOINTS_RAD = {
    "Joint1": math.radians(-42.0),
    "Joint2": 0.0,
    "Joint6": 0.0,
}
OBSERVATION_TOLERANCE_RAD = 0.03
MAX_SLOT_AGE_S = 0.5


def independent_from_feedback(message):
    positions = dict(zip(message.name, message.position))
    if not all(
        name in positions
        and math.isfinite(positions[name])
        and abs(positions[name] - expected) <= OBSERVATION_TOLERANCE_RAD
        for name, expected in OBSERVATION_JOINTS_RAD.items()
    ):
        return None
    return (
        positions["Joint1"] + math.pi / 2.0,
        math.pi / 2.0 - positions["Joint2"],
        positions["Joint6"] + math.radians(3.8),
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--clearance-m", type=float, default=0.030)
    args = parser.parse_args(rospy.myargv()[1:])
    if not math.isfinite(args.clearance_m) or not 0.01 <= args.clearance_m <= 0.15:
        parser.error("clearance must be within [0.01, 0.15] m")
    rospy.init_node("swiftpro_sim_workbench_height_oracle", anonymous=True)
    if rospy.get_param("/raicom/backend", "") != "SIM":
        print(json.dumps({"status": "REJECTED", "reason": "REAL_BACKEND"}))
        raise SystemExit(2)

    joints = rospy.wait_for_message("/swiftpro/joint_states", JointState, timeout=5.0)
    independent = independent_from_feedback(joints)
    if independent is None:
        print(json.dumps({"status": "REJECTED", "reason": "ARM_NOT_AT_STANDARD_OBSERVATION"}))
        raise SystemExit(2)
    observation_tcp_base = forward_position(independent)
    if observation_tcp_base is None:
        print(json.dumps({"status": "REJECTED", "reason": "OBSERVATION_FK_FAILED"}))
        raise SystemExit(2)

    slots = rospy.wait_for_message("/raicom/workbench/slots", WorkbenchSlots, timeout=5.0)
    now_s = rospy.Time.now().to_sec()
    if (not slots.valid or slots.header.frame_id != "base_link"
            or len(slots.slots) != 4
            or not 0.0 <= now_s - slots.header.stamp.to_sec() <= MAX_SLOT_AGE_S):
        print(json.dumps({"status": "REJECTED", "reason": "SLOTS_NOT_FRESH_VALID"}))
        raise SystemExit(2)

    get_link_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
    rospy.wait_for_service("/gazebo/get_link_state", timeout=5.0)
    table = get_link_state(WORKBENCH_LINK, "world")
    tip = get_link_state(END_EFFECTOR_LINK, "world")
    if not table.success or not tip.success:
        print(json.dumps({"status": "REJECTED", "reason": "GAZEBO_LINK_STATE_UNAVAILABLE"}))
        raise SystemExit(2)

    table_link_world_z = table.link_state.pose.position.z
    table_top_world_z = table_link_world_z + WORKBENCH_MESH_TOP_OFFSET_M
    observation_tip_world_z = tip.link_state.pose.position.z
    target_tip_world_z = table_top_world_z + args.clearance_m
    signed_vertical_delta_m = target_tip_world_z - observation_tip_world_z
    target_tcp_base_z_m = observation_tcp_base[2] + signed_vertical_delta_m

    camera_surface_base_link_z = sum(
        slot.position.z for slot in slots.slots
    ) / len(slots.slots)
    # base_link origin is 0.060 m above Gazebo base_footprint in oryxbase.xacro.
    base_footprint = get_link_state("oryxbot::base_footprint", "world")
    if not base_footprint.success:
        print(json.dumps({"status": "REJECTED", "reason": "BASE_FOOTPRINT_STATE_UNAVAILABLE"}))
        raise SystemExit(2)
    camera_surface_world_z = (
        base_footprint.link_state.pose.position.z
        + 0.060
        + camera_surface_base_link_z
    )

    report = {
        "status": "CALIBRATED",
        "sim_only": True,
        "read_only": True,
        "oracle_scope": "height_calibration_only",
        "control_authority": False,
        "workbench_link": WORKBENCH_LINK,
        "mesh_top_offset_m": WORKBENCH_MESH_TOP_OFFSET_M,
        "table_link_world_z_m": table_link_world_z,
        "table_top_world_z_m": table_top_world_z,
        "requested_clearance_m": args.clearance_m,
        "target_tip_world_z_m": target_tip_world_z,
        "observation_tip_world_z_m": observation_tip_world_z,
        "required_descent_m": -signed_vertical_delta_m,
        "observation_tcp_base_m": observation_tcp_base,
        "target_tcp_base_z_by_differential_m": target_tcp_base_z_m,
        "camera_surface_base_link_z_m": camera_surface_base_link_z,
        "camera_surface_world_z_m": camera_surface_world_z,
        "camera_surface_height_bias_m": camera_surface_world_z - table_top_world_z,
        "slots_stable_frames": slots.stable_frames,
    }
    print(json.dumps(report, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
