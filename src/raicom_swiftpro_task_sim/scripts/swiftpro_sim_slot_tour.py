#!/usr/bin/env python3
"""Freeze a camera slot snapshot and visit each admitted SIM hover target."""
from __future__ import annotations

import argparse
import json
import math
import threading

import rospkg
import rospy
import tf
import yaml
from geometry_msgs.msg import PointStamped
from raicom_msgs.msg import WorkbenchSlots
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64

from raicom_swiftpro_task_sim.kinematics import (
    admission_reason,
    display_projection,
    forward_position,
    inverse_kinematics,
)
from raicom_swiftpro_task_sim.motion_adapter import (
    CONTROLLER_JOINT_NAMES,
    JointObservation,
    MotionStatus,
    STEP_SECONDS,
    SwiftproSimMotionAdapter,
)

ARM_BASE_FRAME = "Base"
CAMERA_SLOT_FRAME = "hand_cam_link"
MAX_SLOT_AGE_S = 0.5
OBSERVATION_JOINTS_RAD = {
    "Joint1": math.radians(-42.0),
    "Joint2": 0.0,
    "Joint6": 0.0,
}
OBSERVATION_TOLERANCE_RAD = 0.03
TERMINAL_ERROR_RAD = 0.02
TERMINAL_STABLE_SAMPLES = 3
TERMINAL_TIMEOUT_S = 6.0
DRIVEN_FEEDBACK_NAMES = ("Joint1", "Joint2", "Joint6")
DRIVEN_DISPLAY_INDICES = (0, 1, 5)


def placement_hover_from_profile():
    path = (rospkg.RosPack().get_path("raicom_swiftpro_task_sim")
            + "/config/placement_height.yaml")
    with open(path, encoding="utf-8") as stream:
        profile = yaml.safe_load(stream)
    if (profile.get("runtime_surface_authority") != "AR_HAND_EYE"
            or profile.get("height_axis") != "workbench_outward_normal"):
        raise ValueError("invalid placement height profile")
    clearance = float(profile["surface_clearance_m"])
    if not math.isfinite(clearance) or clearance < 0.0:
        raise ValueError("invalid placement height value")
    # Both SIM and venue target the camera-observed surface plus this clearance.
    return clearance, clearance


def rotated_local_z(orientation):
    x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
    return (
        2.0 * (x * z + w * y),
        2.0 * (y * z - w * x),
        1.0 - 2.0 * (x * x + y * y),
    )


class SlotTour:
    def __init__(self, hover_m, dwell_s):
        self.hover_m = hover_m
        self.dwell_s = dwell_s
        self.lock = threading.Lock()
        self.joint = None
        self.slots = None
        self.grasping = None
        self.tf = tf.TransformListener()
        self.adapter = SwiftproSimMotionAdapter()
        self.publishers = {
            name: rospy.Publisher(
                "/swiftpro/%s_position_controller/command" % name,
                Float64, queue_size=1,
            )
            for name in CONTROLLER_JOINT_NAMES
        }
        self.arm_extended = rospy.Publisher(
            "/raicom/safety/arm_extended", Bool, queue_size=1, latch=True
        )
        rospy.Subscriber("/swiftpro/joint_states", JointState, self.on_joint, queue_size=1)
        rospy.Subscriber("/raicom/workbench/slots", WorkbenchSlots, self.on_slots, queue_size=1)
        rospy.Subscriber("/swiftpro/grasping", Bool, self.on_grasping, queue_size=1)

    def on_joint(self, message):
        observation = JointObservation(
            tuple(message.name), tuple(message.position),
            tuple(message.velocity), message.header.stamp.to_sec(),
        )
        with self.lock:
            self.joint = observation

    def on_slots(self, message):
        with self.lock:
            self.slots = message

    def on_grasping(self, message):
        with self.lock:
            self.grasping = message.data

    def snapshot(self):
        with self.lock:
            return self.joint, self.slots, self.grasping

    @staticmethod
    def finite(values):
        return all(math.isfinite(value) for value in values)

    @staticmethod
    def observation_pose(joint):
        if joint is None:
            return False
        positions = dict(zip(joint.names, joint.positions_rad))
        return all(
            name in positions and abs(positions[name] - expected) <= OBSERVATION_TOLERANCE_RAD
            for name, expected in OBSERVATION_JOINTS_RAD.items()
        )

    def publish_projection(self, projection):
        for name, value in zip(CONTROLLER_JOINT_NAMES, projection):
            self.publishers[name].publish(Float64(data=value))

    def connections_ready(self):
        deadline = rospy.Time.now().to_sec() + 3.0
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
            if all(publisher.get_num_connections() >= 1 for publisher in self.publishers.values()):
                return True
            rospy.sleep(0.02)
        return False

    def transform_target(self, message, slot, hover_m):
        normal = rotated_local_z(slot.orientation)
        # hand_cam_link is an optical frame (+Z into the scene); the observed
        # table outward normal must point back toward the camera (-Z).
        if not self.finite(normal) or normal[2] > -0.5:
            return None, None, "SLOT_NORMAL_UNSAFE"
        source = tuple(
            coordinate + hover_m * axis
            for coordinate, axis in zip(
                (slot.position.x, slot.position.y, slot.position.z), normal
            )
        )
        point = PointStamped()
        point.header.frame_id = message.header.frame_id
        # Preflight requires the arm to remain at the observation pose, so the
        # latest hand-eye transform represents the frozen camera snapshot while
        # avoiding listener-start cache gaps at an older image timestamp.
        point.header.stamp = rospy.Time(0)
        point.point.x, point.point.y, point.point.z = source
        try:
            self.tf.waitForTransform(
                ARM_BASE_FRAME, message.header.frame_id,
                rospy.Time(0), rospy.Duration(2.0),
            )
            transformed = self.tf.transformPoint(ARM_BASE_FRAME, point)
        except Exception:
            return source, None, "SLOT_TF_UNAVAILABLE"
        target = (transformed.point.x, transformed.point.y, transformed.point.z)
        rejected = admission_reason(target)
        return source, target, "VALID" if rejected is None else rejected

    def preflight(self, slots, observation_tcp):
        targets = []
        for index, slot in enumerate(slots.slots):
            source, target, reason = self.transform_target(slots, slot, self.hover_m)
            joints = inverse_kinematics(target) if reason == "VALID" else None
            if joints is None:
                targets.append({
                    "slot_id": index + 1,
                    "status": "REJECTED",
                    "hover_m": self.hover_m,
                    "target_base_m": target,
                    "reason": reason if reason != "VALID" else "IK_FAILED",
                })
                continue
            delta = tuple(target[axis] - observation_tcp[axis] for axis in range(3))
            targets.append({
                "slot_id": index + 1,
                "hover_m": self.hover_m,
                "source_xyz_m": source,
                "target_base_m": target,
                "delta_from_observation_base_m": delta,
                "distance_from_observation_m": math.sqrt(sum(value * value for value in delta)),
                "independent_rad": joints,
            })
        return targets

    def move(self, target, joint):
        planned = self.adapter.plan(target["independent_rad"], joint, rospy.Time.now().to_sec())
        if planned.status is not MotionStatus.READY or planned.plan is None:
            return {"status": "FAILED", "reason": planned.reason}
        for projection in planned.plan.commands:
            self.publish_projection(projection)
            rospy.sleep(STEP_SECONDS)
        target_display = display_projection(target["independent_rad"])
        deadline = rospy.Time.now().to_sec() + TERMINAL_TIMEOUT_S
        stable = 0
        max_error = math.inf
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
            self.publish_projection(target_display)
            rospy.sleep(0.02)
            measured, _, _ = self.snapshot()
            positions = dict(zip(measured.names, measured.positions_rad))
            max_error = max(
                abs(positions[name] - target_display[index])
                for name, index in zip(DRIVEN_FEEDBACK_NAMES, DRIVEN_DISPLAY_INDICES)
            )
            stable = stable + 1 if max_error <= TERMINAL_ERROR_RAD else 0
            if stable >= TERMINAL_STABLE_SAMPLES:
                break
        if stable < TERMINAL_STABLE_SAMPLES:
            return {"status": "FAILED", "reason": "TERMINAL_NOT_ACCEPTED", "max_error_rad": max_error}
        hold_deadline = rospy.Time.now().to_sec() + self.dwell_s
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < hold_deadline:
            self.publish_projection(target_display)
            rospy.sleep(0.02)
        measured, _, _ = self.snapshot()
        positions = dict(zip(measured.names, measured.positions_rad))
        independent = (
            positions["Joint1"] + math.pi / 2.0,
            math.pi / 2.0 - positions["Joint2"],
            positions["Joint6"] + math.radians(3.8),
        )
        tcp = forward_position(independent)
        error_mm = None if tcp is None else [
            (tcp[i] - target["target_base_m"][i]) * 1000.0 for i in range(3)
        ]
        return {
            "status": "HELD",
            "dwell_s": self.dwell_s,
            "max_error_rad": max_error,
            "feedback_tcp_base_m": tcp,
            "feedback_error_mm": error_mm,
        }

    def run(self):
        if rospy.get_param("/raicom/backend", "") != "SIM":
            print(json.dumps({"status": "REJECTED", "reason": "REAL_BACKEND"}))
            return 2
        deadline = rospy.Time.now().to_sec() + 5.0
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
            joint, slots, grasping = self.snapshot()
            if joint is not None and slots is not None and grasping is not None:
                break
            rospy.sleep(0.02)
        joint, slots, grasping = self.snapshot()
        if joint is None or slots is None or grasping is None:
            print(json.dumps({"status": "REJECTED", "reason": "INPUT_TIMEOUT"}))
            return 2
        if grasping:
            print(json.dumps({"status": "REJECTED", "reason": "GRIPPER_NOT_EMPTY"}))
            return 2
        now_s = rospy.Time.now().to_sec()
        if (not slots.valid or slots.header.frame_id != CAMERA_SLOT_FRAME
                or len(slots.slots) != 4
                or not 0.0 <= now_s - slots.header.stamp.to_sec() <= MAX_SLOT_AGE_S):
            print(json.dumps({"status": "REJECTED", "reason": "SLOTS_NOT_FRESH_VALID"}))
            return 2
        if not self.observation_pose(joint):
            print(json.dumps({"status": "REJECTED", "reason": "ARM_NOT_AT_OBSERVATION"}))
            return 2
        if not self.connections_ready():
            print(json.dumps({"status": "REJECTED", "reason": "CONTROLLER_CONNECTION_TIMEOUT"}))
            return 2

        joint_positions = dict(zip(joint.names, joint.positions_rad))
        observation_independent = (
            joint_positions["Joint1"] + math.pi / 2.0,
            math.pi / 2.0 - joint_positions["Joint2"],
            joint_positions["Joint6"] + math.radians(3.8),
        )
        observation_tcp = forward_position(observation_independent)
        if observation_tcp is None:
            print(json.dumps({"status": "REJECTED", "reason": "OBSERVATION_FK_FAILED"}))
            return 2
        frozen = {
            "stamp_s": slots.header.stamp.to_sec(),
            "frame_id": slots.header.frame_id,
            "stable_frames": slots.stable_frames,
            "observation_tcp_base_m": observation_tcp,
            "slot_xyz_m": [
                [slot.position.x, slot.position.y, slot.position.z]
                for slot in slots.slots
            ],
        }
        targets = self.preflight(slots, observation_tcp)
        print(json.dumps({
            "event": "FROZEN_PREFLIGHT", "sim_only": True,
            "hover_m": self.hover_m,
            "frozen_observation": frozen,
            "targets": targets,
        }, sort_keys=True), flush=True)
        admitted = [target for target in targets if target.get("status") != "REJECTED"]
        if not admitted:
            return 2

        self.arm_extended.publish(Bool(data=True))
        results = []
        for target in admitted:
            joint, _, _ = self.snapshot()
            result = self.move(target, joint)
            result.update({
                "slot_id": target["slot_id"],
                "hover_m": target["hover_m"],
                "target_base_m": target["target_base_m"],
            })
            results.append(result)
            print(json.dumps({"event": "SLOT_RESULT", **result}, sort_keys=True), flush=True)
            if result["status"] != "HELD":
                break
        print(json.dumps({
            "status": "TOUR_COMPLETE" if len(results) == len(admitted) and all(item["status"] == "HELD" for item in results) else "TOUR_ABORTED",
            "admitted_slot_ids": [target["slot_id"] for target in admitted],
            "rejected_slot_ids": [target["slot_id"] for target in targets if target.get("status") == "REJECTED"],
            "results": results,
            "gripper_commanded": False,
            "oracle_used": False,
            "arm_extended_latched": True,
        }, sort_keys=True), flush=True)
        return 0 if len(results) == len(admitted) and all(item["status"] == "HELD" for item in results) else 2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--hover-m", type=float)
    parser.add_argument("--dwell-s", type=float, default=1.0)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("swiftpro_sim_slot_tour", anonymous=True)
    try:
        profile_hover, clearance = placement_hover_from_profile()
    except (KeyError, TypeError, ValueError, OSError) as error:
        parser.error(str(error))
    hover = profile_hover if args.hover_m is None else args.hover_m
    if not (0.01 <= hover <= 0.15 and 0.1 <= args.dwell_s <= 5.0):
        parser.error("invalid hover or dwell range")
    print(json.dumps({
        "event": "PLACEMENT_HEIGHT_PROFILE",
        "surface_clearance_m": clearance,
        "effective_normal_offset_m": hover,
    }, sort_keys=True), flush=True)
    raise SystemExit(SlotTour(hover, args.dwell_s).run())


if __name__ == "__main__":
    main()
