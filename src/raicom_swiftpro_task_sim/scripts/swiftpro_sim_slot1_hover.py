#!/usr/bin/env python3
"""One-shot frame-correct camera-controlled SIM hover over slot 3."""

import json
import math
import threading

import rospy
import tf
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from raicom_msgs.msg import WorkbenchSlots
from raicom_swiftpro_task_sim.kinematics import (
    admission_reason,
    display_projection,
    forward_position,
    inverse_kinematics,
)
from raicom_swiftpro_task_sim.motion_adapter import (
    CONTROLLER_JOINT_NAMES,
    FEEDBACK_JOINT_NAMES,
    JointObservation,
    MotionStatus,
    STEP_SECONDS,
    SwiftproSimMotionAdapter,
)


# Differential probe: slot 4 has a normal extended radius (~242 mm) while
# slot 3 folds near the arm base (~128 mm). These are SIM observations, not rules.
SLOT_INDEX = 3
HOVER_OFFSET_M = 0.080
ARM_BASE_FRAME = "Base"
MAX_SLOT_AGE_S = 0.5
TERMINAL_ERROR_RAD = 0.02
TERMINAL_STABLE_SAMPLES = 10
TERMINAL_TIMEOUT_S = 5.0


def rotated_local_z(orientation):
    x, y, z, w = (
        orientation.x, orientation.y, orientation.z, orientation.w
    )
    return (
        2.0 * (x * z + w * y),
        2.0 * (y * z - w * x),
        1.0 - 2.0 * (x * x + y * y),
    )


class FrameCorrectSlotHoverRunner:
    def __init__(self):
        self._lock = threading.Lock()
        self._joint = None
        self._slots = None
        self._tf_listener = tf.TransformListener()
        self._publishers = {
            name: rospy.Publisher(
                "/swiftpro/%s_position_controller/command" % name,
                Float64,
                queue_size=1,
            )
            for name in CONTROLLER_JOINT_NAMES
        }
        self._joint_subscriber = rospy.Subscriber(
            "/swiftpro/joint_states", JointState, self._on_joint, queue_size=1
        )
        self._slot_subscriber = rospy.Subscriber(
            "/raicom/workbench/slots", WorkbenchSlots,
            self._on_slots, queue_size=1
        )

    def _on_joint(self, message):
        observation = JointObservation(
            tuple(message.name), tuple(message.position),
            tuple(message.velocity), message.header.stamp.to_sec()
        )
        with self._lock:
            self._joint = observation

    def _on_slots(self, message):
        with self._lock:
            self._slots = message

    def snapshot(self):
        with self._lock:
            return self._joint, self._slots

    def publish_projection(self, projection):
        for controller, value in zip(CONTROLLER_JOINT_NAMES, projection):
            self._publishers[controller].publish(Float64(data=value))

    def wait_for_controller_connections(self):
        deadline = rospy.Time.now().to_sec() + 3.0
        while not rospy.is_shutdown():
            if all(
                publisher.get_num_connections() >= 1
                for publisher in self._publishers.values()
            ):
                return True
            if rospy.Time.now().to_sec() >= deadline:
                return False
            rospy.sleep(0.02)
        return False

    @staticmethod
    def finite(values):
        return all(math.isfinite(value) for value in values)

    def target_from_slots(self, message, now_s):
        if not message.valid:
            return None, "SLOTS_INVALID"
        if message.header.frame_id != "base_link":
            return None, "SLOT_FRAME_MISMATCH"
        if len(message.slots) != 4:
            return None, "SLOT_COUNT_MISMATCH"
        stamp_s = message.header.stamp.to_sec()
        if stamp_s > now_s or now_s - stamp_s > MAX_SLOT_AGE_S:
            return None, "SLOTS_STALE"
        slot = message.slots[SLOT_INDEX]
        values = (
            slot.position.x, slot.position.y, slot.position.z,
            slot.orientation.x, slot.orientation.y,
            slot.orientation.z, slot.orientation.w,
        )
        if not self.finite(values):
            return None, "SLOT_NONFINITE"
        quaternion_norm = math.sqrt(sum(value * value for value in values[3:]))
        if abs(quaternion_norm - 1.0) > 0.02:
            return None, "SLOT_QUATERNION_INVALID"
        normal = rotated_local_z(slot.orientation)
        if normal[2] < 0.5:
            return None, "SLOT_NORMAL_UNSAFE"
        hover_source = tuple(
            position + HOVER_OFFSET_M * axis
            for position, axis in zip(values[:3], normal)
        )
        point = PointStamped()
        point.header = message.header
        point.point.x, point.point.y, point.point.z = hover_source
        try:
            self._tf_listener.waitForTransform(
                ARM_BASE_FRAME,
                message.header.frame_id,
                message.header.stamp,
                rospy.Duration(2.0),
            )
            transformed = self._tf_listener.transformPoint(ARM_BASE_FRAME, point)
        except Exception:
            return None, "SLOT_TF_UNAVAILABLE"
        hover_arm_base = (
            transformed.point.x,
            transformed.point.y,
            transformed.point.z,
        )
        return (hover_source, hover_arm_base), "VALID"

    def run(self):
        deadline = rospy.Time.now().to_sec() + 5.0
        while not rospy.is_shutdown():
            joint, slots = self.snapshot()
            if joint is not None and slots is not None:
                break
            if rospy.Time.now().to_sec() >= deadline:
                print(json.dumps({"status": "FAILED", "reason": "INPUT_TIMEOUT"}))
                return 1
            rospy.sleep(0.02)
        if not self.wait_for_controller_connections():
            print(json.dumps({
                "status": "FAILED", "reason": "CONTROLLER_CONNECTION_TIMEOUT"
            }))
            return 1

        rospy.sleep(0.2)
        joint, slots = self.snapshot()
        now_s = rospy.Time.now().to_sec()
        transformed_target, reason = self.target_from_slots(slots, now_s)
        if transformed_target is None:
            print(json.dumps({"status": "FAILED", "reason": reason}))
            return 1
        hover_source, target_xyz_m = transformed_target
        rejected = admission_reason(target_xyz_m)
        if rejected is not None:
            print(json.dumps({
                "status": "FAILED",
                "reason": rejected,
                "source_frame": slots.header.frame_id,
                "hover_in_source_frame_m": hover_source,
                "ik_frame": ARM_BASE_FRAME,
                "target_xyz_m": target_xyz_m,
            }, sort_keys=True))
            return 1
        target_independent = inverse_kinematics(target_xyz_m)
        if target_independent is None:
            print(json.dumps({
                "status": "FAILED", "reason": "FRAME_CORRECT_HOVER_UNREACHABLE",
                "source_frame": slots.header.frame_id,
                "hover_in_source_frame_m": hover_source,
                "ik_frame": ARM_BASE_FRAME,
                "target_xyz_m": target_xyz_m,
            }, sort_keys=True))
            return 1

        planned = SwiftproSimMotionAdapter().plan(
            target_independent, joint, now_s
        )
        if planned.status is not MotionStatus.READY or planned.plan is None:
            print(json.dumps({"status": "FAILED", "reason": planned.reason}))
            return 1
        for projection in planned.plan.commands:
            if rospy.is_shutdown():
                return 1
            self.publish_projection(projection)
            rospy.sleep(STEP_SECONDS)
        target_display = display_projection(target_independent)
        terminal_deadline = rospy.Time.now().to_sec() + TERMINAL_TIMEOUT_S
        terminal_stable = 0
        max_position_error = math.inf
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < terminal_deadline:
            self.publish_projection(target_display)
            rospy.sleep(0.02)
            measured, _ = self.snapshot()
            positions = dict(zip(measured.names, measured.positions_rad))
            max_position_error = max(
                abs(positions[name] - value)
                for name, value in zip(FEEDBACK_JOINT_NAMES, target_display)
            )
            terminal_stable = terminal_stable + 1 if max_position_error <= TERMINAL_ERROR_RAD else 0
            if terminal_stable >= TERMINAL_STABLE_SAMPLES:
                break
        if terminal_stable < TERMINAL_STABLE_SAMPLES:
            print(json.dumps({
                "status": "FAILED", "reason": "TERMINAL_NOT_ACCEPTED",
                "max_position_error_rad": max_position_error,
            }, sort_keys=True))
            return 1

        measured, _ = self.snapshot()
        positions = dict(zip(measured.names, measured.positions_rad))
        velocities = dict(zip(measured.names, measured.velocities_rad_s))
        measured_independent = (
            positions["Joint1"] + math.pi / 2.0,
            math.pi / 2.0 - positions["Joint2"],
            positions["Joint6"] + math.radians(3.8),
        )
        measured_xyz_m = forward_position(measured_independent)
        cartesian_error_mm = None if measured_xyz_m is None else [
            (measured_xyz_m[index] - target_xyz_m[index]) * 1000.0
            for index in range(3)
        ]
        print(json.dumps({
            "sim_only": True,
            "no_load": True,
            "control_source": "/raicom/workbench/slots",
            "oracle_used": False,
            "slot_id": SLOT_INDEX + 1,
            "hover_offset_m": HOVER_OFFSET_M,
            "source_frame": slots.header.frame_id,
            "hover_in_source_frame_m": hover_source,
            "ik_frame": ARM_BASE_FRAME,
            "target_xyz_m": target_xyz_m,
            "target_independent_deg": [
                math.degrees(value) for value in target_independent
            ],
            "status": "TRAJECTORY_COMPLETE",
            "reason": "FRAME_CORRECT_SLOT4_HOVER_SETPOINT_HELD",
            "duration_s": planned.plan.duration_s,
            "terminal_stable_samples": terminal_stable,
            "max_position_error_rad": max_position_error,
            "feedback_tcp_xyz_m": measured_xyz_m,
            "feedback_cartesian_error_mm": cartesian_error_mm,
            "max_abs_velocity_rad_s": max(
                abs(velocities[name]) for name in FEEDBACK_JOINT_NAMES
            ),
        }, sort_keys=True), flush=True)
        return 0


def main():
    rospy.init_node("swiftpro_sim_frame_correct_slot4_hover", anonymous=True)
    raise SystemExit(FrameCorrectSlotHoverRunner().run())


if __name__ == "__main__":
    main()
