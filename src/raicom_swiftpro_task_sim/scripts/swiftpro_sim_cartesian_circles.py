#!/usr/bin/env python3
"""SIM-only fixed-Base Cartesian circles with detailed JSONL diagnostics."""

import json
import math
from pathlib import Path
import numbers
import threading
import time

import rospy
import tf
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from raicom_swiftpro_task_sim.kinematics import (
    admission_reason,
    display_projection,
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


CENTER_XY_M = (0.200, -0.150)
RADIUS_M = 0.030
LEVELS_Z_M = (0.120, 0.090)
POINTS_PER_CIRCLE = 24
POINT_WAIT_S = 0.15
POINT_POSITION_TOLERANCE_RAD = 0.01
POINT_ACCEPT_TIMEOUT_S = 1.0
POINT_REQUIRED_SAMPLES = 3
TRAJECTORY_DEADLINE_MARGIN_S = 2.0
JOINT_OBSERVATION_MAX_AGE_S = 0.25
ARM_BASE_FRAME = "Base"
TCP_FRAME = "virtual_end_effector"
LOG_EVERY_STEPS = 10


class CartesianCircleRunner:
    def __init__(self):
        self._lock = threading.Lock()
        self._joint = None
        self._tf_listener = tf.TransformListener()
        # Sim time may still be zero while the node is being constructed.
        # Wall-clock nanoseconds are used only for an artifact-unique filename.
        log_name = "swiftpro-cartesian-circles-%d.jsonl" % time.time_ns()
        self.log_path = Path("/workspace/runs") / log_name
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log_file = self.log_path.open("w")
        self._publishers = {
            name: rospy.Publisher(
                "/swiftpro/%s_position_controller/command" % name,
                Float64,
                queue_size=1,
            )
            for name in CONTROLLER_JOINT_NAMES
        }
        self._subscriber = rospy.Subscriber(
            "/swiftpro/joint_states", JointState, self._on_joint, queue_size=1
        )

    def _on_joint(self, message):
        observation = JointObservation(
            tuple(message.name), tuple(message.position),
            tuple(message.velocity), message.header.stamp.to_sec()
        )
        with self._lock:
            self._joint = observation

    def latest(self):
        with self._lock:
            return self._joint

    def log(self, event, **fields):
        payload = {
            "event": event,
            "ros_time_s": rospy.Time.now().to_sec(),
        }
        payload.update(fields)
        self._log_file.write(json.dumps(payload, sort_keys=True) + "\n")
        self._log_file.flush()

    def wait_ready(self):
        deadline = rospy.Time.now().to_sec() + 5.0
        while not rospy.is_shutdown():
            if self.latest() is not None and all(
                publisher.get_num_connections() >= 1
                for publisher in self._publishers.values()
            ):
                return True
            if rospy.Time.now().to_sec() >= deadline:
                return False
            rospy.sleep(0.02)
        return False

    def validated_joint_vectors(self, now_s=None):
        observation = self.latest()
        if not isinstance(observation, JointObservation):
            return None, None, "MISSING_JOINT_STATES"
        names = observation.names
        if (not isinstance(names, tuple)
                or not all(isinstance(name, str) for name in names)):
            return None, None, "INVALID_JOINT_NAMES"
        if (len(set(names)) != len(names)
                or not set(FEEDBACK_JOINT_NAMES).issubset(names)):
            return None, None, "INVALID_JOINT_NAMES"
        if (not isinstance(observation.positions_rad, tuple)
                or not isinstance(observation.velocities_rad_s, tuple)
                or len(observation.positions_rad) != len(names)
                or len(observation.velocities_rad_s) != len(names)):
            return None, None, "INVALID_JOINT_VECTORS"
        values = observation.positions_rad + observation.velocities_rad_s
        if not all(isinstance(value, numbers.Real)
                   and not isinstance(value, bool)
                   and math.isfinite(value) for value in values):
            return None, None, "NONFINITE_JOINT_STATES"
        if now_s is not None:
            stamp_s = observation.observed_at_s
            if (not isinstance(stamp_s, numbers.Real)
                    or isinstance(stamp_s, bool) or not math.isfinite(stamp_s)
                    or stamp_s > now_s
                    or now_s - stamp_s > JOINT_OBSERVATION_MAX_AGE_S):
                return None, None, "STALE_JOINT_STATES"
        return (
            dict(zip(names, observation.positions_rad)),
            dict(zip(names, observation.velocities_rad_s)),
            "",
        )

    def publish_measured_hold(self):
        positions, _, reason = self.validated_joint_vectors(
            rospy.Time.now().to_sec()
        )
        if reason:
            return False, reason
        for feedback, controller in zip(
                FEEDBACK_JOINT_NAMES, CONTROLLER_JOINT_NAMES):
            self._publishers[controller].publish(Float64(data=positions[feedback]))
        return True, "MEASURED_HOLD"

    def measured_snapshot(self):
        observation = self.latest()
        positions, velocities, reason = self.validated_joint_vectors()
        if reason:
            return {"valid": False, "reason": reason}
        tcp_xyz = None
        tcp_quaternion = None
        tcp_rpy = None
        try:
            self._tf_listener.waitForTransform(
                ARM_BASE_FRAME, TCP_FRAME, rospy.Time(0), rospy.Duration(1.0)
            )
            tcp_xyz, tcp_quaternion = self._tf_listener.lookupTransform(
                ARM_BASE_FRAME, TCP_FRAME, rospy.Time(0)
            )
            tcp_rpy = tf.transformations.euler_from_quaternion(tcp_quaternion)
        except Exception:
            pass
        return {
            "valid": True,
            "joint_stamp_s": observation.observed_at_s,
            "joint_positions_rad": [positions[name] for name in FEEDBACK_JOINT_NAMES],
            "joint_velocities_rad_s": [velocities[name] for name in FEEDBACK_JOINT_NAMES],
            "tcp_xyz_m": tcp_xyz,
            "tcp_quaternion_xyzw": tcp_quaternion,
            "tcp_rpy_rad": tcp_rpy,
        }

    def wait_for_position_target(self, target_display):
        start_s = rospy.Time.now().to_sec()
        deadline_s = start_s + POINT_ACCEPT_TIMEOUT_S
        accepted_samples = 0
        previous_stamp_s = None
        max_error_rad = float("inf")
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline_s:
            observation = self.latest()
            positions, _, reason = self.validated_joint_vectors(
                rospy.Time.now().to_sec()
            )
            if reason or observation.observed_at_s == previous_stamp_s:
                rospy.sleep(0.01)
                continue
            previous_stamp_s = observation.observed_at_s
            max_error_rad = max(
                abs(positions[name] - target)
                for name, target in zip(FEEDBACK_JOINT_NAMES, target_display)
            )
            accepted_samples = (
                accepted_samples + 1
                if max_error_rad <= POINT_POSITION_TOLERANCE_RAD
                else 0
            )
            if accepted_samples >= POINT_REQUIRED_SAMPLES:
                return True, rospy.Time.now().to_sec() - start_s, max_error_rad
            rospy.sleep(0.01)
        return False, rospy.Time.now().to_sec() - start_s, max_error_rad

    def move_to_xyz(self, xyz_m, level, point):
        rejected = admission_reason(xyz_m)
        if rejected is not None:
            return None, rejected
        target_independent = inverse_kinematics(xyz_m)
        if target_independent is None:
            return None, "IK_FAILED"
        target_display = display_projection(target_independent)
        rospy.sleep(0.1)
        observation = self.latest()
        planned = SwiftproSimMotionAdapter().plan(
            target_independent, observation, rospy.Time.now().to_sec()
        )
        if planned.status is not MotionStatus.READY or planned.plan is None:
            return None, planned.reason
        start_snapshot = self.measured_snapshot()
        if not start_snapshot.get("valid"):
            return None, start_snapshot["reason"]
        self.log(
            "point_start",
            level=level,
            point=point,
            command_xyz_m=xyz_m,
            target_independent_rad=target_independent,
            target_display_rad=target_display,
            planned_duration_s=planned.plan.duration_s,
            measured=start_snapshot,
        )
        command_count = len(planned.plan.commands)
        trajectory_deadline_s = (
            rospy.Time.now().to_sec()
            + planned.plan.duration_s
            + TRAJECTORY_DEADLINE_MARGIN_S
        )
        for step, projection in enumerate(planned.plan.commands, 1):
            for controller, value in zip(CONTROLLER_JOINT_NAMES, projection):
                now_s = rospy.Time.now().to_sec()
                if rospy.is_shutdown() or now_s >= trajectory_deadline_s:
                    held, hold_reason = self.publish_measured_hold()
                    self.log(
                        "trajectory_interrupted",
                        level=level,
                        point=point,
                        step=step,
                        controller=controller,
                        reason=("SHUTDOWN" if rospy.is_shutdown()
                                else "TRAJECTORY_DEADLINE"),
                        held=held,
                        hold_reason=hold_reason,
                    )
                    return None, (
                        "SHUTDOWN" if rospy.is_shutdown()
                        else "TRAJECTORY_DEADLINE"
                    )
                self._publishers[controller].publish(Float64(data=value))
            if step == 1 or step % LOG_EVERY_STEPS == 0 or step == command_count:
                self.log(
                    "trajectory_sample",
                    level=level,
                    point=point,
                    step=step,
                    steps=command_count,
                    command_display_rad=projection,
                    measured=self.measured_snapshot(),
                )
            rospy.sleep(STEP_SECONDS)
        # plan_motion() includes the exact target as its final command. Do not
        # republish it outside the per-controller shutdown/deadline checks.
        rospy.sleep(POINT_WAIT_S)
        position_accepted, acceptance_wait_s, acceptance_error_rad = (
            self.wait_for_position_target(target_display)
        )

        measured = self.measured_snapshot()
        if not measured.get("valid"):
            held, hold_reason = self.publish_measured_hold()
            self.log(
                "invalid_terminal_observation",
                level=level,
                point=point,
                reason=measured["reason"],
                held=held,
                hold_reason=hold_reason,
            )
            return None, measured["reason"]
        positions = measured["joint_positions_rad"]
        velocities = measured["joint_velocities_rad_s"]
        tcp_xyz = measured["tcp_xyz_m"]
        if tcp_xyz is None:
            return None, "TCP_TF_UNAVAILABLE"
        tcp_error_m = math.sqrt(sum(
            (actual - target) ** 2 for actual, target in zip(tcp_xyz, xyz_m)
        ))
        result = {
            "command_xyz_m": xyz_m,
            "target_independent_rad": target_independent,
            "target_display_rad": target_display,
            "tcp_xyz_m": tcp_xyz,
            "tcp_quaternion_xyzw": measured["tcp_quaternion_xyzw"],
            "tcp_rpy_rad": measured["tcp_rpy_rad"],
            "tcp_error_m": tcp_error_m,
            "duration_s": planned.plan.duration_s,
            "position_accepted": position_accepted,
            "acceptance_wait_s": acceptance_wait_s,
            "acceptance_error_rad": acceptance_error_rad,
            "joint_positions_rad": positions,
            "joint_velocities_rad_s": velocities,
            "joint_errors_rad": [
                actual - target for actual, target in zip(positions, target_display)
            ],
            "max_position_error_rad": max(
                abs(actual - target)
                for actual, target in zip(positions, target_display)
            ),
            "max_abs_velocity_rad_s": max(abs(value) for value in velocities),
        }
        self.log("point_end", level=level, point=point, **result)
        print(json.dumps({
            "event": "point_end", "level": level, "point": point,
            "command_xyz_m": xyz_m,
            "tcp_xyz_m": tcp_xyz,
            "tcp_rpy_deg": [math.degrees(value) for value in measured["tcp_rpy_rad"]],
            "tcp_error_m": tcp_error_m,
            "max_position_error_rad": result["max_position_error_rad"],
            "max_abs_velocity_rad_s": result["max_abs_velocity_rad_s"],
            "log_path": str(self.log_path),
        }, sort_keys=True), flush=True)
        return result, "OK"

    def run(self):
        self.log(
            "run_start",
            center_xy_m=CENTER_XY_M,
            radius_m=RADIUS_M,
            levels_z_m=LEVELS_Z_M,
            points_per_circle=POINTS_PER_CIRCLE,
            cv_used=False,
            oracle_used=False,
        )
        if not self.wait_ready():
            self.log("run_failed", reason="INPUT_OR_CONNECTION_TIMEOUT")
            return 1
        points = []
        for level, z_m in enumerate(LEVELS_Z_M, 1):
            for index in range(POINTS_PER_CIRCLE):
                angle = 2.0 * math.pi * index / POINTS_PER_CIRCLE
                xyz_m = (
                    CENTER_XY_M[0] + RADIUS_M * math.cos(angle),
                    CENTER_XY_M[1] + RADIUS_M * math.sin(angle),
                    z_m,
                )
                result, reason = self.move_to_xyz(xyz_m, level, index + 1)
                if result is None:
                    self.log(
                        "run_failed", reason=reason, level=level,
                        point=index + 1, command_xyz_m=xyz_m,
                    )
                    return 1
                result.update({"level": level, "point": index + 1})
                points.append(result)

        summary = {
            "status": "TRAJECTORY_COMPLETE",
            "reason": "TWO_LEVEL_CARTESIAN_CIRCLES_COMPLETE",
            "max_tcp_error_m": max(point["tcp_error_m"] for point in points),
            "max_position_error_rad": max(
                point["max_position_error_rad"] for point in points
            ),
            "max_abs_velocity_rad_s": max(
                point["max_abs_velocity_rad_s"] for point in points
            ),
            "log_path": str(self.log_path),
        }
        self.log("run_end", **summary)
        print(json.dumps(summary, sort_keys=True), flush=True)
        return 0


def main():
    rospy.init_node("swiftpro_sim_cartesian_circles", anonymous=True)
    runner = CartesianCircleRunner()
    try:
        raise SystemExit(runner.run())
    finally:
        runner._log_file.close()


if __name__ == "__main__":
    main()
