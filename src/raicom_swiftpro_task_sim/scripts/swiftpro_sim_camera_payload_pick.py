#!/usr/bin/env python3
"""SIM-only exact-payload camera pick and mandatory rear placement.

The source target is CameraPayloadPick + timestamped TF only. Gazebo state is
never read. A post-grasp failure retains vacuum and the measured arm hold so an
elevated payload is not deliberately dropped.
"""
import argparse
import json
import math
import threading

import rospy
import tf
import yaml
from geometry_msgs.msg import PointStamped, Vector3Stamped
from raicom_msgs.msg import CameraPayloadPick, CameraPayloadPickRequest
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Empty

from raicom_swiftpro_task_sim.kinematics import (
    admission_reason,
    display_projection,
    independent_in_limits,
    inverse_kinematics,
)
from raicom_swiftpro_task_sim.motion_adapter import (
    CONTROLLER_JOINT_NAMES,
    STEP_SECONDS,
    JointObservation,
    MotionStatus,
    SwiftproSimMotionAdapter,
)
from raicom_swiftpro_task_sim.payload_pick import (
    CartesianStage,
    build_motion_plan,
    execute_motion_plan,
    request_from_config,
)

BASE_FRAME = "Base"
ARM_EXTENDED_TOPIC = "/raicom/safety/arm_extended"
DRIVEN_FEEDBACK_NAMES = ("Joint1", "Joint2", "Joint6")
DRIVEN_DISPLAY_INDICES = (0, 1, 5)
TERMINAL_TIMEOUT_S = 6.0
TERMINAL_ERROR_RAD = 0.02
TERMINAL_STABLE_SAMPLES = 3


def _unit(values):
    length = math.sqrt(sum(value * value for value in values))
    if not math.isfinite(length) or abs(length - 1.0) > 0.02:
        return None
    return tuple(value / length for value in values)


class CameraPayloadPickRunner:
    def __init__(self, config, payload):
        self.config = config
        self.payload = payload
        self.lock = threading.Lock()
        self.target = None
        self.target_samples = []
        self.joint = None
        self.grasping = (None, 0.0)
        self.request_id = 0
        self.tf_listener = tf.TransformListener()
        self.adapter = SwiftproSimMotionAdapter()
        self.publishers = {
            name: rospy.Publisher(
                "/swiftpro/%s_position_controller/command" % name,
                Float64,
                queue_size=1,
            )
            for name in CONTROLLER_JOINT_NAMES
        }
        self.extended = rospy.Publisher(
            ARM_EXTENDED_TOPIC, Bool, queue_size=1, latch=True
        )
        self.pick_request = rospy.Publisher(
            "/raicom/workbench/camera_payload_pick/request",
            CameraPayloadPickRequest,
            queue_size=1,
        )
        rospy.Subscriber(
            "/raicom/workbench/camera_payload_pick",
            CameraPayloadPick,
            self._target,
            queue_size=4,
        )
        rospy.Subscriber(
            "/swiftpro/joint_states", JointState, self._joint, queue_size=1
        )
        rospy.Subscriber(
            "/swiftpro/grasping", Bool, self._grasping, queue_size=1
        )

    def _target(self, message):
        if message.payload == self.payload.payload:
            with self.lock:
                self.target = message
                self.target_samples.append(message)
                self.target_samples = self.target_samples[-100:]

    def _joint(self, message):
        observation = JointObservation(
            tuple(message.name),
            tuple(message.position),
            tuple(message.velocity),
            message.header.stamp.to_sec(),
        )
        with self.lock:
            self.joint = observation

    def _grasping(self, message):
        with self.lock:
            self.grasping = (bool(message.data), rospy.Time.now().to_sec())

    def snapshot(self):
        with self.lock:
            return self.target, self.joint, self.grasping

    def target_base(self, target):
        now = rospy.Time.now().to_sec()
        motion = self.config["motion"]
        if (
            target.payload != self.payload.payload
            or target.header.stamp.to_sec() <= 0.0
            or now - target.header.stamp.to_sec() > motion["target_age_s"]
            or not target.header.frame_id
        ):
            return None
        point = PointStamped()
        point.header = target.header
        point.point = target.tcp_pick_point
        normal = Vector3Stamped()
        normal.header = target.header
        normal.vector = target.support_normal
        try:
            self.tf_listener.waitForTransform(
                BASE_FRAME,
                target.header.frame_id,
                target.header.stamp,
                rospy.Duration(1.0),
            )
            transformed_point = self.tf_listener.transformPoint(BASE_FRAME, point)
            transformed_normal = self.tf_listener.transformVector3(BASE_FRAME, normal)
        except Exception:
            return None
        direction = _unit(
            (
                transformed_normal.vector.x,
                transformed_normal.vector.y,
                transformed_normal.vector.z,
            )
        )
        xyz = (
            transformed_point.point.x,
            transformed_point.point.y,
            transformed_point.point.z,
        )
        if direction is None or not all(math.isfinite(value) for value in xyz):
            return None
        return xyz, direction

    def acquire_snapshot_source(self):
        """Aggregate a bounded post-request burst; no sample survives requests."""
        motion = self.config["motion"]
        with self.lock:
            self.target = None
            self.target_samples = []
        connection_deadline = rospy.Time.now().to_sec() + 0.5
        while (not rospy.is_shutdown()
               and self.pick_request.get_num_connections() == 0
               and rospy.Time.now().to_sec() < connection_deadline):
            rospy.sleep(0.01)
        requested_at = rospy.Time.now().to_sec()
        self.request_id += 1
        request_id = self.request_id
        self.pick_request.publish(CameraPayloadPickRequest(
            request_id=request_id,
            payload=self.payload.payload,
        ))
        deadline = requested_at + motion["target_wait_s"]
        examined = 0
        detections = []
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
            with self.lock:
                samples = list(self.target_samples)
            while examined < len(samples) and examined < motion["snapshot_max_frames"]:
                sample = samples[examined]
                examined += 1
                stamp = sample.header.stamp.to_sec()
                if (stamp <= requested_at
                        or sample.request_id != request_id
                        or sample.payload != self.payload.payload
                        or sample.quality <= 0.0
                        or not sample.detector
                        or sample.detail not in ("AWAITING_STABLE_FRAMES", "STABLE")):
                    continue
                transformed = self.target_base(sample)
                if transformed is not None:
                    detections.append((sample, transformed))
            required = motion["snapshot_required_detections"]
            if len(detections) >= required:
                selected = detections[-required:]
                centers = [
                    (item[0].pixel_center_x, item[0].pixel_center_y)
                    for item in selected
                ]
                spread = max(
                    math.dist(first, second)
                    for first in centers for second in centers
                )
                if spread <= motion["snapshot_max_center_spread_px"]:
                    sample, transformed = selected[-1]
                    print(json.dumps({
                        "event": "CAMERA_PAYLOAD_SNAPSHOT_FROZEN",
                        "payload": self.payload.payload,
                        "request_stamp_s": requested_at,
                        "request_id": request_id,
                        "request_payload": self.payload.payload,
                        "selected_stamp_s": sample.header.stamp.to_sec(),
                        "examined_frames": examined,
                        "successful_detections": len(detections),
                        "center_spread_px": spread,
                        "detector": sample.detector,
                        "gazebo_control_authority": False,
                    }), flush=True)
                    return transformed
                detections = detections[-(required - 1):]
            if examined >= motion["snapshot_max_frames"]:
                break
            rospy.sleep(0.01)
        return None

    def _publish(self, topic, value):
        self.publishers[topic.split("/")[-2]].publish(Float64(value))

    def move_joints(self, joints, stage, target_base_m=None):
        _, observation, _ = self.snapshot()
        planned = self.adapter.plan(joints, observation, rospy.Time.now().to_sec())
        if planned.status is not MotionStatus.READY or planned.plan is None:
            status, reason, max_error = "FAILED", planned.reason, None
        else:
            for projection in planned.plan.commands:
                for controller, value in zip(CONTROLLER_JOINT_NAMES, projection):
                    self.publishers[controller].publish(Float64(value))
                rospy.sleep(STEP_SECONDS)
            target_display = display_projection(joints)
            deadline = rospy.Time.now().to_sec() + TERMINAL_TIMEOUT_S
            stable = 0
            max_error = math.inf
            while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
                for controller, value in zip(CONTROLLER_JOINT_NAMES, target_display):
                    self.publishers[controller].publish(Float64(value))
                try:
                    message = rospy.wait_for_message(
                        "/swiftpro/joint_states", JointState, timeout=1.0
                    )
                except rospy.ROSException:
                    break
                measured = JointObservation(
                    tuple(message.name), tuple(message.position),
                    tuple(message.velocity), message.header.stamp.to_sec()
                )
                with self.lock:
                    self.joint = measured
                positions = dict(zip(measured.names, measured.positions_rad))
                max_error = max(
                    abs(positions[name] - target_display[index])
                    for name, index in zip(
                        DRIVEN_FEEDBACK_NAMES, DRIVEN_DISPLAY_INDICES
                    )
                )
                stable = stable + 1 if max_error <= TERMINAL_ERROR_RAD else 0
                if stable >= TERMINAL_STABLE_SAMPLES:
                    break
            status = "SETTLED" if stable >= TERMINAL_STABLE_SAMPLES else "FAILED"
            reason = status if status == "SETTLED" else "TERMINAL_NOT_ACCEPTED"
        print(json.dumps({
            "event": "CAMERA_PAYLOAD_PICK_STAGE",
            "payload": self.payload.payload,
            "stage": stage,
            "target_base_m": target_base_m,
            "status": status,
            "reason": reason,
            "max_driven_error_rad": max_error,
            "gazebo_control_authority": False,
        }), flush=True)
        return status == "SETTLED"

    def move(self, xyz, stage):
        joints = inverse_kinematics(xyz) if admission_reason(xyz) is None else None
        if joints is None:
            return False
        return self.move_joints(joints, stage, xyz)

    def home(self):
        return self.move_joints(
            tuple(self.config["home_independent_rad"]), "HOME", None
        )

    # PickTransactionBackend: the shared transaction names the guarantees it
    # needs; this SIM adapter owns ROS controller and feedback details.
    def move_and_settle(self, target_base_m, stage):
        return self.move(target_base_m, stage)

    def set_vacuum_verified(self, enabled):
        return self.vacuum(enabled)

    def home_and_settle(self):
        return self.home()

    def set_arm_extended(self, extended):
        self.extended.publish(Bool(extended))

    def wait_fresh_grasp(self, expected, after_s, timeout_s):
        deadline = rospy.Time.now().to_sec() + timeout_s
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
            value, stamp = self.snapshot()[2]
            if stamp > after_s and value is expected:
                return True
            rospy.sleep(0.02)
        return False

    def require_initial_not_grasping(self):
        now = rospy.Time.now().to_sec()
        value, stamp = self.snapshot()[2]
        return value is False and 0.0 <= now - stamp <= self.config["motion"]["target_age_s"]

    def vacuum(self, enabled):
        issued = rospy.Time.now().to_sec()
        prior_value, prior_stamp = self.snapshot()[2]
        prior_fresh_off = (
            not enabled and prior_value is False
            and 0.0 <= issued - prior_stamp
            <= self.config["motion"]["target_age_s"]
        )
        service = "/swiftpro/on" if enabled else "/swiftpro/off"
        try:
            rospy.wait_for_service(service, timeout=2.0)
            rospy.ServiceProxy(service, Empty)()
        except (rospy.ROSException, rospy.ServiceException):
            return False
        if prior_fresh_off:
            # The SIM vacuum plugin publishes state transitions, not a new Bool
            # when /off is idempotently called. A fresh False immediately before
            # an accepted /off call is therefore the strongest available proof;
            # release from True still requires a new post-command False below.
            return True
        return self.wait_fresh_grasp(
            enabled,
            issued,
            self.config["motion"]["vacuum_feedback_timeout_s"],
        )

    def wait_fresh_joint(self, timeout_s=3.0):
        deadline = rospy.Time.now().to_sec() + timeout_s
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
            _, observation, _ = self.snapshot()
            if (observation is not None
                    and 0.0 <= rospy.Time.now().to_sec() - observation.observed_at_s
                    <= self.adapter.config.thresholds["observation_max_age_s"]):
                return observation
            rospy.sleep(0.01)
        return None

    def preflight(self, cartesian_targets, observation=None):
        observation = observation or self.wait_fresh_joint()
        if observation is None or not independent_in_limits(
            tuple(self.config["home_independent_rad"])
        ):
            return False
        home_plan = self.adapter.plan(
            tuple(self.config["home_independent_rad"]),
            observation,
            rospy.Time.now().to_sec(),
        )
        if home_plan.status is not MotionStatus.READY:
            return False
        for item in cartesian_targets:
            target = item.target_base_m if isinstance(item, CartesianStage) else item[1]
            if admission_reason(target) is not None or inverse_kinematics(target) is None:
                return False
        return True

    def resume_held_rear_place(self):
        """Resume only a verified post-grasp rear-place SAFE_HOLD."""
        self.extended.publish(Bool(True))
        observed_after = rospy.Time.now().to_sec()
        if not self.wait_fresh_grasp(
                True, observed_after,
                self.config["motion"]["vacuum_feedback_timeout_s"]):
            rospy.logerr("SAFE_HOLD: no fresh held-payload evidence")
            return 1
        rear = self.config["rear_destinations"][self.payload.rear_slot]
        rear_place = tuple(rear["place_base_m"])
        rear_hover = tuple(rear["hover_base_m"])
        _, observation, _ = self.snapshot()
        if not self.preflight(
                (("REAR_PLACE_RECOVERY", rear_place),
                 ("REAR_RETRACT_RECOVERY", rear_hover)), observation):
            rospy.logerr("SAFE_HOLD: rear recovery preflight failed")
            return 1
        if not self.move(rear_place, "REAR_PLACE_RECOVERY"):
            rospy.logerr("SAFE_HOLD: rear recovery place failed; retaining vacuum")
            return 1
        if not self.vacuum(False):
            rospy.logerr("SAFE_HOLD: rear recovery release unverified")
            return 1
        if not self.move(rear_hover, "REAR_RETRACT_RECOVERY"):
            rospy.logerr("SAFE_HOLD: rear recovery retract failed")
            return 1
        if not self.home():
            rospy.logerr("SAFE_HOLD: rear recovery HOME failed")
            return 1
        self.extended.publish(Bool(False))
        print(json.dumps({
            "event": "REFEREE_HOLD",
            "payload": self.payload.payload,
            "rear_slot": self.payload.rear_slot,
            "status": "RECOVERED_REAR_PLACEMENT",
            "vacuum": False,
            "arm_extended": False,
        }), flush=True)
        return 0

    def run(self):
        # The 48-degree observation pose is already extended. Interlock base
        # motion before perception and establish a fresh vacuum-off generation
        # for this transaction rather than accepting an old latched Bool.
        self.extended.publish(Bool(True))
        if not self.vacuum(False):
            print(json.dumps({
                "event": "CAMERA_PAYLOAD_PICK_ABORT",
                "payload": self.payload.payload,
                "reason": "FRESH_INITIAL_VACUUM_OFF_UNVERIFIED",
                "motion_started": False,
            }), flush=True)
            return 1
        source = self.acquire_snapshot_source()
        if source is None:
            print(json.dumps({
                "event": "CAMERA_PAYLOAD_PICK_ABORT",
                "payload": self.payload.payload,
                "reason": "FRESH_SNAPSHOT_TARGET_UNAVAILABLE",
                "motion_started": False,
            }), flush=True)
            return 1

        pick, normal = source
        motion = self.config["motion"]
        rear = self.config["rear_destinations"][self.payload.rear_slot]
        plan = build_motion_plan(
            pick,
            normal,
            rear["hover_base_m"],
            rear["place_base_m"],
            motion["hover_m"],
            motion["lift_m"],
            motion["cartesian_waypoint_spacing_m"],
        )
        if plan is None:
            return 1
        result = execute_motion_plan(self, plan)
        print(json.dumps({
            "event": result.status,
            "payload": self.payload.payload,
            "rear_slot": self.payload.rear_slot,
            "status": result.reason,
            "vacuum_held": result.grasped,
            "arm_extended": result.arm_extended,
        }), flush=True)
        return 0 if result.status == "REFEREE_HOLD" else 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("payload")
    parser.add_argument("--config", required=True)
    parser.add_argument("--resume-held-rear-place", action="store_true")
    arguments = parser.parse_args(rospy.myargv()[1:])
    with open(arguments.config, encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)
    request = request_from_config(config, arguments.payload)
    if request is None:
        raise SystemExit("invalid payload configuration")
    rospy.init_node("swiftpro_sim_camera_payload_pick", anonymous=True)
    runner = CameraPayloadPickRunner(config, request)
    raise SystemExit(
        runner.resume_held_rear_place()
        if arguments.resume_held_rear_place else runner.run()
    )


if __name__ == "__main__":
    main()
