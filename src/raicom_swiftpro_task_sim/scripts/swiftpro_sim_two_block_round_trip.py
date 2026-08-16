#!/usr/bin/env python3
"""SIM-only, referee-gated single QR block rear-transfer round trip.

Each invocation moves exactly one runtime-spawned QR block: QR-01 from painted
slot 2 to rear slot 1 and camera slot 1, or QR-04 from painted slot 3 to rear
slot 2 and camera slot 4. Camera slots, joint feedback, and vacuum feedback are
control authority. Gazebo is read only after completed stages as SIM evidence.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import threading

import rospkg
import rospy
import tf
import yaml
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState, SpawnModel
from geometry_msgs.msg import PointStamped, Pose
from raicom_msgs.msg import WorkbenchSlots
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Empty

from raicom_swiftpro_task_sim.kinematics import (
    admission_reason, display_projection, forward_position, independent_in_limits,
    inverse_kinematics,
)
from raicom_swiftpro_task_sim.motion_adapter import (
    CONTROLLER_JOINT_NAMES, STEP_SECONDS, JointObservation, MotionStatus,
    SwiftproSimMotionAdapter,
)

SIM_ASSUMPTION = "SIM_WORKING_ASSUMPTION"
ARM_BASE_FRAME = "Base"
CAMERA_SLOT_FRAME = "hand_cam_link"
SIM_SPAWN_FRAME = "odom"  # SIM fixture contract: Gazebo world initially coincides with odom.
PAYLOADS = {
    "qr01": {"model": "qr_block_01", "source_slot": 2, "rear_slot": 1, "destination_slot": 1},
    "qr04": {"model": "qr_block_04", "source_slot": 3, "rear_slot": 2, "destination_slot": 4},
}
OBSERVATION_JOINTS_RAD = {"Joint1": math.radians(-42.0), "Joint2": 0.0, "Joint6": 0.0}
OBSERVATION_TOLERANCE_RAD = 0.03
MAX_SLOT_AGE_S = 0.5
TERMINAL_TIMEOUT_S = 6.0
TERMINAL_ERROR_RAD = 0.02
TERMINAL_STABLE_SAMPLES = 3
DRIVEN_FEEDBACK_NAMES = ("Joint1", "Joint2", "Joint6")
DRIVEN_DISPLAY_INDICES = (0, 1, 5)
HOME_INDEPENDENT_RAD = (math.pi / 2.0, math.pi / 2.0, math.radians(3.8))


def render_payload_sdf(model, block):
    """Render the narrow runtime payload without importing another task package."""
    size = float(block["size_xy_m"])
    height = float(block["thickness_m"])
    mass = float(block["mass_kg"])
    inertia_xy = mass * (size * size + height * height) / 12.0
    inertia_z = mass * size * size / 6.0
    color = "0.15 0.35 0.85 1" if model == "qr_block_01" else "0.85 0.25 0.15 1"
    return ("<sdf version='1.6'><model name='%s'><static>false</static><link name='link'>"
            "<inertial><mass>%.9g</mass><inertia><ixx>%.9g</ixx><iyy>%.9g</iyy>"
            "<izz>%.9g</izz></inertia></inertial><collision name='collision'><geometry>"
            "<box><size>%.9g %.9g %.9g</size></box></geometry></collision>"
            "<visual name='body'><geometry><box><size>%.9g %.9g %.9g</size></box>"
            "</geometry><material><ambient>%s</ambient><diffuse>%s</diffuse></material>"
            "</visual></link></model></sdf>" %
            (model, mass, inertia_xy, inertia_xy, inertia_z,
             size, size, height, size, size, height, color, color))


def load_contract(path):
    with open(path, encoding="utf-8") as stream:
        contract = yaml.safe_load(stream)
    if not isinstance(contract, dict) or contract.get("assumption_status") != SIM_ASSUMPTION:
        raise ValueError("two-block contract must be a SIM_WORKING_ASSUMPTION")
    if contract["block"]["size_xy_m"] != 0.040 or contract["block"]["thickness_m"] != 0.010:
        raise ValueError("unexpected QR plan size or SIM thickness")
    return contract


class RefereeRoundTrip:
    def __init__(self, contract, payload, use_existing=False, rear_only=False, rear_slot=None):
        self.contract = contract
        self.payload = payload
        self.use_existing = use_existing
        self.rear_only = rear_only
        self.rear_slot = rear_slot or payload["rear_slot"]
        self.lock = threading.Lock()
        self.joint = self.slots = self.grasping = None
        self.tf_listener = tf.TransformListener()
        self.adapter = SwiftproSimMotionAdapter()
        self.publishers = {name: rospy.Publisher("/swiftpro/%s_position_controller/command" % name, Float64, queue_size=1)
                           for name in CONTROLLER_JOINT_NAMES}
        self.arm_extended = rospy.Publisher("/raicom/safety/arm_extended", Bool, queue_size=1, latch=True)
        rospy.Subscriber("/swiftpro/joint_states", JointState, self.on_joint, queue_size=1)
        rospy.Subscriber("/raicom/workbench/slots", WorkbenchSlots, self.on_slots, queue_size=1)
        rospy.Subscriber("/swiftpro/grasping", Bool, self.on_grasping, queue_size=1)

    def emit(self, event, **fields):
        print(json.dumps({"event": event, "sim_only": True, "payload": self.payload["model"], **fields}, sort_keys=True), flush=True)

    def on_joint(self, message):
        with self.lock:
            self.joint = JointObservation(tuple(message.name), tuple(message.position), tuple(message.velocity), message.header.stamp.to_sec())

    def on_slots(self, message):
        with self.lock:
            self.slots = message

    def on_grasping(self, message):
        with self.lock:
            self.grasping = bool(message.data)

    def snapshot(self):
        with self.lock:
            return self.joint, self.slots, self.grasping

    def observation_pose(self, joint):
        if joint is None:
            return False
        positions = dict(zip(joint.names, joint.positions_rad))
        return all(name in positions and abs(positions[name] - expected) <= OBSERVATION_TOLERANCE_RAD
                   for name, expected in OBSERVATION_JOINTS_RAD.items())

    def wait_inputs(self):
        deadline = rospy.Time.now().to_sec() + 5.0
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
            if all(value is not None for value in self.snapshot()):
                return True
            rospy.sleep(0.02)
        return False

    def wait_connections(self):
        deadline = rospy.Time.now().to_sec() + 3.0
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
            if all(publisher.get_num_connections() for publisher in self.publishers.values()):
                return True
            rospy.sleep(0.02)
        return False

    def publish_projection(self, projection):
        for controller, value in zip(CONTROLLER_JOINT_NAMES, projection):
            self.publishers[controller].publish(Float64(data=value))

    def feedback_fk(self):
        joint, _, _ = self.snapshot()
        if joint is None:
            return None
        positions = dict(zip(joint.names, joint.positions_rad))
        try:
            return forward_position((positions["Joint1"] + math.pi / 2.0,
                                     math.pi / 2.0 - positions["Joint2"],
                                     positions["Joint6"] + math.radians(3.8)))
        except KeyError:
            return None

    def oracle(self, stage):
        """Gazebo is deliberately observed only after a completed stage."""
        try:
            models = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=2.0)
        except rospy.ROSException:
            self.emit("GAZEBO_ORACLE", stage=stage, available=False)
            return
        model = self.payload["model"]
        observed = None
        if model in models.name:
            pose = models.pose[models.name.index(model)]
            observed = [pose.position.x, pose.position.y, pose.position.z]
        self.emit("GAZEBO_ORACLE", stage=stage, available=True, model_position_world_m=observed)

    def move_joints(self, joints, stage, target_base_m=None):
        joint, _, _ = self.snapshot()
        planned = self.adapter.plan(joints, joint, rospy.Time.now().to_sec())
        if planned.status is not MotionStatus.READY or planned.plan is None:
            self.emit("STAGE", stage=stage, status="FAILED", reason=planned.reason,
                      target_base_m=target_base_m, vacuum=self.snapshot()[2])
            return False
        for projection in planned.plan.commands:
            self.publish_projection(projection)
            rospy.sleep(STEP_SECONDS)
        target_display = display_projection(joints)
        deadline = rospy.Time.now().to_sec() + TERMINAL_TIMEOUT_S
        stable = 0
        max_error = math.inf
        measured = joint
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
            self.publish_projection(target_display)
            try:
                message = rospy.wait_for_message("/swiftpro/joint_states", JointState, timeout=1.0)
            except rospy.ROSException:
                break
            measured = JointObservation(tuple(message.name), tuple(message.position),
                                        tuple(message.velocity), message.header.stamp.to_sec())
            positions = dict(zip(measured.names, measured.positions_rad))
            max_error = max(abs(positions[name] - target_display[index])
                            for name, index in zip(DRIVEN_FEEDBACK_NAMES, DRIVEN_DISPLAY_INDICES))
            stable = stable + 1 if max_error <= TERMINAL_ERROR_RAD else 0
            if stable >= TERMINAL_STABLE_SAMPLES:
                break
        status = "SETTLED" if stable >= TERMINAL_STABLE_SAMPLES else "FAILED"
        reason = "SETTLED" if status == "SETTLED" else "TERMINAL_NOT_ACCEPTED"
        self.emit("STAGE", stage=stage, status=status, reason=reason,
                  target_base_m=target_base_m, feedback_fk_base_m=self.feedback_fk(),
                  max_error_rad=max_error, vacuum=self.snapshot()[2])
        self.oracle(stage)
        return status == "SETTLED"

    def move(self, target_base_m, stage):
        reason = admission_reason(target_base_m)
        joints = inverse_kinematics(target_base_m) if reason is None else None
        if joints is None:
            self.emit("STAGE", stage=stage, status="REJECTED", reason=reason or "IK_FAILED", target_base_m=target_base_m)
            return False
        return self.move_joints(joints, stage, target_base_m)

    def set_vacuum(self, enabled, stage):
        service = "/swiftpro/on" if enabled else "/swiftpro/off"
        try:
            rospy.wait_for_service(service, timeout=self.contract["motion"]["service_timeout_s"])
            rospy.ServiceProxy(service, Empty)()
        except (rospy.ROSException, rospy.ServiceException) as error:
            self.emit("STAGE", stage=stage, status="FAILED", reason="VACUUM_SERVICE", detail=str(error))
            return False
        expected = bool(enabled)
        deadline = rospy.Time.now().to_sec() + self.contract["motion"]["vacuum_feedback_timeout_s"]
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
            if self.snapshot()[2] is expected:
                self.emit("STAGE", stage=stage, status="SETTLED", vacuum=expected, feedback_fk_base_m=self.feedback_fk())
                self.oracle(stage)
                return True
            rospy.sleep(0.02)
        self.emit("STAGE", stage=stage, status="FAILED", reason="VACUUM_FEEDBACK_TIMEOUT", vacuum=self.snapshot()[2])
        self.oracle(stage)
        return False

    @staticmethod
    def normal(slot):
        x, y, z, w = slot.orientation.x, slot.orientation.y, slot.orientation.z, slot.orientation.w
        normal = (2.0 * (x * z + w * y), 2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + y * y))
        magnitude = math.sqrt(sum(value * value for value in normal))
        if not math.isfinite(magnitude) or magnitude < 0.99 or magnitude > 1.01 or normal[2] > -0.5:
            return None
        return normal

    def camera_point_to_base(self, slots, slot_id, offset_m):
        slot = slots.slots[slot_id - 1]
        normal = self.normal(slot)
        if normal is None:
            return None
        point = PointStamped(); point.header.frame_id = slots.header.frame_id; point.header.stamp = rospy.Time(0)
        point.point.x = slot.position.x + normal[0] * offset_m
        point.point.y = slot.position.y + normal[1] * offset_m
        point.point.z = slot.position.z + normal[2] * offset_m
        try:
            self.tf_listener.waitForTransform(ARM_BASE_FRAME, CAMERA_SLOT_FRAME, rospy.Time(0), rospy.Duration(2.0))
            transformed = self.tf_listener.transformPoint(ARM_BASE_FRAME, point)
        except Exception:
            return None
        return (transformed.point.x, transformed.point.y, transformed.point.z)

    def camera_point_to_spawn_frame(self, slots, slot_id, offset_m):
        slot = slots.slots[slot_id - 1]
        normal = self.normal(slot)
        if normal is None:
            return None
        point = PointStamped(); point.header.frame_id = slots.header.frame_id; point.header.stamp = rospy.Time(0)
        point.point.x = slot.position.x + normal[0] * offset_m
        point.point.y = slot.position.y + normal[1] * offset_m
        point.point.z = slot.position.z + normal[2] * offset_m
        try:
            self.tf_listener.waitForTransform(SIM_SPAWN_FRAME, CAMERA_SLOT_FRAME, rospy.Time(0), rospy.Duration(2.0))
            transformed = self.tf_listener.transformPoint(SIM_SPAWN_FRAME, point)
        except Exception:
            return None
        return transformed

    def frozen_plan(self):
        joint, slots, grasping = self.snapshot()
        now = rospy.Time.now().to_sec()
        if (joint is None or slots is None or grasping is not False or not self.observation_pose(joint)
                or not slots.valid or slots.header.frame_id != CAMERA_SLOT_FRAME or len(slots.slots) != 4
                or not 0.0 <= now - slots.header.stamp.to_sec() <= MAX_SLOT_AGE_S):
            return None
        physical_clearance = self.contract["motion"]["physical_tcp_clearance_m"]
        source = self.camera_point_to_base(slots, self.payload["source_slot"], physical_clearance)
        destination = self.camera_point_to_base(slots, self.payload["destination_slot"], physical_clearance)
        if source is None or destination is None:
            return None
        rear_hover_by_slot = {
            1: self.contract["rear_transfer"]["hover_base_m"]["qr_block_01"],
            2: self.contract["rear_transfer"]["hover_base_m"]["qr_block_04"],
        }
        rear_hover = rear_hover_by_slot[self.rear_slot]
        rear_place = (rear_hover[0], rear_hover[1], self.contract["rear_transfer"]["place_z_m"])
        rear_pick = (rear_hover[0], rear_hover[1], self.contract["rear_transfer"]["pickup_z_m"])
        source_lift = (source[0], source[1], source[2] + self.contract["motion"]["source_lift_m"])
        destination_hover = (destination[0], destination[1], destination[2] + self.contract["motion"]["destination_hover_m"])
        cartesian = {
            "source_pick": source, "source_lift": source_lift, "rear_hover": rear_hover,
            "rear_place": rear_place, "rear_pick": rear_pick, "destination_hover": destination_hover,
            "destination_place": destination,
        }
        rejected = {name: admission_reason(target) for name, target in cartesian.items() if admission_reason(target) is not None}
        # Home is a configured joint pose rather than a Cartesian target; its
        # explicit joint-limit check is the applicable preflight gate.
        home_fk = forward_position(HOME_INDEPENDENT_RAD)
        home_reason = None if independent_in_limits(HOME_INDEPENDENT_RAD) else "HOME_JOINT_LIMIT"
        if home_fk is None:
            home_reason = "HOME_FK_INVALID"
        if rejected or home_reason:
            self.emit("WHOLE_SEQUENCE_PREFLIGHT", status="REJECTED", rejected=rejected, home_reason=home_reason)
            return None
        spawn = self.camera_point_to_spawn_frame(slots, self.payload["source_slot"], self.contract["block"]["thickness_m"] / 2.0)
        if spawn is None:
            self.emit("WHOLE_SEQUENCE_PREFLIGHT", status="REJECTED", rejected={"spawn": "WORLD_TF_UNAVAILABLE"})
            return None
        self.emit("FROZEN_CAMERA_SLOTS", stamp_s=slots.header.stamp.to_sec(), frame_id=slots.header.frame_id,
                  stable_frames=slots.stable_frames, source_slot=self.payload["source_slot"],
                  destination_slot=self.payload["destination_slot"], plan_base_m=cartesian,
                  spawn_frame=SIM_SPAWN_FRAME,
                  spawn_fixture_xyz_m=[spawn.point.x, spawn.point.y, spawn.point.z], home_fk_base_m=home_fk)
        self.emit("WHOLE_SEQUENCE_PREFLIGHT", status="SETTLED", checked_targets=list(cartesian), home_independent_rad=HOME_INDEPENDENT_RAD)
        return cartesian, spawn

    def spawn_source(self, spawn_point):
        model = self.payload["model"]
        # Gazebo's spawn service enforces model-name uniqueness.  Do not read
        # model state before control begins; model states remain post-stage oracle evidence.
        pose = Pose(); pose.position.x, pose.position.y, pose.position.z = spawn_point.point.x, spawn_point.point.y, spawn_point.point.z
        pose.orientation.w = 1.0
        try:
            rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=2.0)
            response = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)(
                model, render_payload_sdf(model, self.contract["block"]), "", pose, "world")
        except (rospy.ROSException, rospy.ServiceException) as error:
            self.emit("SPAWN", status="FAILED", model=model, detail=str(error))
            return False
        if not response.success:
            reason = "MODEL_NAME_CONFLICT" if "exist" in response.status_message.lower() else "SPAWN_FAILED"
            self.emit("SPAWN", status="FAILED", reason=reason, model=model, detail=response.status_message)
            return False
        self.emit("SPAWN", status="SETTLED", model=model, runtime_only=True)
        self.oracle("SPAWN")
        return True

    def existing_payload_ready(self):
        try:
            rospy.wait_for_service("/gazebo/get_model_state", timeout=2.0)
            response = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)(
                self.payload["model"], "world")
        except (rospy.ROSException, rospy.ServiceException) as error:
            self.emit("EXISTING_PAYLOAD", status="REJECTED", detail=str(error))
            return False
        status = "SETTLED" if response.success else "REJECTED"
        self.emit("EXISTING_PAYLOAD", status=status, rear_slot=self.rear_slot)
        return response.success

    def run_rear_piece(self, plan):
        model = self.payload["model"]
        for stage, target in (("TRANSFER_HOVER", "rear_hover"),
                              ("TRANSFER_PICK_UNVERIFIED_ASSUMPTION", "rear_pick")):
            if not self.move(plan[target], "%s_%s" % (model, stage)): return False
        if not self.set_vacuum(True, "%s_TRANSFER_VACUUM_ON" % model): return False
        for stage, target in (("TRANSFER_RETRACT_LOADED", "rear_hover"),
                              ("DESTINATION_HOVER", "destination_hover"),
                              ("DESTINATION_PLACE", "destination_place")):
            if not self.move(plan[target], "%s_%s" % (model, stage)): return False
        if not self.set_vacuum(False, "%s_DESTINATION_VACUUM_OFF" % model): return False
        return self.move(plan["destination_hover"], "%s_DESTINATION_RETRACT" % model)

    def run_piece(self, plan):
        model = self.payload["model"]
        stages = (("SOURCE_PICK", "source_pick"),)
        for stage, target in stages:
            if not self.move(plan[target], "%s_%s" % (model, stage)): return False
        if not self.set_vacuum(True, "%s_SOURCE_VACUUM_ON" % model): return False
        # Explicit vertical lift before any lateral rear-transfer command.
        for stage, target in (("SOURCE_VERTICAL_LIFT", "source_lift"), ("TRANSFER_HOVER", "rear_hover"), ("TRANSFER_PLACE", "rear_place")):
            if not self.move(plan[target], "%s_%s" % (model, stage)): return False
        if not self.set_vacuum(False, "%s_TRANSFER_VACUUM_OFF" % model): return False
        for stage, target in (("TRANSFER_RETRACT", "rear_hover"), ("TRANSFER_PICK_UNVERIFIED_ASSUMPTION", "rear_pick")):
            if not self.move(plan[target], "%s_%s" % (model, stage)): return False
        if not self.set_vacuum(True, "%s_TRANSFER_VACUUM_ON" % model): return False
        for stage, target in (("TRANSFER_RETRACT_LOADED", "rear_hover"), ("DESTINATION_HOVER", "destination_hover"), ("DESTINATION_PLACE", "destination_place")):
            if not self.move(plan[target], "%s_%s" % (model, stage)): return False
        if not self.set_vacuum(False, "%s_DESTINATION_VACUUM_OFF" % model): return False
        return self.move(plan["destination_hover"], "%s_DESTINATION_RETRACT" % model)

    def finish(self, success):
        vacuum_off = self.set_vacuum(False, "CLEANUP_VACUUM_OFF")
        home = self.move_joints(HOME_INDEPENDENT_RAD, "HOME")
        if home:
            self.arm_extended.publish(Bool(data=False))
        success = success and vacuum_off and home
        self.emit("REFEREE_HOLD", status="READY_FOR_REFEREE" if success else "ABORTED", arm_extended=not home,
                  vacuum=self.snapshot()[2], rear_slot=self.rear_slot, destination_slot=self.payload["destination_slot"])
        return 0 if success else 2

    def run(self):
        if rospy.get_param("/raicom/backend", "") != "SIM":
            self.emit("RESULT", status="REJECTED", reason="REAL_BACKEND")
            return 2
        if not self.wait_inputs() or not self.wait_connections():
            self.emit("RESULT", status="REJECTED", reason="INPUT_OR_CONTROLLER_TIMEOUT")
            return 2
        frozen = self.frozen_plan()
        if frozen is None:
            self.emit("RESULT", status="REJECTED", reason="WHOLE_SEQUENCE_PREFLIGHT_FAILED")
            return 2
        plan, spawn_point = frozen
        if self.use_existing:
            if not self.existing_payload_ready():
                return 2
        elif not self.spawn_source(spawn_point):
            return 2
        self.arm_extended.publish(Bool(data=True))
        action = self.run_rear_piece(plan) if self.rear_only else self.run_piece(plan)
        return self.finish(action)


def main():
    parser = argparse.ArgumentParser(description="Run exactly one SIM referee QR transfer piece.")
    parser.add_argument("--payload", required=True, choices=tuple(PAYLOADS))
    parser.add_argument("--use-existing", action="store_true")
    parser.add_argument("--rear-only", action="store_true")
    parser.add_argument("--rear-slot", type=int, choices=(1, 2))
    args = parser.parse_args(rospy.myargv()[1:])
    if args.rear_only and not args.use_existing:
        parser.error("--rear-only requires --use-existing")
    rospy.init_node("swiftpro_sim_two_block_round_trip", anonymous=True)
    path = os.path.join(rospkg.RosPack().get_path("raicom_swiftpro_task_sim"), "config", "two_block_round_trip.yaml")
    try:
        raise SystemExit(RefereeRoundTrip(load_contract(path), PAYLOADS[args.payload],
                                          args.use_existing, args.rear_only,
                                          args.rear_slot).run())
    except (KeyError, TypeError, ValueError, rospy.ROSException) as error:
        print(json.dumps({"event": "RESULT", "status": "REJECTED", "reason": "CONFIG_OR_RUNTIME", "detail": str(error)}), flush=True)
        raise SystemExit(2)


if __name__ == "__main__":
    main()
