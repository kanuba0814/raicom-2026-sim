#!/usr/bin/env python3
"""Narrow SIM_ONLY guarded docking adapter. Never publishes /cmd_vel."""
import json
import math
import threading
import xmlrpc.client

import rosgraph
import rospy
import tf2_geometry_msgs  # Registers PointStamped transforms with tf2_ros.
import tf2_ros
from geometry_msgs.msg import PointStamped, Twist, Vector3Stamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from raicom_round_table_navigation.boundary_guard import Footprint, guard_footprint, guard_rotation_sweep
from raicom_round_table_navigation.docking_adapter import (
    AdmissionSnapshot, FaultLatch, ObservationDropoutGate,
    SIM_WORKING_ASSUMPTION_CLEARANCE_UNCERTAINTY_M,
    angular_stopping_horizon_s, base_velocity_to_map, canonical_occupancy_grid_sha256,
    map_grid_from_message, safe_suggestion, validate_admission,
)
from raicom_round_table_navigation.docking_geometry import DockingConfig, DockingGeometry


class DockingAdapter:
    def __init__(self):
        self.team = rospy.get_param("~team")
        self.selected_team = rospy.get_param("~selected_team")
        self.expected_map_image = rospy.get_param("~expected_map_image")
        self.expected_grid_hash = rospy.get_param("~expected_grid_hash")
        self.enabled = rospy.get_param("~enabled", False)
        self.timeout = rospy.Duration(rospy.get_param("~data_timeout_s", 0.5))
        self.pair_timeout = rospy.Duration(rospy.get_param("~valid_pair_timeout_s", 0.2))
        self.sync_timeout = rospy.Duration(rospy.get_param("~observation_scan_sync_s", 0.1))
        self.map_tf_skew_s = rospy.get_param("~map_tf_max_skew_s", 0.75)
        self.phase_timeout_s = rospy.get_param("~phase_timeout_s", 20.0)
        self.rotation_reaction_s = rospy.get_param("~rotation_reaction_time_s", 0.2)
        self.rotation_deceleration_rps2 = rospy.get_param("~rotation_deceleration_rps2", 0.4)
        self.identity_recorded = bool(rospy.get_param("~map_identity_contract_recorded", False))
        uncertainty = rospy.get_param("~clearance_uncertainty_m", SIM_WORKING_ASSUMPTION_CLEARANCE_UNCERTAINTY_M)
        if uncertainty < SIM_WORKING_ASSUMPTION_CLEARANCE_UNCERTAINTY_M:
            raise rospy.ROSException("clearance uncertainty cannot weaken SIM_WORKING_ASSUMPTION")
        if (self.rotation_reaction_s < 0.0 or self.rotation_deceleration_rps2 <= 0.0
                or self.map_tf_skew_s <= 0.0):
            raise rospy.ROSException("invalid angular stopping or map TF timing model")
        self.geometry = DockingGeometry(DockingConfig(
            target_front_gap_m=rospy.get_param("~target_front_gap_m", 0.03),
            completion_gap_min_m=rospy.get_param("~completion_gap_min_m", 0.02),
            completion_gap_max_m=rospy.get_param("~completion_gap_max_m", 0.04),
            hard_minimum_gap_m=rospy.get_param("~hard_minimum_gap_m", 0.01)))
        self.uncertainty = uncertainty
        self.footprint = Footprint(0.24, 0.24, 0.20, 0.20)
        self.tf = tf2_ros.Buffer(cache_time=rospy.Duration(5))
        self.listener = tf2_ros.TransformListener(self.tf)
        self.circle = self.valid = self.grid_message = self.scan = None
        self.circle_received = self.valid_received = self.map_received = self.scan_received = rospy.Time(0)
        self.sensor_lock = threading.Lock()
        self.latch = FaultLatch()
        self.observation_gate = ObservationDropoutGate(latch_after_frames=4, recover_after_frames=3)
        self.phase_started = rospy.Time.now()
        self.requalification_pending = False
        self.requalification_zeros = 0
        self.enable_seen_after_ack = False
        self.was_enabled = self.enabled
        self.command_publisher = rospy.Publisher("/raicom/cmd_vel/relative", Twist, queue_size=1)
        self.estop_publisher = rospy.Publisher("/raicom/safety/soft_estop", Bool, queue_size=1, latch=True)
        self.status_publisher = rospy.Publisher("/raicom/round_table_docking/status", String, queue_size=1, latch=True)
        rospy.Subscriber("/round_table/circle", Vector3Stamped, self._circle, queue_size=1)
        rospy.Subscriber("/round_table/valid", Bool, self._valid, queue_size=1)
        rospy.Subscriber("/raicom/team_boundary_mask", OccupancyGrid, self._map, queue_size=1)
        rospy.Subscriber(rospy.get_param("~scan_topic", "/fuse_scan"), LaserScan, self._scan, queue_size=1)
        rospy.Subscriber("/raicom/round_table_docking/ack", Bool, self._ack, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self._tick)

    def _circle(self, msg):
        with self.sensor_lock:
            self.circle, self.circle_received = msg, rospy.Time.now()

    def _valid(self, msg):
        with self.sensor_lock:
            self.valid, self.valid_received = msg, rospy.Time.now()

    def _map(self, msg):
        with self.sensor_lock:
            self.grid_message, self.map_received = msg, rospy.Time.now()

    def _scan(self, msg):
        with self.sensor_lock:
            self.scan, self.scan_received = msg, rospy.Time.now()

    def _sensor_snapshot(self):
        with self.sensor_lock:
            return (self.circle, self.circle_received, self.valid, self.valid_received,
                    self.grid_message, self.map_received, self.scan, self.scan_received)

    def _ack(self, msg):
        # Acknowledgement is deliberately accepted only while motion is disabled.
        if msg.data and not rospy.get_param("~enabled", False):
            self.latch.reset()
            self.geometry.acknowledge()
            self.observation_gate.reset()
            self.requalification_pending = True
            self.requalification_zeros = 0
            self.enable_seen_after_ack = False
            # Disabled dwell time must never consume a future motion phase's
            # budget. The enabled edge below anchors it again after requalification.
            self.phase_started = rospy.Time.now()

    @staticmethod
    def _fresh(received, timeout, now):
        return received != rospy.Time(0) and now >= received and now - received <= timeout

    @staticmethod
    def _header_fresh(message, timeout, now):
        if message is None or message.header.stamp == rospy.Time(0):
            return False
        age = now - message.header.stamp
        return rospy.Duration(0) <= age <= timeout

    def _paired(self, valid, valid_received, circle, circle_received):
        # Bool has no header; receive-time pairing is the explicit detector
        # contract. All four values come from one locked callback snapshot.
        return (valid is not None and circle is not None and
                abs((valid_received - circle_received).to_sec()) <= self.pair_timeout.to_sec())

    @staticmethod
    def _circle_is_valid(circle):
        if circle is None:
            return False
        v = circle.vector
        return all(math.isfinite(value) for value in (v.x, v.y, v.z)) and 0.12 <= v.z <= 0.40

    def _publish_zero(self, reason, latch=False, state=None):
        self.command_publisher.publish(Twist())
        if latch:
            self.latch.fault(reason)
        self.estop_publisher.publish(Bool(self.latch.latched))
        now = rospy.Time.now()
        _, circle_received, _, valid_received, _, map_received, _, scan_received = self._sensor_snapshot()
        def age(received):
            return None if received == rospy.Time(0) else max(0.0, (now - received).to_sec())
        self.status_publisher.publish(String(json.dumps({"state": state or ("SOFT_ESTOP" if self.latch.latched else "STOP"),
            "reason": reason, "phase": self.geometry.phase.value, "team": self.team, "selected_team": self.selected_team,
            "expected_map_image": self.expected_map_image, "expected_grid_hash": self.expected_grid_hash,
            "map_identity_source": "canonical_occupancy_grid_sha256",
            "team_boundary_enforced": True, "simulation_only": True,
            "callback_age_s": {"circle": age(circle_received), "valid": age(valid_received),
                               "scan": age(scan_received), "map": age(map_received)}})))

    def _publishers(self, topic):
        try:
            code, _, state = xmlrpc.client.ServerProxy(rosgraph.get_master_uri()).getSystemState(rospy.get_name())
            if code != 1: return None
            # ROS master getSystemState() is [publishers, subscribers, services].
            # Admission must inspect publishers, never subscribers.
            return {node for name, nodes in state[0] if name == topic for node in nodes}
        except Exception:
            return None

    def _admission(self, snapshot, map_valid, map_identity_matches, tf_fresh, valid_now, now):
        cmd_publishers, relative_publishers, nav_publishers, map_publishers = (self._publishers(name) for name in
            ("/cmd_vel", "/raicom/cmd_vel/relative", "/raicom/cmd_vel/nav", "/raicom/team_boundary_mask"))
        try:
            code, _, state = xmlrpc.client.ServerProxy(rosgraph.get_master_uri()).getSystemState(rospy.get_name())
            all_nodes = ({node for _, nodes in state[0] for node in nodes} | {node for _, nodes in state[1] for node in nodes} |
                         {node for _, nodes in state[2] for node in nodes}) if code == 1 else None
        except Exception:
            all_nodes = None
        circle, circle_received, valid, valid_received, _grid, map_received, scan, scan_received = snapshot
        scan_stamp_fresh = self._header_fresh(scan, self.timeout, now)
        circle_stamp_fresh = self._header_fresh(circle, self.timeout, now)
        circle_scan_synced = (scan_stamp_fresh and circle_stamp_fresh and
            abs((circle.header.stamp - scan.header.stamp).to_sec()) <= self.sync_timeout.to_sec())
        return validate_admission(AdmissionSnapshot(
            self.enabled, rospy.get_param("/raicom/backend", ""), self.team if self.selected_team == self.team else "",
            self.expected_map_image, self.expected_grid_hash, self.identity_recorded, map_identity_matches,
            self._fresh(circle_received, self.timeout, now) and self._circle_is_valid(circle) and circle_scan_synced,
            circle_stamp_fresh, valid_now,
            self._paired(valid, valid_received, circle, circle_received),
            self._fresh(scan_received, self.timeout, now), scan_stamp_fresh,
            map_received != rospy.Time(0) and map_publishers == {"/raicom_team_boundary_map_server"},
            tf_fresh, map_valid,
            cmd_publishers == {"/raicom_cmd_vel_gate"}, relative_publishers == {rospy.get_name()},
            all_nodes is not None and "/move_base" not in all_nodes, nav_publishers == set()))

    def _base_observation_and_pose(self, circle):
        try:
            point = PointStamped()
            point.header = circle.header
            point.point.x, point.point.y, point.point.z = circle.vector.x, circle.vector.y, 0.0
            transformed = self.tf.transform(point, "base_footprint", rospy.Duration(0.05))
            # AMCL intentionally broadcasts map->odom ahead by its transform
            # tolerance. Use the latest map pose, but admit it only when its
            # timestamp remains tightly paired with this scan observation.
            pose = self.tf.lookup_transform("map", "base_footprint", rospy.Time(0), rospy.Duration(0.05))
            if pose.header.stamp == rospy.Time(0) or abs(
                    (pose.header.stamp - circle.header.stamp).to_sec()) > self.map_tf_skew_s:
                return None
            q = pose.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return transformed.point.x, transformed.point.y, circle.vector.z, pose.transform.translation.x, pose.transform.translation.y, yaw, True
        except Exception:
            return None

    def _tick(self, _event):
        self.enabled = rospy.get_param("~enabled", False)
        if not self.enabled:
            self.was_enabled = False
            return self._publish_zero("DISABLED")
        if not self.was_enabled:
            # Start every enabled session with a fresh bounded phase budget;
            # time spent inspecting the node while disabled is irrelevant.
            self.phase_started = rospy.Time.now()
            self.was_enabled = True
        if self.requalification_pending and not self.enable_seen_after_ack:
            # A disabled-only ack cannot be followed by motion in the same tick.
            self.enable_seen_after_ack = True
            self.was_enabled = True
            return self._publish_zero("REQUALIFICATION_ENABLE_REQUIRED")
        snapshot = self._sensor_snapshot()
        circle, circle_received, valid, valid_received, grid_message, _map_received, _scan, _scan_received = snapshot
        now = rospy.Time.now()
        paired = self._paired(valid, valid_received, circle, circle_received)
        valid_now = (valid is not None and self._fresh(valid_received, self.timeout, now)
                     and valid.data and paired)
        observation_action = self.observation_gate.update(valid_now)
        if observation_action == "HOLD_ZERO":
            return self._publish_zero("OBSERVATION_REQUALIFICATION_ZERO", state="UNCERTAIN")
        if observation_action == "LATCH":
            return self._publish_zero("OBSERVATION_CONSECUTIVE_INVALID", latch=True)
        grid = map_grid_from_message(grid_message) if grid_message else None
        actual_hash = canonical_occupancy_grid_sha256(grid_message) if grid_message else None
        values = self._base_observation_and_pose(circle) if circle else None
        reason = self._admission(snapshot, grid is not None, actual_hash == self.expected_grid_hash,
                                 values is not None and values[-1], valid_now, now)
        if self.latch.latched:
            return self._publish_zero("SOFT_ESTOP_LATCHED")
        if self.requalification_pending:
            # Three full enabled control samples are intentionally zero-only.
            self.requalification_zeros += 1
            self.was_enabled = True
            if self.requalification_zeros < 3:
                return self._publish_zero("REQUALIFICATION_ZERO_WINDOW")
            self.requalification_pending = False
        else:
            self.was_enabled = True
        if reason != "ADMITTED":
            return self._publish_zero(reason, latch=True)
        cx, cy, radius, px, py, yaw, _tf_fresh = values
        if rospy.Time.now() - self.phase_started > rospy.Duration(self.phase_timeout_s):
            return self._publish_zero("PHASE_TIMEOUT", latch=True)
        previous_phase = self.geometry.phase
        decision = self.geometry.decide(cx, cy, radius, True, self.uncertainty)
        if decision.phase != previous_phase:
            self.phase_started = rospy.Time.now()
        okay, command_reason = safe_suggestion(decision.suggestion, decision.phase)
        if not okay or decision.phase.value == "STOP":
            return self._publish_zero(command_reason if not okay else decision.reason, latch=True)
        command = decision.suggestion
        # Apply only the matching guard. ALIGN is rotation-only; APPROACH is
        # forward-only, with full-footprint stopping sweep in the team mask.
        if command.vx_mps > 0.0:
            vx_map, vy_map = base_velocity_to_map(command.vx_mps, 0.0, yaw)
            translation = guard_footprint(
                grid, self.footprint, px, py, yaw, vx_map, vy_map,
                reaction_time_s=0.2, deceleration_mps2=0.3)
            if not translation.safe:
                return self._publish_zero("TRANSLATION_GUARD_" + translation.reason, state="BOUNDARY_LIMIT")
        if command.wz_rps != 0.0:
            angular_stop_s = angular_stopping_horizon_s(
                command.wz_rps, self.rotation_reaction_s, self.rotation_deceleration_rps2)
            rotation = guard_rotation_sweep(
                grid, self.footprint, px, py, yaw,
                yaw + command.wz_rps * angular_stop_s)
            if not rotation.safe:
                return self._publish_zero("ROTATION_GUARD_" + rotation.reason, state="BOUNDARY_LIMIT")
        output = Twist(); output.linear.x = command.vx_mps; output.linear.y = command.vy_mps; output.angular.z = command.wz_rps
        self.command_publisher.publish(output)
        self.estop_publisher.publish(Bool(False))
        self.status_publisher.publish(String(json.dumps({"state": "ACTIVE", "phase": decision.phase.value, "reason": decision.reason,
            "team": self.team, "selected_team": self.selected_team, "expected_map_image": self.expected_map_image,
            "expected_grid_hash": self.expected_grid_hash, "actual_grid_hash": actual_hash,
            "map_identity_source": "canonical_occupancy_grid_sha256",
            "team_boundary_enforced": True, "simulation_only": True,
            "radial_observation": {"cx_m": cx, "cy_m": cy, "radius_m": radius,
                                   "face_error_rad": decision.face_error_rad,
                                   "measured_front_gap_m": decision.front_gap_m,
                                   "conservative_front_gap_lower_m": decision.front_gap_m - self.uncertainty,
                                   "conservative_front_gap_upper_m": decision.front_gap_m + self.uncertainty,
                                   "uncertainty_semantics": "derived_front_gap_total_bound"}})))


if __name__ == "__main__":
    rospy.init_node("round_table_docking_adapter")
    DockingAdapter()
    rospy.spin()
