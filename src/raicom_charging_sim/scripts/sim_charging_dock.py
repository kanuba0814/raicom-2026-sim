#!/usr/bin/env python3
"""One-shot SIM-only AR alignment, straight touch, and evidence verification."""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import sys
import threading
import time

import rosgraph
import rospy
import yaml
from ar_track_alvar_msgs.msg import AlvarMarkers
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply

from raicom_charging_sim.docking import (
    DockPhase, DockProfile, MarkerObservation, alignment_command,
    dwell_complete, touch_command, valid_contact,
)
from raicom_platform.charging import (
    ChargingBackend, ChargingEvidence, ChargingOutcome, ChargingSession,
    EvidenceStatus, VerificationScope,
)


class Runner:
    def __init__(self, profile: DockProfile) -> None:
        self.profile = profile
        self.lock = threading.Lock()
        self.markers = None
        self.contacts = None
        self.publisher = rospy.Publisher("/raicom/cmd_vel/relative", Twist, queue_size=1)
        self.estop = rospy.Publisher("/raicom/safety/soft_estop", Bool, queue_size=1, latch=True)
        rospy.Subscriber(profile.marker_topic, AlvarMarkers, self.on_markers, queue_size=1)
        rospy.Subscriber(profile.contact_topic, ContactsState, self.on_contacts, queue_size=10)

    def on_markers(self, message: AlvarMarkers) -> None:
        with self.lock:
            self.markers = (time.monotonic(), tuple(message.markers))

    def on_contacts(self, message: ContactsState) -> None:
        pairs = tuple((state.collision1_name, state.collision2_name) for state in message.states)
        with self.lock:
            self.contacts = (time.monotonic(), pairs)

    def zero(self) -> None:
        rate = rospy.Rate(30)
        for _ in range(10):
            self.publisher.publish(Twist())
            rate.sleep()

    def final_zero_confirmed(self) -> bool:
        try:
            for _ in range(3):
                message = rospy.wait_for_message("/cmd_vel", Twist, timeout=1.0)
                values = (message.linear.x, message.linear.y, message.linear.z,
                          message.angular.x, message.angular.y, message.angular.z)
                if not all(math.isfinite(value) and abs(value) <= 1e-6 for value in values):
                    return False
            return True
        except (rospy.ROSException, rospy.ROSInterruptException):
            return False

    def observation(self) -> MarkerObservation:
        with self.lock:
            snapshot = self.markers
        if snapshot is None:
            raise ValueError("MARKER_MISSING")
        stamp, markers = snapshot
        selected = [item for item in markers if item.id == self.profile.marker_id]
        if len(selected) != 1:
            raise ValueError("MARKER_CARDINALITY_INVALID")
        item = selected[0]
        pose = item.pose.pose
        q = pose.orientation
        correction = quaternion_from_euler(*self.profile.marker_frame_correction_rpy_rad)
        corrected = quaternion_multiply((q.x, q.y, q.z, q.w), correction)
        yaw = euler_from_quaternion(corrected)[2]
        return MarkerObservation(item.id, item.header.frame_id, pose.position.x,
                                 pose.position.y, yaw, len(selected),
                                 time.monotonic() - stamp)

    def contact_pair(self):
        with self.lock:
            snapshot = self.contacts
        if snapshot is None or time.monotonic() - snapshot[0] > 0.20:
            return None
        return next((pair for pair in snapshot[1] if valid_contact(self.profile, *pair)), None)

    def admission(self) -> None:
        if rospy.get_param("/raicom/backend", "") != "SIM":
            raise ValueError("REAL_BACKEND_REJECTED")
        if not rospy.get_param("/use_sim_time", False):
            raise ValueError("SIM_TIME_REQUIRED")
        publishers = dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
        if publishers.get("/cmd_vel") != ["/raicom_cmd_vel_gate"]:
            raise ValueError("CMD_VEL_OWNERSHIP_INVALID")
        if rospy.get_param("/raicom_cmd_vel_gate/name", "") not in ("SIM_SAFE", "SIM_DEV_FAST"):
            raise ValueError("CMD_VEL_GATE_INVALID")

    def run(self):
        self.admission()
        deadline = time.monotonic() + self.profile.total_timeout_s
        session = ChargingSession(backend=ChargingBackend.SIM, maximum_attempts=1)
        session.begin_candidate_approach(True, self.profile.station_id, "oryxbot", deadline)
        marker_acquire_deadline = min(deadline, time.monotonic() + 5.0)
        marker_available = False
        while not rospy.is_shutdown() and time.monotonic() < marker_acquire_deadline:
            with self.lock:
                marker_available = self.markers is not None
            if marker_available:
                break
            rospy.sleep(0.03)
        if not marker_available:
            raise ValueError("MARKER_INPUT_TIMEOUT")
        stable = 0
        phase = DockPhase.ALIGN
        align_peak = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        touch_start = touch_odom = None
        contact_started = current_started = None
        contact_pair = None
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            marker = self.observation()
            if phase is DockPhase.ALIGN:
                command = alignment_command(self.profile, marker)
                if command.phase is DockPhase.TOUCH_COMMIT:
                    stable += 1
                    self.publisher.publish(Twist())
                    if stable >= self.profile.stable_samples:
                        odom = rospy.wait_for_message("/odom", Odometry, timeout=1.0)
                        touch_odom = (odom.pose.pose.position.x, odom.pose.pose.position.y)
                        touch_start = time.monotonic()
                        phase = DockPhase.TOUCH_COMMIT
                    rate.sleep()
                    continue
                stable = 0
                twist = Twist()
                twist.linear.x, twist.linear.y = command.forward_mps, command.lateral_mps
                twist.angular.z = command.angular_radps
                align_peak["vx"] = max(align_peak["vx"], abs(twist.linear.x))
                align_peak["vy"] = max(align_peak["vy"], abs(twist.linear.y))
                align_peak["wz"] = max(align_peak["wz"], abs(twist.angular.z))
                self.publisher.publish(twist)
            elif phase is DockPhase.TOUCH_COMMIT:
                odom = rospy.wait_for_message("/odom", Odometry, timeout=1.0)
                traveled = math.hypot(odom.pose.pose.position.x - touch_odom[0],
                                      odom.pose.pose.position.y - touch_odom[1])
                contact_pair = self.contact_pair()
                command = touch_command(self.profile, marker, traveled,
                                        time.monotonic() - touch_start,
                                        contact_pair is not None)
                if command.phase is DockPhase.CONTACT_DWELL:
                    self.zero()
                    session.approach_complete()
                    contact_started = time.monotonic()
                    phase = DockPhase.CONTACT_DWELL
                else:
                    twist = Twist(); twist.linear.x = command.forward_mps
                    self.publisher.publish(twist)
            elif phase is DockPhase.CONTACT_DWELL:
                now = time.monotonic()
                if dwell_complete(now_s=now, started_s=contact_started,
                                  duration_s=self.profile.contact_dwell_s,
                                  deadline_s=deadline,
                                  contact_valid=self.contact_pair() is not None):
                    evidence = ChargingEvidence(
                        ChargingBackend.SIM, self.profile.station_id, "oryxbot",
                        time.monotonic(), deadline, VerificationScope.SIMULATION,
                        contact=EvidenceStatus.CONFIRMED)
                    session.verify_contact(evidence)
                    session.begin_current_verification()
                    current_started = time.monotonic()
                    phase = DockPhase.CURRENT_DWELL
            elif phase is DockPhase.CURRENT_DWELL:
                now = time.monotonic()
                if dwell_complete(now_s=now, started_s=current_started,
                                  duration_s=self.profile.current_dwell_s,
                                  deadline_s=deadline,
                                  contact_valid=self.contact_pair() is not None):
                    evidence = ChargingEvidence(
                        ChargingBackend.SIM, self.profile.station_id, "oryxbot",
                        time.monotonic(), deadline, VerificationScope.SIMULATION,
                        contact=EvidenceStatus.CONFIRMED, current=EvidenceStatus.CONFIRMED)
                    session.verify_current(evidence)
                    break
            rate.sleep()
        if session.outcome is not ChargingOutcome.SIM_EVIDENCE_CONFIRMED:
            raise TimeoutError("CHARGING_EVIDENCE_TIMEOUT")
        self.zero()
        final_zero = self.final_zero_confirmed()
        if not final_zero:
            raise ValueError("FINAL_ZERO_UNCONFIRMED")
        session.settlement_safe_stop()
        return {
            "ok": True, "simulation_only": True,
            "outcome": ChargingOutcome.SIM_EVIDENCE_CONFIRMED.value,
            "physical_charge_confirmed": False,
            "contact_pair": contact_pair,
            "align_peak": align_peak,
            "final_zero_verified": True,
            "settlement_safe_stop": True,
        }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", required=True)
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    profile = DockProfile.from_mapping(yaml.safe_load(Path(args.profile).read_text(encoding="utf-8")))
    rospy.init_node("raicom_sim_charging_dock", anonymous=False)
    runner = Runner(profile)
    try:
        report = runner.run()
    except Exception as exc:
        runner.zero()
        final_zero = runner.final_zero_confirmed()
        runner.estop.publish(Bool(data=True))
        report = {"ok": False, "simulation_only": True, "outcome": "FAILED_CLOSED",
                  "reason": str(exc), "physical_charge_confirmed": False,
                  "final_zero_verified": final_zero}
    print(json.dumps(report, sort_keys=True), flush=True)
    raise SystemExit(0 if report["ok"] else 2)


if __name__ == "__main__":
    main()
