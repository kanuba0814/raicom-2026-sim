"""ROS-independent snapshot model; intentionally contains no control APIs."""
from __future__ import annotations
import copy
import time
from collections import deque

from raicom_state_server.mission import normalize_mission_snapshot

UNKNOWN = "UNKNOWN"

class RobotContext:
    def __init__(self, freshness=None, history_seconds=90, now=time.time):
        self.now = now
        self.freshness = freshness or {
            "odom_ms": 300,
            "localization_ms": 1000,
            "gazebo_ground_truth_ms": 300,
            "tf_ms": 300,
            "mission_ms": 1000,
            "arm_ms": 500,
            "safety_ms": 1000,
        }
        self.values = {}
        self.history = deque()
        self.events = deque()
        self.history_seconds = history_seconds
        self._last_sample = None

    def update(self, source, value, source_timestamp=None):
        received = self.now()
        self.values[source] = {"value": value, "source_timestamp": source_timestamp, "receive_timestamp": received}

    def record_sample(self):
        """Keep a down-sampled structured snapshot ring; never raw ROS streams."""
        current = self.now()
        if self._last_sample is not None and current - self._last_sample < 1.0:
            return
        self.history.append({"timestamp": current, "snapshot": self.snapshot("compact")})
        self._last_sample = current
        cutoff = current - self.history_seconds
        while self.history and self.history[0]["timestamp"] < cutoff:
            self.history.popleft()

    def event(self, event_type, details=None):
        item = {"timestamp": self.now(), "type": event_type}
        if details: item["details"] = details
        self.events.append(item)
        self._trim()

    def _age(self, source):
        record = self.values.get(source)
        if not record: return None
        return max(0, int((self.now() - record["receive_timestamp"]) * 1000))

    def source_state(self, source, threshold_key):
        age = self._age(source)
        if age is None: return "MISSING", None
        return ("FRESH" if age <= self.freshness.get(threshold_key, 1000) else "STALE"), age

    def _value(self, source, default):
        record = self.values.get(source)
        return copy.deepcopy(record["value"]) if record else default

    def _mission_snapshot(self, mission_status, mission_age):
        mission = normalize_mission_snapshot(self._value("mission", {}))
        mission["freshness"] = mission_status
        mission["age_ms"] = mission_age
        return mission

    def snapshot(self, detail="compact"):
        odom_status, odom_age = self.source_state("odom", "odom_ms")
        localization_status, localization_age = self.source_state("localization", "localization_ms")
        ground_truth_status, ground_truth_age = self.source_state(
            "gazebo_ground_truth", "gazebo_ground_truth_ms"
        )
        mission_status, mission_age = self.source_state("mission", "mission_ms")
        arm_status, arm_age = self.source_state("arm", "arm_ms")
        safety_status, safety_age = self.source_state("safety", "safety_ms")
        unknown_pose = {"frame": UNKNOWN, "x_m": None, "y_m": None, "yaw_deg": None}
        odometry_pose = self._value("odom", unknown_pose)
        localized_pose = self._value("localization", unknown_pose)
        robot_pose = localized_pose if localization_status != "MISSING" else odometry_pose
        snap = {
            "timestamp": {
                "wall": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                "ros": self.now(),
                "age_ms": localization_age if localization_age is not None else odom_age,
            },
            "backend": "SIM",
            "verification": "SIM_TESTED" if ground_truth_status != "MISSING" else "HW_UNVERIFIED",
            "robot": {
                "pose": robot_pose,
                "odometry_pose": odometry_pose,
                "velocity": self._value("cmd_vel", {"linear_mps": None, "angular_rps": None}),
            },
            "localization": {
                "state": "OK" if localization_status == "FRESH" else localization_status,
                "pose": localized_pose,
                "confidence": self._value("localization_confidence", None),
                "age_ms": localization_age,
            },
            "simulation": {
                "ground_truth": self._value("gazebo_ground_truth", unknown_pose),
                "freshness": ground_truth_status,
                "age_ms": ground_truth_age,
                "source": "/gazebo/model_states",
            },
            "mission": self._mission_snapshot(mission_status, mission_age),
            "navigation": self._value("navigation", {"state": UNKNOWN, "goal_name": UNKNOWN, "distance_remaining_m": None}),
            "arm": self._value("arm", {"state": "HW_UNVERIFIED", "tcp": {}, "target": {}, "joint_limit_state": UNKNOWN, "freshness": arm_status, "age_ms": arm_age}),
            "gripper": self._value("gripper", {"command": UNKNOWN, "grasp": UNKNOWN}),
            "vision": self._value("vision", {"selected_target": UNKNOWN, "confidence": None, "pickable": None}),
            "charging": self._value("charging", {"state": "HW_UNVERIFIED"}),
            "safety": self._value("safety", {"state": "UNKNOWN" if safety_status == "MISSING" else safety_status, "estop": None, "motion_allowed": None, "age_ms": safety_age}),
        }
        if detail == "full":
            snap["sources"] = {key: {"freshness": self.source_state(key, "tf_ms" if key == "tf" else "mission_ms")[0], "age_ms": self._age(key)} for key in self.values}
        return snap

    def health(self):
        result = {}
        mapping = {"tf": ("tf", "tf_ms"), "localization": ("localization", "localization_ms"), "gazebo_ground_truth": ("gazebo_ground_truth", "gazebo_ground_truth_ms"), "mission": ("mission", "mission_ms"), "safety": ("safety", "safety_ms"), "arm": ("arm", "arm_ms"), "gripper": ("gripper", "arm_ms"), "charging": ("charging", "arm_ms")}
        result["ros_master"] = {
            "status": "PASS",
            "last_seen_age_ms": 0,
            "reason": "state server is connected to ROS master",
            "recommended_next_check": "none",
        }
        for name, (source, threshold) in mapping.items():
            freshness, age = self.source_state(source, threshold)
            status = "PASS" if freshness == "FRESH" else ("DEGRADED" if freshness == "STALE" else "UNKNOWN")
            if name in ("arm", "gripper", "charging") and freshness == "MISSING": status = "HW_UNVERIFIED"
            result[name] = {"status": status, "last_seen_age_ms": age, "reason": freshness, "recommended_next_check": "start SIM state server or verify configured topic"}
        for name in ("navigation", "front_lidar", "rear_lidar", "base_camera", "hand_camera"):
            result[name] = {"status": "UNKNOWN", "last_seen_age_ms": None, "reason": "not configured in P0 profile", "recommended_next_check": "configure and validate hardware contract"}
        return result

    def timeline(self, seconds=10, focus=None, max_events=20):
        cutoff = self.now() - max(1, min(seconds, self.history_seconds))
        events = [e for e in self.events if e["timestamp"] >= cutoff and (not focus or focus.lower() in e["type"].lower())]
        return events[-max(1, min(max_events, 50)):]

    def _trim(self):
        cutoff = self.now() - self.history_seconds
        while self.events and self.events[0]["timestamp"] < cutoff: self.events.popleft()
