#!/usr/bin/env python3
"""Read-only ROS aggregation node for Robot Context P0."""
from __future__ import annotations
import json
import math
import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse
from raicom_state_server.snapshot import RobotContext
from raicom_state_server.mission import normalize_mission_snapshot

class StateServer:
    def __init__(self):
        profile = rospy.get_param("~profile", {})
        freshness = profile.get("freshness", {})
        self.context = RobotContext(freshness, int(profile.get("history_seconds", 90)), rospy.get_time)
        self.localization_max_age_s = float(freshness.get("localization_ms", 1000)) / 1000.0
        self.motion_active = False
        self.localization_available = False
        topics = profile.get("topics", {})
        self.gazebo_model_name = profile.get("gazebo_model_name", "oryxbot")
        frames = profile.get("frames", {})
        self.map_frame = frames.get("map", "map")
        self.base_frame = frames.get("base", "base_link")
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.context.update("ros", {"master": "connected"})
        self._subscribe(topics.get("odom", "/odom"), Odometry, self.odom)
        self._subscribe(
            topics.get("localization", "/amcl_pose"),
            PoseWithCovarianceStamped,
            self.localization,
        )
        self._subscribe(
            topics.get("gazebo_ground_truth", "/gazebo/model_states"),
            ModelStates,
            self.gazebo_ground_truth,
        )
        self._subscribe(topics.get("cmd_vel", "/cmd_vel"), Twist, self.cmd_vel)
        self._subscribe(topics.get("mission", "/raicom/mission/state"), String, self.mission)
        self._subscribe(topics.get("mission_snapshot", "/raicom/mission/snapshot"), String, self.mission_snapshot)
        self._subscribe(topics.get("mode", "/raicom/mode"), String, self.mode)
        self._subscribe(topics.get("safety_paused", "/raicom/safety/paused"), Bool, self.safety)
        rospy.Service("~state", Trigger, self.state_service)
        rospy.Service("~health", Trigger, self.health_service)
        rospy.Service("~history", Trigger, self.history_service)
        rospy.Timer(rospy.Duration(0.1), self.sample_tf)
        rospy.Timer(rospy.Duration(0.2), self.monitor_localization)
        rospy.Timer(rospy.Duration(1.0), self.publish_query_cache)
        self.publish_query_cache(None)

    def _subscribe(self, topic, message_type, callback):
        if topic: rospy.Subscriber(topic, message_type, callback, queue_size=1)

    @staticmethod
    def pose_value(pose, frame):
        q = pose.orientation
        yaw = math.degrees(
            math.atan2(
                2 * (q.w * q.z + q.x * q.y),
                1 - 2 * (q.y * q.y + q.z * q.z),
            )
        )
        return {
            "frame": frame,
            "x_m": pose.position.x,
            "y_m": pose.position.y,
            "yaw_deg": yaw,
        }

    def odom(self, message):
        self.context.update(
            "odom",
            self.pose_value(message.pose.pose, message.header.frame_id or "odom"),
            message.header.stamp.to_sec(),
        )

    def localization(self, message):
        self.context.update(
            "localization",
            self.pose_value(message.pose.pose, message.header.frame_id or "map"),
            message.header.stamp.to_sec(),
        )
        covariance = message.pose.covariance
        self.context.update(
            "localization_confidence",
            {
                "covariance_x": covariance[0],
                "covariance_y": covariance[7],
                "covariance_yaw": covariance[35],
            },
            message.header.stamp.to_sec(),
        )

    def gazebo_ground_truth(self, message):
        try:
            model_index = message.name.index(self.gazebo_model_name)
        except ValueError:
            return
        self.context.update(
            "gazebo_ground_truth",
            self.pose_value(message.pose[model_index], "world"),
        )

    def sample_tf(self, _event):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(0.02),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        pose = type("PoseValue", (), {"position": translation, "orientation": rotation})()
        transform_stamp = transform.header.stamp.to_sec()
        if transform_stamp <= 0 or rospy.get_time() - transform_stamp > self.localization_max_age_s:
            return
        self.context.update(
            "localization",
            self.pose_value(pose, self.map_frame),
            transform_stamp,
        )
        self.context.update(
            "tf",
            {"source_frame": self.map_frame, "target_frame": self.base_frame},
            transform_stamp,
        )

    def monitor_localization(self, _event):
        freshness, _age = self.context.source_state("localization", "localization_ms")
        available = freshness == "FRESH"
        if available != self.localization_available:
            self.context.event("LOCALIZATION_RECOVERED" if available else "LOCALIZATION_LOST")
            self.localization_available = available

    def cmd_vel(self, message):
        self.context.update("cmd_vel", {"linear_mps": message.linear.x, "angular_rps": message.angular.z})
        active = abs(message.linear.x) > 1e-4 or abs(message.angular.z) > 1e-4
        if active != self.motion_active:
            self.context.event("MOTION_STARTED" if active else "MOTION_STOPPED")
            self.motion_active = active
    def mission(self, message):
        current = self.context._value("mission", {"mode": "UNKNOWN"})
        current.update({"state": message.data or "UNKNOWN"})
        self.context.update("mission", current)
    def mission_snapshot(self, message):
        """Ingest data-only JSON; malformed input becomes an explicit UNKNOWN snapshot."""
        self.context.update("mission", normalize_mission_snapshot(message.data))

    def mode(self, message):
        current = self.context._value("mission", {"state": "UNKNOWN"})
        current.update({"mode": message.data or "UNKNOWN"})
        self.context.update("mission", current)
    def safety(self, message): self.context.update("safety", {"state": "STOPPED" if message.data else "OK", "estop": message.data, "motion_allowed": not message.data})
    def state_service(self, _): return TriggerResponse(True, json.dumps(self.context.snapshot("full"), separators=(",", ":")))
    def health_service(self, _): return TriggerResponse(True, json.dumps(self.context.health(), separators=(",", ":")))
    def history_service(self, _): return TriggerResponse(True, json.dumps(self.context.timeline(), separators=(",", ":")))
    def publish_query_cache(self, _):
        self.context.record_sample()
        rospy.set_param("/raicom/context/snapshot", json.dumps(self.context.snapshot("full"), separators=(",", ":")))
        rospy.set_param("/raicom/context/health", json.dumps(self.context.health(), separators=(",", ":")))
        rospy.set_param("/raicom/context/history", json.dumps(self.context.timeline(), separators=(",", ":")))
def main():
    rospy.init_node("raicom_state_server")
    StateServer()
    rospy.spin()
if __name__ == "__main__": main()
