#!/usr/bin/env python3
"""ROS adapter for SIM-only circle observations; it publishes no robot commands."""
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from raicom_round_table_navigation.circle_detector import CircleDetector, DetectorConfig


class RoundTableDetectorNode:
    def __init__(self):
        config_values = {}
        for field_name in DetectorConfig.__dataclass_fields__:
            config_values[field_name] = rospy.get_param("~" + field_name, getattr(DetectorConfig, field_name))
        self.detector = CircleDetector(DetectorConfig(**config_values))
        # PoseStamped remains a compatibility-only center observation. Radius is
        # exclusively carried by the typed circle Vector3Stamped channel.
        self.observation_publisher = rospy.Publisher(rospy.get_param("~observation_topic", "/round_table/observation"), PoseStamped, queue_size=1)
        self.circle_publisher = rospy.Publisher(rospy.get_param("~circle_topic", "/round_table/circle"), Vector3Stamped, queue_size=1)
        self.valid_publisher = rospy.Publisher(rospy.get_param("~valid_topic", "/round_table/valid"), Bool, queue_size=1)
        self.diagnostic_publisher = rospy.Publisher(rospy.get_param("~diagnostic_topic", "/round_table/diagnostic"), DiagnosticArray, queue_size=1)
        rospy.Subscriber(rospy.get_param("~scan_topic", "/fuse_scan"), LaserScan, self._on_scan, queue_size=1)

    def _on_scan(self, scan):
        result = self.detector.detect(scan.ranges, scan.angle_min, scan.angle_increment)
        self.valid_publisher.publish(result.valid)
        diagnostic = DiagnosticStatus(
            level=DiagnosticStatus.OK if result.valid else DiagnosticStatus.WARN,
            name="round_table_detector",
            message=result.reason,
            values=[KeyValue("stable_frames", str(result.stable_frames))],
        )
        if result.observation is not None:
            observed = result.observation
            diagnostic.values.extend([
                KeyValue("radius_m", "{:.6f}".format(observed.radius_m)),
                KeyValue("residual_m", "{:.6f}".format(observed.residual_m)),
                KeyValue("point_count", str(observed.point_count)),
            ])
        self.diagnostic_publisher.publish(DiagnosticArray(header=scan.header, status=[diagnostic]))
        if not result.valid:
            return
        pose = PoseStamped()
        pose.header = scan.header
        pose.pose.position.x = result.observation.center_x_m
        pose.pose.position.y = result.observation.center_y_m
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        circle = Vector3Stamped()
        circle.header = scan.header
        circle.vector.x = result.observation.center_x_m
        circle.vector.y = result.observation.center_y_m
        circle.vector.z = result.observation.radius_m
        self.circle_publisher.publish(circle)
        self.observation_publisher.publish(pose)


if __name__ == "__main__":
    rospy.init_node("round_table_detector")
    RoundTableDetectorNode()
    rospy.spin()
