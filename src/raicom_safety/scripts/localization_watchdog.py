#!/usr/bin/env python3
"""Fail-closed SIM_ONLY AMCL health evidence without pose-heartbeat coupling.

AMCL may retain a valid stationary estimate without republishing ``/amcl_pose``.
After one finite estimate initializes localization, continuously fresh scan,
odometry, and finite timestamped ``map`` to ``odom`` TF evidence are required.
This node publishes only safety state and never publishes motion commands.
"""
from __future__ import annotations

import math
from typing import Optional

import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class LocalizationWatchdog:
    def __init__(self) -> None:
        self._last_scan_s: Optional[float] = None
        self._last_odom_s: Optional[float] = None
        self._initialized = False
        self._input_timeout_s = self._positive_finite_param("~input_timeout_s", 1.0)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._output = rospy.Publisher(
            "/raicom/safety/localization_valid", Bool, queue_size=1, latch=False
        )
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._pose_callback, queue_size=1)
        rospy.Subscriber("/fuse_scan", LaserScan, self._scan_callback, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self._odom_callback, queue_size=1)
        self._output.publish(Bool(data=False))
        rospy.Timer(rospy.Duration(0.1), self._tick)

    @staticmethod
    def _positive_finite_param(name: str, default: float) -> float:
        value = float(rospy.get_param(name, default))
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be a finite positive value")
        return value

    def _pose_callback(self, message: PoseWithCovarianceStamped) -> None:
        pose = message.pose.pose
        values = (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
        )
        self._initialized = all(math.isfinite(value) for value in values)

    def _scan_callback(self, _: LaserScan) -> None:
        self._last_scan_s = rospy.get_time()

    def _odom_callback(self, _: Odometry) -> None:
        self._last_odom_s = rospy.get_time()

    def _transform_evidence(self) -> Optional[tuple]:
        try:
            stamped = self._tf_buffer.lookup_transform(
                "map", "odom", rospy.Time(0), rospy.Duration(0.05)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        transform = stamped.transform
        values = (
            transform.translation.x, transform.translation.y, transform.translation.z,
            transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w,
        )
        return values, stamped.header.stamp.to_sec()

    def _tick(self, _: rospy.TimerEvent) -> None:
        now_s = rospy.get_time()
        evidence = self._transform_evidence()
        values, tf_stamp_s = evidence if evidence is not None else (None, None)
        valid = (
            self._initialized
            and self._last_scan_s is not None
            and self._last_odom_s is not None
            and now_s - self._last_scan_s <= self._input_timeout_s
            and now_s - self._last_odom_s <= self._input_timeout_s
            and values is not None
            and all(math.isfinite(value) for value in values)
            and tf_stamp_s is not None
            and math.isfinite(tf_stamp_s)
            and abs(now_s - tf_stamp_s) <= self._input_timeout_s
        )
        self._output.publish(Bool(data=valid))


def main() -> None:
    rospy.init_node("raicom_localization_watchdog")
    LocalizationWatchdog()
    rospy.spin()


if __name__ == "__main__":
    main()
