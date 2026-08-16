#!/usr/bin/env python3
"""Fail-closed SLAM health evidence for stationary SIM_ONLY live-map navigation.

Unlike the AMCL watchdog, this node validates the live GMapping graph: fresh
lidar, odometry, occupancy-grid updates, and a finite ``map`` to
``base_footprint`` transform are all required. It only publishes safety state;
it never publishes a motion command.
"""
from __future__ import annotations

import math
from typing import Optional, Sequence

import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from raicom_safety.slam_health import SlamHealth


class SlamHealthWatchdog:
    def __init__(self) -> None:
        self._timeout_s = self._positive_finite_param("~input_timeout_s", 1.0)
        self._health = SlamHealth(self._timeout_s)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._output = rospy.Publisher(
            "/raicom/safety/localization_valid", Bool, queue_size=1, latch=False
        )
        rospy.Subscriber("/fuse_scan", LaserScan, self._scan_callback, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self._odom_callback, queue_size=1)
        rospy.Subscriber("/map", OccupancyGrid, self._map_callback, queue_size=1)
        self._output.publish(Bool(data=False))
        rospy.Timer(rospy.Duration(0.1), self._tick)

    @staticmethod
    def _positive_finite_param(name: str, default: float) -> float:
        value = float(rospy.get_param(name, default))
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be a finite positive value")
        return value

    def _scan_callback(self, _: LaserScan) -> None:
        self._health.observe_scan(rospy.get_time())

    def _odom_callback(self, _: Odometry) -> None:
        self._health.observe_odom(rospy.get_time())

    def _map_callback(self, _: OccupancyGrid) -> None:
        self._health.observe_map(rospy.get_time())

    def _transform_evidence(self) -> Optional[tuple]:
        try:
            stamped = self._tf_buffer.lookup_transform(
                "map", "odom", rospy.Time(0), rospy.Duration(0.05)
            )
            transform = stamped.transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        translation = transform.translation
        rotation = transform.rotation
        values = (translation.x, translation.y, translation.z,
                  rotation.x, rotation.y, rotation.z, rotation.w)
        return values, stamped.header.stamp.to_sec()

    def _tick(self, _: rospy.TimerEvent) -> None:
        now_s = rospy.get_time()
        evidence = self._transform_evidence()
        values, stamp_s = evidence if evidence is not None else (None, None)
        valid = self._health.valid(now_s, values, stamp_s)
        self._output.publish(Bool(data=valid))


def main() -> None:
    rospy.init_node("raicom_slam_health_watchdog")
    SlamHealthWatchdog()
    rospy.spin()


if __name__ == "__main__":
    main()
