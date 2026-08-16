#!/usr/bin/env python3
"""Bounded, simulator-only lidar sweep for occupancy-grid mapping.

This process is intentionally not a general teleoperation tool. It requests
MANUAL mode, publishes only to the RAICOM manual source, rotates in place for a
configured bounded duration, then latches the soft E-stop. The enclosing
RAICOM-only survey launch maps the safety gate output to Gazebo ``/cmd_vel``;
this script never publishes to that final topic.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Optional

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


class MappingSweepError(RuntimeError):
    """Raised when the survey cannot prove its preconditions."""


@dataclass(frozen=True)
class SweepParameters:
    angular_speed_rad_s: float
    duration_s: float
    publish_hz: float
    mode_request_timeout_s: float
    maximum_angular_speed_rad_s: float
    maximum_duration_s: float
    completion_soft_estop: bool

    @classmethod
    def from_ros(cls) -> "SweepParameters":
        parameters = cls(
            angular_speed_rad_s=float(rospy.get_param("~angular_speed_rad_s")),
            duration_s=float(rospy.get_param("~duration_s")),
            publish_hz=float(rospy.get_param("~publish_hz")),
            mode_request_timeout_s=float(rospy.get_param("~mode_request_timeout_s")),
            maximum_angular_speed_rad_s=float(
                rospy.get_param("~maximum_angular_speed_rad_s")
            ),
            maximum_duration_s=float(rospy.get_param("~maximum_duration_s")),
            completion_soft_estop=bool(rospy.get_param("~completion_soft_estop")),
        )
        parameters.validate()
        return parameters

    def validate(self) -> None:
        values = (
            self.angular_speed_rad_s,
            self.duration_s,
            self.publish_hz,
            self.mode_request_timeout_s,
            self.maximum_angular_speed_rad_s,
            self.maximum_duration_s,
        )
        if not all(math.isfinite(value) for value in values):
            raise MappingSweepError("sweep parameters must be finite")
        if self.publish_hz <= 0.0 or self.mode_request_timeout_s <= 0.0:
            raise MappingSweepError("publish_hz and mode_request_timeout_s must be positive")
        if self.maximum_angular_speed_rad_s <= 0.0 or self.maximum_duration_s <= 0.0:
            raise MappingSweepError("configured sweep maxima must be positive")
        if self.duration_s <= 0.0 or self.duration_s > self.maximum_duration_s:
            raise MappingSweepError("duration_s exceeds the configured simulator-only bound")
        if abs(self.angular_speed_rad_s) <= 0.0 or (
            abs(self.angular_speed_rad_s) > self.maximum_angular_speed_rad_s
        ):
            raise MappingSweepError("angular_speed_rad_s exceeds the configured simulator-only bound")


class MappingSweep:
    def __init__(self, parameters: SweepParameters) -> None:
        self._parameters = parameters
        self._mode: Optional[str] = None
        self._manual_command = rospy.Publisher(
            "/raicom/cmd_vel/manual", Twist, queue_size=1
        )
        self._mode_request = rospy.Publisher(
            "/raicom/mode/request", String, queue_size=1, latch=True
        )
        self._soft_estop = rospy.Publisher(
            "/raicom/safety/soft_estop", Bool, queue_size=1, latch=True
        )
        rospy.Subscriber("/raicom/mode", String, self._mode_callback, queue_size=1)
        rospy.on_shutdown(self._publish_zero)

    def run(self) -> None:
        self._request_manual_mode()
        rate = rospy.Rate(self._parameters.publish_hz)
        started_at = rospy.get_time()
        deadline = started_at + self._parameters.duration_s
        command = Twist()
        command.angular.z = self._parameters.angular_speed_rad_s

        while not rospy.is_shutdown() and rospy.get_time() < deadline:
            self._manual_command.publish(command)
            rate.sleep()

        self._publish_zero()
        if self._parameters.completion_soft_estop:
            self._soft_estop.publish(Bool(data=True))
        print("sim_mapping_sweep=" + json.dumps({
            "simulation_only": True,
            "mode": "MANUAL",
            "linear_x_m_s": 0.0,
            "angular_z_rad_s": self._parameters.angular_speed_rad_s,
            "requested_duration_s": self._parameters.duration_s,
            "elapsed_s": max(0.0, rospy.get_time() - started_at),
            "soft_estop_latched": self._parameters.completion_soft_estop,
        }, sort_keys=True))

    def _request_manual_mode(self) -> None:
        deadline = rospy.get_time() + self._parameters.mode_request_timeout_s
        rate = rospy.Rate(10.0)
        request = String(data="MANUAL")
        while not rospy.is_shutdown() and rospy.get_time() < deadline:
            self._mode_request.publish(request)
            if self._mode == "MANUAL":
                return
            rate.sleep()
        raise MappingSweepError("MANUAL mode was not accepted before the survey timeout")

    def _mode_callback(self, message: String) -> None:
        self._mode = message.data

    def _publish_zero(self) -> None:
        self._manual_command.publish(Twist())


def main() -> None:
    rospy.init_node(
        "raicom_sim_mapping_sweep",
        anonymous=False,
        disable_rosout=True,
    )
    try:
        MappingSweep(SweepParameters.from_ros()).run()
    except MappingSweepError as exc:
        print("sim_mapping_sweep_error=" + str(exc))
        rospy.signal_shutdown(str(exc))
        raise SystemExit(2)


if __name__ == "__main__":
    main()
