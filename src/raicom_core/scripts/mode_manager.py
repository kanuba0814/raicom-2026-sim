#!/usr/bin/env python3
"""ROS bridge for the safe AUTO/DISPATCH/MANUAL transition model.

This node does not drive any actuator. It accepts mode requests, checks the
safe-point/precheck evidence published by other system components, and exposes
the currently accepted mode for the safety velocity gate and future HMI.
"""
from __future__ import annotations

import rospy
from std_msgs.msg import Bool, String

from raicom_core import OperationMode, TransitionError
from raicom_core.modes import ModeManager


class ModeManagerNode:
    def __init__(self) -> None:
        self._primitive_safe_point = True
        self._safety_paused = True
        self._precheck_complete = False
        self._manager = ModeManager()
        self._mode_pub = rospy.Publisher("/raicom/mode", String, queue_size=1, latch=True)
        self._reason_pub = rospy.Publisher("/raicom/mode/reason", String, queue_size=1, latch=True)
        rospy.Subscriber("/raicom/mode/request", String, self._request_callback, queue_size=1)
        rospy.Subscriber("/raicom/mission/primitive_safe_point", Bool, self._safe_point_callback, queue_size=1)
        rospy.Subscriber("/raicom/safety/paused", Bool, self._safety_paused_callback, queue_size=1)
        rospy.Subscriber("/raicom/precheck/complete", Bool, self._precheck_callback, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / float(rospy.get_param("~heartbeat_hz", 5.0))), self._heartbeat)
        self._publish("initialized")

    def _safe_point_callback(self, message: Bool) -> None:
        self._primitive_safe_point = message.data

    def _safety_paused_callback(self, message: Bool) -> None:
        self._safety_paused = message.data

    def _precheck_callback(self, message: Bool) -> None:
        self._precheck_complete = message.data

    def _request_callback(self, message: String) -> None:
        try:
            target = OperationMode(message.data)
            self._manager.switch(
                target,
                primitive_safe_point=self._primitive_safe_point,
                safety_paused=self._safety_paused,
                precheck_complete=self._precheck_complete,
            )
            self._publish("accepted")
        except (ValueError, TransitionError) as exc:
            self._publish(f"rejected:{exc}")

    def _heartbeat(self, _: rospy.TimerEvent) -> None:
        self._publish("heartbeat")

    def _publish(self, reason: str) -> None:
        self._mode_pub.publish(String(data=self._manager.mode.value))
        self._reason_pub.publish(String(data=reason))


def main() -> None:
    rospy.init_node("raicom_mode_manager")
    ModeManagerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
