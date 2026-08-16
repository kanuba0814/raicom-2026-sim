#!/usr/bin/env python3
"""ROS transport for the RAICOM safety velocity gate.

No input is forwarded directly. The timer publishes a command at a fixed rate;
if the selected input is stale or denied it publishes zero. The default output
is deliberately a non-vendor topic. A future, verified real bringup must make
an explicit remap to the physical base command topic.
"""
from __future__ import annotations

import json
from typing import Optional

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty, String

from raicom_safety.execution_profile import ExecutionProfileError, validate_execution_profile
from raicom_safety.gate import VelocityCommand, VelocityGate
from raicom_safety.supervisor import CommandSource, SafetySupervisor


class CmdVelGateNode:
    def __init__(self) -> None:
        self._supervisor = SafetySupervisor()
        self._localization_valid = False
        self._last_localization_message_s = 0.0
        self._arm_extended = False
        self._mode = "AUTO"
        self._last_mode_message_s = 0.0
        backend = rospy.get_param("/raicom/backend")
        profile = validate_execution_profile(rospy.get_param("~"), backend)
        timeout_s = profile["stale_timeout_s"]
        gate_limits = profile["gate_limits"]
        self._mode_timeout_s = profile["mode_timeout_s"]
        self._localization_timeout_s = profile["localization_timeout_s"]
        self._gate = VelocityGate(
            self._supervisor,
            timeout_s,
            max_abs_linear_x=gate_limits["max_abs_linear_x"],
            max_abs_linear_y=gate_limits["max_abs_linear_y"],
            max_planar_speed=gate_limits["max_planar_speed"],
            max_abs_angular_z=gate_limits["max_abs_angular_z"],
        )

        output_topic = rospy.get_param("~output_topic", "/raicom/cmd_vel/final")
        self._output = rospy.Publisher(output_topic, Twist, queue_size=1)
        self._motion_permitted = rospy.Publisher("/raicom/safety/motion_permitted", Bool, queue_size=1, latch=True)
        self._state = rospy.Publisher("/raicom/safety/state", String, queue_size=1, latch=True)

        rospy.Subscriber("/raicom/cmd_vel/nav", Twist, self._command_callback, CommandSource.NAVIGATION, queue_size=1)
        rospy.Subscriber("/raicom/cmd_vel/relative", Twist, self._command_callback, CommandSource.RELATIVE_MOVE, queue_size=1)
        rospy.Subscriber("/raicom/cmd_vel/manual", Twist, self._command_callback, CommandSource.MANUAL, queue_size=1)
        rospy.Subscriber("/raicom/mode", String, self._mode_callback, queue_size=1)
        rospy.Subscriber("/raicom/safety/localization_valid", Bool, self._localization_callback, queue_size=1)
        rospy.Subscriber("/raicom/safety/arm_extended", Bool, self._arm_callback, queue_size=1)
        rospy.Subscriber("/raicom/safety/soft_estop", Bool, self._soft_estop_callback, queue_size=1)
        rospy.Subscriber("/raicom/safety/hard_estop", Bool, self._hard_estop_callback, queue_size=1)
        rospy.Subscriber("/raicom/safety/ack", Empty, self._ack_callback, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / profile["publish_hz"]), self._tick)
        self._publish_state("initialized")

    def _command_callback(self, message: Twist, source: CommandSource) -> None:
        self._gate.update(
            source,
            VelocityCommand(
                linear_x=message.linear.x,
                angular_z=message.angular.z,
                linear_y=message.linear.y,
                linear_z=message.linear.z,
                angular_x=message.angular.x,
                angular_y=message.angular.y,
            ),
            rospy.get_time(),
        )

    def _mode_callback(self, message: String) -> None:
        self._mode = message.data if message.data in ("AUTO", "DISPATCH", "MANUAL") else "INVALID"
        self._last_mode_message_s = rospy.get_time()

    def _localization_callback(self, message: Bool) -> None:
        self._localization_valid = message.data
        self._last_localization_message_s = rospy.get_time()

    def _arm_callback(self, message: Bool) -> None:
        self._arm_extended = message.data

    def _soft_estop_callback(self, message: Bool) -> None:
        if message.data:
            self._supervisor.soft_estop()

    def _hard_estop_callback(self, message: Bool) -> None:
        if message.data:
            self._supervisor.hard_estop()

    def _ack_callback(self, _: Empty) -> None:
        self._supervisor.acknowledge_soft_estop()

    def _tick(self, _: rospy.TimerEvent) -> None:
        mode_fresh = rospy.get_time() - self._last_mode_message_s <= self._mode_timeout_s
        localization_fresh = (
            rospy.get_time() - self._last_localization_message_s <= self._localization_timeout_s
        )
        self._supervisor.update_inputs(
            localization_valid=self._localization_valid and localization_fresh,
            watchdog_healthy=mode_fresh,
            arm_extended=self._arm_extended,
        )
        decision = self._gate.decide(self._mode, rospy.get_time())
        output = Twist()
        output.linear.x = decision.command.linear_x
        output.linear.y = decision.command.linear_y
        output.angular.z = decision.command.angular_z
        self._output.publish(output)
        self._motion_permitted.publish(Bool(data=not decision.command.is_zero))
        self._publish_state(decision.reason, decision.source)

    def _publish_state(self, reason: str, source: Optional[CommandSource] = None) -> None:
        state = self._supervisor.snapshot()
        self._state.publish(String(data=json.dumps({
            "soft_estop_latched": state.soft_estop_latched,
            "hard_estop_latched": state.hard_estop_latched,
            "localization_valid": state.localization_valid,
            "watchdog_healthy": state.watchdog_healthy,
            "arm_extended": state.arm_extended,
            "selected_source": source.value if source else None,
            "reason": reason,
        }, sort_keys=True)))


def main() -> None:
    rospy.init_node("raicom_cmd_vel_gate")
    try:
        CmdVelGateNode()
    except (ExecutionProfileError, KeyError, TypeError) as error:
        rospy.logfatal("cmd_vel gate requires an explicit validated execution profile: %s", error)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
