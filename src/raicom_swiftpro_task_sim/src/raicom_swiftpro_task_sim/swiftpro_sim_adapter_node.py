#!/usr/bin/env python3
"""Explicit opt-in ROS bridge for the SIM-only SwiftPro motion adapter.

It intentionally exposes no service/action that can initiate motion.  An
integrator must call the core adapter from an approved SIM task lane.
"""
from __future__ import annotations

import threading

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from .motion_adapter import CONTROLLER_JOINT_NAMES, JointObservation


class SwiftproSimAdapterNode:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest = None
        self._publishers = {
            name: rospy.Publisher("/swiftpro/%s_position_controller/command" % name,
                                 Float64, queue_size=1)
            for name in CONTROLLER_JOINT_NAMES
        }
        self._subscriber = rospy.Subscriber("/swiftpro/joint_states", JointState,
                                            self._on_joint_state, queue_size=1)

    def _on_joint_state(self, message: JointState) -> None:
        # An absent velocity vector is rejected by the core adapter: settled
        # evidence cannot be inferred from position-only feedback.
        observation = JointObservation(tuple(message.name), tuple(message.position),
                                       tuple(message.velocity), message.header.stamp.to_sec())
        with self._lock:
            self._latest = observation

    def latest_observation(self):
        with self._lock:
            return self._latest

    def publish(self, topic: str, position: float) -> None:
        name = topic.split("/")[-2].replace("_position_controller", "")
        publisher = self._publishers.get(name)
        if publisher is None:
            raise ValueError("refusing non-SwiftPro command topic")
        publisher.publish(Float64(data=position))


def main() -> None:
    rospy.init_node("swiftpro_sim_motion_adapter")
    SwiftproSimAdapterNode()
    rospy.loginfo("SIM_WORKING_ASSUMPTION: SwiftPro adapter armed for observation only; no motion API is exposed")
    rospy.spin()


if __name__ == "__main__":
    main()
