#!/usr/bin/env python3
"""Read one Twist and prove every linear/angular component is exactly zero.

Used only by controlled SIM_ONLY validation commands. It subscribes to a topic
and exits nonzero unless all six fields are numerically zero. It never
publishes, calls services, or sends actions.
"""
from __future__ import annotations

import argparse
import math
import sys

import rospy
from geometry_msgs.msg import Twist


class ZeroTwistVerifier:
    def __init__(self, topic: str) -> None:
        self._topic = topic

    def verify(self, timeout_s: float) -> None:
        try:
            message = rospy.wait_for_message(self._topic, Twist, timeout=timeout_s)
        except rospy.ROSException as exc:
            raise RuntimeError(f"timed out waiting for Twist on {self._topic}") from exc
        values = {
            "linear.x": message.linear.x,
            "linear.y": message.linear.y,
            "linear.z": message.linear.z,
            "angular.x": message.angular.x,
            "angular.y": message.angular.y,
            "angular.z": message.angular.z,
        }
        nonzero = {
            name: value
            for name, value in values.items()
            if not math.isfinite(value) or value != 0.0
        }
        if nonzero:
            raise RuntimeError(f"nonzero Twist components: {nonzero}")
        print("zero_twist_verified=" + self._topic)


def main() -> None:
    parser = argparse.ArgumentParser(description="Verify all six Twist values are zero.")
    parser.add_argument("topic")
    parser.add_argument("--timeout-s", type=float, default=3.0)
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    if not math.isfinite(args.timeout_s) or args.timeout_s <= 0.0:
        raise ValueError("--timeout-s must be finite and positive")
    rospy.init_node("raicom_verify_zero_twist", anonymous=True, disable_rosout=True)
    ZeroTwistVerifier(args.topic).verify(args.timeout_s)


if __name__ == "__main__":
    main()
