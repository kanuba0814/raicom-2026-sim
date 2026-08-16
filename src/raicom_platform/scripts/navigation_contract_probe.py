#!/usr/bin/env python3
"""Read-only navigation/alignment ROS graph probe.

The probe subscribes only to TF transport and queries ROS-master topic
metadata. It creates no publishers, action clients, service clients, or motion
commands. A compatible result means only that the requested graph interfaces
were observed; it does not authorize navigation or alignment.
"""
from __future__ import annotations

import json
from typing import Iterable, Set

import rospy
from tf2_msgs.msg import TFMessage

from raicom_platform.navigation_readiness import assess_navigation_readiness


class TfFrameObserver:
    """Collect frame names from passive TF and TF-static subscriptions."""

    def __init__(self) -> None:
        self.frames: Set[str] = set()
        self._tf_subscriber = rospy.Subscriber("/tf", TFMessage, self._callback, queue_size=10)
        self._tf_static_subscriber = rospy.Subscriber(
            "/tf_static", TFMessage, self._callback, queue_size=10
        )

    def _callback(self, message: TFMessage) -> None:
        for transform in message.transforms:
            parent = transform.header.frame_id.strip("/")
            child = transform.child_frame_id.strip("/")
            if parent:
                self.frames.add(parent)
            if child:
                self.frames.add(child)

    def close(self) -> None:
        self._tf_subscriber.unregister()
        self._tf_static_subscriber.unregister()


def _string_list(value: object) -> Iterable[str]:
    return value if isinstance(value, list) else []


def main() -> None:
    rospy.init_node(
        "raicom_navigation_contract_probe",
        anonymous=False,
        disable_rosout=True,
    )
    expected_topics = _string_list(rospy.get_param("~expected_topics", []))
    expected_frames = _string_list(rospy.get_param("~expected_frames", []))
    observation_timeout_s = float(rospy.get_param("~observation_timeout_s", 1.0))
    if observation_timeout_s < 0.0 or observation_timeout_s > 5.0:
        raise ValueError("~observation_timeout_s must be between 0 and 5 seconds")

    observer = TfFrameObserver()
    try:
        rospy.sleep(observation_timeout_s)
        published_topics = dict(rospy.get_published_topics())
        readiness = assess_navigation_readiness(
            published_topics=published_topics,
            observed_frames=observer.frames,
            expected_topics=expected_topics,
            expected_frames=expected_frames,
        )
    finally:
        observer.close()

    payload = {
        "status": readiness.status.value,
        "expected_topics": list(readiness.expected_topics),
        "missing_topics": list(readiness.missing_topics),
        "expected_frames": list(readiness.expected_frames),
        "missing_frames": list(readiness.missing_frames),
        "observed_frames": list(readiness.observed_frames),
        "read_only": True,
        "meaning": "graph evidence only; no navigation, alignment, or safety authorization",
    }
    print("navigation_contract_probe=" + json.dumps(payload, sort_keys=True))


if __name__ == "__main__":
    main()
