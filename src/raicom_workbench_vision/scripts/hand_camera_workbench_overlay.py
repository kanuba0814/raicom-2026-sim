#!/usr/bin/env python3
"""Publish a passive hand-camera overlay of observed markers and derived slots."""
from __future__ import annotations

import math
import threading

import cv2
import rospy
import tf
from ar_track_alvar_msgs.msg import AlvarMarkers
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from raicom_msgs.msg import WorkbenchSlots
from sensor_msgs.msg import CameraInfo, Image


MAX_INFO_AGE_S = 2.0
MAX_SLOTS_AGE_S = 0.5
MARKER_IDS = (100, 101)
COLORS = (
    (0, 255, 255),
    (0, 165, 255),
    (0, 255, 0),
    (255, 255, 0),
)


class HandCameraWorkbenchOverlay:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._bridge = CvBridge()
        self._tf = tf.TransformListener()
        self._camera_info = None
        self._slots = None
        self._markers = None
        image_topic = rospy.get_param("~image_topic", "/hand_camera/image_raw")
        info_topic = rospy.get_param("~camera_info_topic", "/hand_camera/camera_info")
        slots_topic = rospy.get_param("~slots_topic", "/raicom/workbench/slots")
        markers_topic = rospy.get_param("~markers_topic", "/raicom/workbench/markers")
        output_topic = rospy.get_param(
            "~output_topic", "/raicom/workbench/hand_camera_overlay"
        )
        self._publisher = rospy.Publisher(output_topic, Image, queue_size=1)
        rospy.Subscriber(info_topic, CameraInfo, self._on_camera_info, queue_size=1)
        rospy.Subscriber(slots_topic, WorkbenchSlots, self._on_slots, queue_size=1)
        rospy.Subscriber(markers_topic, AlvarMarkers, self._on_markers, queue_size=1)
        rospy.Subscriber(image_topic, Image, self._on_image, queue_size=1, buff_size=2 ** 22)

    def _on_camera_info(self, message: CameraInfo) -> None:
        with self._lock:
            self._camera_info = message

    def _on_slots(self, message: WorkbenchSlots) -> None:
        with self._lock:
            self._slots = message

    def _on_markers(self, message: AlvarMarkers) -> None:
        with self._lock:
            self._markers = message

    def _snapshot(self):
        with self._lock:
            return self._camera_info, self._slots, self._markers

    @staticmethod
    def _finite(values) -> bool:
        return all(math.isfinite(value) for value in values)

    @staticmethod
    def _project_camera_point(point, camera_info):
        if point.z <= 1e-6:
            return None
        fx, fy = camera_info.K[0], camera_info.K[4]
        cx, cy = camera_info.K[2], camera_info.K[5]
        pixel = (fx * point.x / point.z + cx, fy * point.y / point.z + cy)
        if not HandCameraWorkbenchOverlay._finite(pixel):
            return None
        return tuple(int(round(value)) for value in pixel)

    def _project(self, xyz, source_frame, image, camera_info):
        point = PointStamped()
        point.header.stamp = image.header.stamp
        point.header.frame_id = source_frame
        point.point.x, point.point.y, point.point.z = xyz
        try:
            self._tf.waitForTransform(
                image.header.frame_id,
                source_frame,
                image.header.stamp,
                rospy.Duration(0.08),
            )
            transformed = self._tf.transformPoint(image.header.frame_id, point)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            return None
        return self._project_camera_point(transformed.point, camera_info)

    @staticmethod
    def _inside(pixel, image) -> bool:
        return pixel is not None and 0 <= pixel[0] < image.shape[1] and 0 <= pixel[1] < image.shape[0]

    @staticmethod
    def _label(image, text, origin, color=(255, 255, 255), scale=0.48) -> None:
        cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
                    color, 1, cv2.LINE_AA)

    def _on_image(self, message: Image) -> None:
        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logwarn_throttle(2.0, "hand overlay conversion failed: %s", error)
            return
        camera_info, slots, markers = self._snapshot()
        now_s = message.header.stamp.to_sec()
        info_ok = (
            camera_info is not None
            and camera_info.header.frame_id == message.header.frame_id
            and camera_info.width == message.width
            and camera_info.height == message.height
            and len(camera_info.K) == 9
            and 0.0 <= now_s - camera_info.header.stamp.to_sec() <= MAX_INFO_AGE_S
            and camera_info.K[0] > 0.0
            and camera_info.K[4] > 0.0
        )
        slots_age = math.inf if slots is None else now_s - slots.header.stamp.to_sec()
        slots_ok = (
            info_ok and slots is not None and slots.valid
            and len(slots.slots) == 4
            and 0.0 <= slots_age <= MAX_SLOTS_AGE_S
            and bool(slots.header.frame_id)
        )

        if info_ok and markers is not None:
            for marker in markers.markers:
                if marker.id not in MARKER_IDS:
                    continue
                position = marker.pose.pose.position
                pixel = self._project(
                    (position.x, position.y, position.z),
                    marker.header.frame_id or markers.header.frame_id,
                    message,
                    camera_info,
                )
                if self._inside(pixel, image):
                    cv2.drawMarker(image, pixel, (255, 0, 255), cv2.MARKER_CROSS, 18, 2)
                    self._label(image, "M%d" % marker.id, (pixel[0] + 7, pixel[1] - 7), (255, 0, 255))

        if slots_ok:
            slot_pixels = []
            for index, slot in enumerate(slots.slots):
                position = slot.position
                pixel = self._project(
                    (position.x, position.y, position.z),
                    slots.header.frame_id,
                    message,
                    camera_info,
                )
                slot_pixels.append(pixel)
                if self._inside(pixel, image):
                    color = COLORS[index]
                    cv2.circle(image, pixel, 11, color, 2, cv2.LINE_AA)
                    cv2.drawMarker(image, pixel, color, cv2.MARKER_CROSS, 13, 2)
                    self._label(image, "S%d" % (index + 1), (pixel[0] + 10, pixel[1] - 10), color, 0.58)
            if all(self._inside(pixel, image) for pixel in slot_pixels):
                # Published order is [1,2,3,4], image layout is 1/3 over 2/4.
                for first, second in ((0, 1), (0, 2), (1, 3), (2, 3)):
                    cv2.line(image, slot_pixels[first], slot_pixels[second], (180, 180, 180), 1, cv2.LINE_AA)
            status = "VALID stable=%d sep=%.1fmm normal=%.3fdeg" % (
                slots.stable_frames,
                slots.marker_separation_m * 1000.0,
                math.degrees(slots.normal_disagreement_rad),
            )
            self._label(image, status, (12, 24), (0, 255, 0), 0.52)
            self._label(image, "camera/TF only; Gazebo oracle OFF", (12, 47), (0, 255, 0), 0.48)
        else:
            reason = "NO_CAMERA_INFO" if not info_ok else (
                "NO_SLOTS" if slots is None else
                "SLOTS_INVALID" if not slots.valid else
                "SLOTS_STALE %.3fs" % slots_age
            )
            self._label(image, "WORKBENCH OVERLAY INVALID: " + reason,
                        (12, 26), (0, 0, 255), 0.52)

        output = self._bridge.cv2_to_imgmsg(image, encoding="bgr8")
        output.header = message.header
        self._publisher.publish(output)


def main() -> None:
    rospy.init_node("hand_camera_workbench_overlay")
    HandCameraWorkbenchOverlay()
    rospy.spin()


if __name__ == "__main__":
    main()
