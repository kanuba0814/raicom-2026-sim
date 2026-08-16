#!/usr/bin/env python3
"""Camera-only authority for painted workbench-number slots; it never commands motion."""
from __future__ import annotations

import math
import threading

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose
from raicom_msgs.msg import WorkbenchSlots
from sensor_msgs.msg import CameraInfo, Image

from raicom_workbench_vision.camera_slots import (
    assign_slot_ids,
    complete_three_corner_grid,
    detect_number_circles,
    intersect_pixel_ray,
    marker_plane,
    quaternion_from_plane,
)

CAMERA_FRAME = "hand_cam_link"
MARKER_IDS = (100, 101)
MAX_INPUT_AGE_S = 0.35
MAX_INPUT_SKEW_S = 0.35
MAX_SLOT_DELTA_M = 0.003
REQUIRED_STABLE_FRAMES = 5


class CameraWorkbenchSlotDetector:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._bridge = CvBridge()
        self._camera_info = None
        self._markers = None
        self._last_slots = None
        self._last_image_stamp = None
        self._stable_frames = 0
        image_topic = rospy.get_param("~image_topic", "/hand_camera/image_raw")
        info_topic = rospy.get_param("~camera_info_topic", "/hand_camera/camera_info")
        marker_topic = rospy.get_param("~marker_topic", "/raicom/workbench/markers")
        self._publisher = rospy.Publisher("/raicom/workbench/slots", WorkbenchSlots, queue_size=1)
        rospy.Subscriber(info_topic, CameraInfo, self._on_camera_info, queue_size=1)
        rospy.Subscriber(marker_topic, AlvarMarkers, self._on_markers, queue_size=1)
        rospy.Subscriber(image_topic, Image, self._on_image, queue_size=1, buff_size=2 ** 22)

    def _on_camera_info(self, message: CameraInfo) -> None:
        with self._lock:
            self._camera_info = message

    def _on_markers(self, message: AlvarMarkers) -> None:
        with self._lock:
            self._markers = message

    def _snapshot(self):
        with self._lock:
            return self._camera_info, self._markers

    @staticmethod
    def _stamp(message) -> float:
        return message.header.stamp.to_sec()

    def _invalid(self, image: Image, detail: str) -> None:
        self._last_slots = None
        self._stable_frames = 0
        output = WorkbenchSlots()
        output.header.stamp = image.header.stamp
        output.header.frame_id = CAMERA_FRAME
        output.valid = False
        output.detail = "camera-only; " + detail
        self._publisher.publish(output)

    def _on_image(self, image_message: Image) -> None:
        camera_info, marker_message = self._snapshot()
        image_stamp = self._stamp(image_message)
        if self._last_image_stamp is not None and image_stamp <= self._last_image_stamp:
            return self._invalid(image_message, "nonincreasing image timestamp")
        self._last_image_stamp = image_stamp
        if image_message.header.frame_id != CAMERA_FRAME:
            return self._invalid(image_message, "invalid image frame")
        if camera_info is None or marker_message is None:
            return self._invalid(image_message, "missing camera info or markers")
        info_stamp = self._stamp(camera_info)
        required = [marker for marker in marker_message.markers if marker.id in MARKER_IDS]
        marker_stamps = [self._stamp(marker) for marker in required]
        if len(marker_stamps) != 2:
            return self._invalid(image_message, "missing required marker")
        if any(not math.isfinite(value) or value < 0.0 or value > MAX_INPUT_AGE_S
               for value in [image_stamp - info_stamp]
               + [image_stamp - stamp for stamp in marker_stamps]):
            return self._invalid(image_message, "stale camera info or markers")
        if max([image_stamp, info_stamp] + marker_stamps) - min(
                [image_stamp, info_stamp] + marker_stamps) > MAX_INPUT_SKEW_S:
            return self._invalid(image_message, "input timestamp skew")
        if (camera_info.header.frame_id != CAMERA_FRAME
                or camera_info.width != image_message.width
                or camera_info.height != image_message.height
                or len(camera_info.K) != 9):
            return self._invalid(image_message, "invalid camera calibration")
        selected = {marker.id: marker for marker in marker_message.markers if marker.id in MARKER_IDS}
        if set(selected) != set(MARKER_IDS):
            return self._invalid(image_message, "missing required marker")
        if any((marker.header.frame_id or marker_message.header.frame_id) != CAMERA_FRAME
               or abs(image_stamp - self._stamp(marker)) > MAX_INPUT_SKEW_S for marker in selected.values()):
            return self._invalid(image_message, "invalid marker frame or timestamp")
        poses = []
        for marker_id in MARKER_IDS:
            pose = selected[marker_id].pose.pose
            poses.append(((pose.position.x, pose.position.y, pose.position.z),
                          (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)))
        plane = marker_plane(poses)
        if plane is None:
            return self._invalid(image_message, "invalid marker plane")
        try:
            image = self._bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logwarn_throttle(2.0, "camera slot conversion failed: %s", error)
            return self._invalid(image_message, "invalid image")
        circles = detect_number_circles(image)
        if len(circles) not in (3, 4):
            return self._invalid(image_message, "expected three or four number circles")
        points = [intersect_pixel_ray(circle.pixel, camera_info.K, plane) for circle in circles]
        if any(point is None for point in points):
            return self._invalid(image_message, "ray-plane intersection failed")
        slots = (assign_slot_ids(points, plane) if len(points) == 4
                 else complete_three_corner_grid(points, plane))
        if slots is None:
            return self._invalid(image_message, "ambiguous marker-local layout")
        if self._last_slots is not None and max(
                math.dist(current, previous)
                for current, previous in zip(slots, self._last_slots)) <= MAX_SLOT_DELTA_M:
            self._stable_frames += 1
        else:
            self._stable_frames = 1
        self._last_slots = slots
        output = WorkbenchSlots()
        output.header.stamp = image_message.header.stamp
        output.header.frame_id = CAMERA_FRAME
        output.valid = self._stable_frames >= REQUIRED_STABLE_FRAMES
        output.stable_frames = self._stable_frames
        output.marker_separation_m = math.dist(poses[0][0], poses[1][0])
        output.normal_disagreement_rad = 0.0
        output.detail = ("camera-only; Hough circles + green-annulus rejection; marker-plane ray intersections; "
                         + ("four observed" if len(points) == 4 else "one occluded corner reconstructed"))
        orientation = quaternion_from_plane(plane)
        for index, point in enumerate(slots):
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = point
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation
            output.slots[index] = pose
        self._publisher.publish(output)


if __name__ == "__main__":
    rospy.init_node("camera_workbench_slot_detector")
    CameraWorkbenchSlotDetector()
    rospy.spin()
