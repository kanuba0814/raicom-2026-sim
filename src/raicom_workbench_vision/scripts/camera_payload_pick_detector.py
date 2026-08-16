#!/usr/bin/env python3
"""Per-frame OpenCV QR adapter for bounded on-demand pick snapshots."""
import sys
import threading

QR_OPENCV_PATH = "/opt/raicom-qr-opencv/python"
if QR_OPENCV_PATH not in sys.path:
    sys.path.insert(0, QR_OPENCV_PATH)

import rospy
import tf
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Vector3Stamped
from raicom_msgs.msg import CameraPayloadPick, CameraPayloadPickRequest
from sensor_msgs.msg import CameraInfo, Image

from raicom_workbench_vision.payload_pick import (
    CameraInfoObservation,
    Footprint,
    PayloadProjector,
    PayloadRequestGate,
    SupportPlane,
)


class CameraPayloadPickNode:
    """Publish each frame's independent result; callers own snapshot aggregation."""
    def __init__(self):
        with open(rospy.get_param("~config"), encoding="utf-8") as source:
            self.config = yaml.safe_load(source)
        self.projector = PayloadProjector(self.config)
        self.request_gate = PayloadRequestGate(
            tuple(self.config["allowlisted_payloads"]),
            int(self.config["request_max_frames"]),
        )
        self.request_lock = threading.Lock()
        self.processing_lock = threading.Lock()
        self.bridge = CvBridge()
        cv2 = __import__("cv2")
        if not hasattr(cv2, "QRCodeDetector"):
            raise RuntimeError("the configured OpenCV lacks QRCodeDetector")
        self.detector = cv2.QRCodeDetector()
        self.camera_info = None
        self.tf_listener = tf.TransformListener()
        self.publisher = rospy.Publisher(
            "/raicom/workbench/camera_payload_pick",
            CameraPayloadPick,
            queue_size=len(self.config["allowlisted_payloads"]),
        )
        rospy.Subscriber(
            rospy.get_param("~camera_info_topic", "/hand_camera/camera_info"),
            CameraInfo,
            self._camera_info,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~image_topic", "/hand_camera/image_raw"),
            Image,
            self._image,
            queue_size=1,
        )
        rospy.Subscriber(
            rospy.get_param("~request_topic", "/raicom/workbench/camera_payload_pick/request"),
            CameraPayloadPickRequest,
            self._request,
            queue_size=1,
        )

    def _camera_info(self, message):
        self.camera_info = message

    def _request(self, message):
        with self.processing_lock:
            with self.request_lock:
                accepted = self.request_gate.begin(message.request_id, message.payload)
                if accepted:
                    self.projector.reset(message.payload)
        if not accepted:
            rospy.logwarn("camera payload request rejected")

    def _support_plane(self, image_message):
        point = PointStamped()
        point.header.stamp = image_message.header.stamp
        point.header.frame_id = self.config["support_plane_frame"]
        point.point.x, point.point.y, point.point.z = self.config["support_plane_point_m"]
        normal = Vector3Stamped()
        normal.header = point.header
        normal.vector.x, normal.vector.y, normal.vector.z = self.config["support_plane_normal"]
        try:
            self.tf_listener.waitForTransform(
                image_message.header.frame_id,
                point.header.frame_id,
                image_message.header.stamp,
                rospy.Duration(0.2),
            )
            transformed_point = self.tf_listener.transformPoint(
                image_message.header.frame_id, point
            )
            transformed_normal = self.tf_listener.transformVector3(
                image_message.header.frame_id, normal
            )
        except Exception:
            return SupportPlane("", 0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
        return SupportPlane(
            image_message.header.frame_id,
            image_message.header.stamp.to_sec(),
            (transformed_point.point.x, transformed_point.point.y,
             transformed_point.point.z),
            (transformed_normal.vector.x, transformed_normal.vector.y,
             transformed_normal.vector.z),
        )

    def _image(self, message):
        with self.processing_lock:
            self._process_image(message)

    def _process_image(self, message):
        # Idle camera traffic is deliberately ignored: no QR decode and no target
        # publication occurs without a current executor request.
        with self.request_lock:
            request = self.request_gate.take_frame()
        if request is None:
            return
        request_id, requested_payload = request
        footprints = []
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            detected, texts, corners, _ = self.detector.detectAndDecodeMulti(image)
            decoded = []
            if detected and corners is not None:
                decoded.extend(zip(texts, corners))
            # OpenCV can detect a lone, perspective-skewed QR with the Multi
            # API yet return an empty decoded string. The Single API decodes
            # that same raw frame reliably, so use it only when Multi did not
            # produce the exact requested payload.
            if not any(text == requested_payload for text, _ in decoded):
                text, corner_set, _ = self.detector.detectAndDecode(image)
                if text and corner_set is not None:
                    decoded.append((text, corner_set))
            for text, corner_set in decoded:
                if text != requested_payload:
                    continue
                points = tuple(
                    tuple(float(value) for value in point)
                    for point in corner_set.reshape(-1, 2)
                )
                if len(points) != 4:
                    continue
                center = tuple(
                    sum(point[axis] for point in points) / 4.0
                    for axis in range(2)
                )
                footprints.append(Footprint(
                    text, center, points, 1.0, "opencv_qr_raw"
                ))
        except Exception:
            footprints = []
        info = self.camera_info
        camera = (
            CameraInfoObservation((), "", 0.0, 0, 0)
            if info is None else CameraInfoObservation(
                tuple(info.K),
                info.header.frame_id,
                info.header.stamp.to_sec(),
                info.width,
                info.height,
            )
        )
        results = self.projector.process(
            footprints,
            self._support_plane(message),
            camera,
            message.header.stamp.to_sec(),
            message.header.frame_id,
            message.width,
            message.height,
            rospy.Time.now().to_sec(),
        )
        result = results[requested_payload]
        if result.valid:
            with self.request_lock:
                self.request_gate.complete(request_id, requested_payload)
        # A request exposes only its payload, including a fail-closed invalid
        # result; unrelated labels cannot satisfy this transaction.
        for result in (result,):
            output = CameraPayloadPick()
            output.header = message.header
            output.header.frame_id = result.frame_id
            output.request_id = request_id
            output.payload = result.payload
            output.class_name = result.payload
            output.detector = result.detector
            output.pixel_center_x, output.pixel_center_y = result.pixel_center
            output.surface_point.x, output.surface_point.y, output.surface_point.z = result.surface_point
            output.support_normal.x, output.support_normal.y, output.support_normal.z = result.support_normal
            output.tcp_pick_point.x, output.tcp_pick_point.y, output.tcp_pick_point.z = result.tcp_point
            output.quality = result.quality
            output.stable_frames = result.stable_frames
            output.valid = result.valid
            output.detail = result.detail
            self.publisher.publish(output)


if __name__ == "__main__":
    rospy.init_node("camera_payload_pick_detector")
    CameraPayloadPickNode()
    rospy.spin()
