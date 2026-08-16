#!/usr/bin/env python3
"""Camera-only QR detector; it consumes neither Gazebo state nor control interfaces."""
import math
import sys

QR_OPENCV_PATH = "/opt/raicom-qr-opencv/python"
if QR_OPENCV_PATH not in sys.path:
    sys.path.insert(0, QR_OPENCV_PATH)

import cv2
import numpy as np
import rospy
import tf2_ros
import yaml
from sensor_msgs.msg import CameraInfo, Image
from raicom_msgs.msg import QrObservation
from raicom_qr_task_sim.logic import (finite, image_to_bgr, normalized_quaternion,
                                      load_sim_config, quaternion_multiply, rotation_matrix_quaternion, update_stability, valid_camera_info)

class Detector:
    def __init__(self):
        with open(rospy.get_param("~config"), encoding="utf-8") as stream: self.config = load_sim_config(yaml.safe_load(stream))
        self.allowlist = {item["payload"]: item["target_id"] for item in self.config["qr_targets"]}
        self.infos, self.last_payload, self.frames = {}, {}, {}
        self.tf = tf2_ros.Buffer(); self.listener = tf2_ros.TransformListener(self.tf)
        self.publisher = rospy.Publisher("/raicom/qr/observations", QrObservation, queue_size=10)
        if cv2.__version__ != "4.14.0" or not cv2.__file__.startswith(QR_OPENCV_PATH):
            raise RuntimeError("isolated QR OpenCV runtime is unavailable")
        self.decoder = cv2.QRCodeDetector()
        for camera in ("bottom", "hand"):
            rospy.Subscriber("/%s_camera/camera_info" % camera, CameraInfo, self._info, camera, queue_size=1)
            rospy.Subscriber("/%s_camera/image_raw" % camera, Image, self._image, camera, queue_size=1)

    def _info(self, message, camera): self.infos[camera] = message
    def _reset(self, camera): self.last_payload[camera], self.frames[camera] = update_stability(self.last_payload.get(camera), self.frames.get(camera, 0), None)
    def _invalid(self, image, camera, detail):
        self._reset(camera); self.publisher.publish(QrObservation(header=image.header, source_camera=camera, valid=False, detail=detail))

    def _image(self, image, camera):
        try:
            age = (rospy.Time.now() - image.header.stamp).to_sec()
            limits = self.config["detector"]
            if age < 0 or age > limits["image_max_age_s"]: return self._invalid(image, camera, "stale or future image")
            info = self.infos.get(camera)
            if not valid_camera_info(info, image, limits["camera_info_max_skew_s"]): return self._invalid(image, camera, "invalid CameraInfo contract")
            frame = image_to_bgr(image, np)
            payload, corners, _ = self.decoder.detectAndDecode(frame)
            if payload not in self.allowlist or corners is None: return self._invalid(image, camera, "payload not allowlisted or no QR corners")
            points = corners.reshape(4, 2).astype(np.float32)
            if points.shape != (4, 2) or not finite(list(points.flat)): return self._invalid(image, camera, "invalid QR corners")
            half = self.config["block"]["qr_marker_size_m"] / 2.0
            object_points = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], np.float32)
            camera_matrix, distortion = np.array(info.K).reshape(3, 3), np.array(info.D)
            ok, rvec, tvec = cv2.solvePnP(object_points, points, camera_matrix, distortion)
            if not ok or not finite(list(rvec.flat) + list(tvec.flat)): return self._invalid(image, camera, "PnP failed")
            projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, distortion)
            error = float(np.mean(np.linalg.norm(projected.reshape(4, 2) - points, axis=1)))
            if not math.isfinite(error) or error > limits["max_reprojection_error_px"]: return self._invalid(image, camera, "reprojection error")
            transform = self.tf.lookup_transform(self.config["frame_id"], image.header.frame_id, image.header.stamp, rospy.Duration(0.05))
            tf_q = transform.transform.rotation; q_tf = (tf_q.x, tf_q.y, tf_q.z, tf_q.w)
            if not normalized_quaternion(q_tf): return self._invalid(image, camera, "invalid TF quaternion")
            rot, _ = cv2.Rodrigues(rvec)
            x, y, z, w = q_tf
            matrix = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
            point = matrix.dot(tvec.reshape(3)) + np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
            if not finite(list(point)) or any(point[i] < limits["workspace_xyz_min"][i] or point[i] > limits["workspace_xyz_max"][i] for i in range(3)): return self._invalid(image, camera, "outside SIM workspace")
            q_marker = rotation_matrix_quaternion(rot)
            q_world = quaternion_multiply(q_tf, q_marker)
            previous, count = update_stability(self.last_payload.get(camera), self.frames.get(camera, 0), payload); self.last_payload[camera], self.frames[camera] = previous, count
            output = QrObservation(header=image.header, target_id=self.allowlist[payload], payload=payload, source_camera=camera, reprojection_error_px=error, stable_frames=count, valid=count >= limits["stable_frames_required"], detail="SIM_WORKING_ASSUMPTION")
            output.header.frame_id = self.config["frame_id"]; output.pose.position.x, output.pose.position.y, output.pose.position.z = point; output.pose.orientation.x, output.pose.orientation.y, output.pose.orientation.z, output.pose.orientation.w = q_world
            self.publisher.publish(output)
        except Exception: self._invalid(image, camera, "fail-closed detector exception")

if __name__ == "__main__":
    rospy.init_node("qr_camera_detector"); Detector(); rospy.spin()
