import pathlib
import unittest

from raicom_workbench_vision.payload_pick import (
    CameraInfoObservation,
    Footprint,
    PayloadProjector,
    PayloadRequestGate,
    SupportPlane,
)

C = {"output_frame": "hand_cam_link", "stale_after_s": .5,
     "max_input_skew_s": .1, "min_normal_camera_dot": .05,
     "tcp_clearance_m": .02578, "min_ray_plane_dot": .05,
     "min_quality": .01, "required_stable_frames": 2,
     "max_center_delta_px": 3., "allowlisted_payloads": ["SIM_QR_01", "SIM_QR_04"]}
I = CameraInfoObservation((100, 0, 50, 0, 100, 50, 0, 0, 1),
                          "hand_cam_link", 1., 100, 100)
P = SupportPlane("hand_cam_link", 1., (0, 0, 1), (0, 0, 1))


def f(label):
    return Footprint(label, (50., 50.), ((40, 40), (60, 40), (60, 60), (40, 60)),
                     1., "opencv_qr")


class Test(unittest.TestCase):
    def test_projection_requires_only_camera_info_and_calibrated_plane(self):
        p = PayloadProjector(C)
        a = p.process([f("SIM_QR_01"), f("SIM_QR_04")], P, I, 1., "hand_cam_link", 100, 100, 1.1)
        b = p.process([f("SIM_QR_01")], SupportPlane("hand_cam_link", 1.1, (0, 0, 1), (0, 0, 1)), CameraInfoObservation(I.k, I.frame_id, 1.1, 100, 100), 1.1, "hand_cam_link", 100, 100, 1.2)
        self.assertEqual(a["SIM_QR_01"].support_normal, (0., 0., -1.))
        self.assertTrue(b["SIM_QR_01"].valid)
        self.assertEqual(b["SIM_QR_04"].detail, "PAYLOAD_ABSENT")

    def test_fail_closed_for_stale_duplicate_or_invalid_inputs(self):
        p = PayloadProjector(C)
        duplicate = p.process([f("SIM_QR_01"), f("SIM_QR_01")], P, I, 1., "hand_cam_link", 100, 100, 1.1)
        self.assertEqual(duplicate["SIM_QR_01"].detail, "AMBIGUOUS_DUPLICATE_LABEL")
        stale = p.process([f("SIM_QR_01")], P, I, 1., "hand_cam_link", 100, 100, 1.2)
        self.assertEqual(stale["SIM_QR_01"].detail, "NONINCREASING_IMAGE_STAMP")
        bad = PayloadProjector(C).process([f("SIM_QR_01")], P, CameraInfoObservation(I.k, "wrong", 1., 100, 100), 1., "hand_cam_link", 100, 100, 1.1)
        self.assertEqual(bad["SIM_QR_01"].detail, "INVALID_CAMERA_INFO")

    def test_request_gate_is_bounded_and_rejects_unknown_or_duplicate_completion(self):
        gate = PayloadRequestGate(("SIM_QR_01",), 2)
        self.assertFalse(gate.begin(1, "OTHER"))
        self.assertFalse(gate.begin(0, "SIM_QR_01"))
        self.assertFalse(gate.accepts_frame())
        self.assertTrue(gate.begin(1, "SIM_QR_01"))
        projector = PayloadProjector(C)
        projector.process([f("SIM_QR_01")], P, I, 1., "hand_cam_link", 100, 100, 1.1)
        self.assertTrue(projector.reset("SIM_QR_01"))
        self.assertFalse(projector.reset("OTHER"))
        reset_result = projector.process(
            [f("SIM_QR_01")], SupportPlane("hand_cam_link", .5, (0, 0, 1), (0, 0, 1)),
            CameraInfoObservation(I.k, I.frame_id, .5, 100, 100), .5,
            "hand_cam_link", 100, 100, .6)
        self.assertNotEqual(reset_result["SIM_QR_01"].detail,
                            "NONINCREASING_IMAGE_STAMP")
        self.assertFalse(gate.begin(2, "SIM_QR_01"))
        self.assertEqual(gate.take_frame(), (1, "SIM_QR_01"))
        self.assertTrue(gate.accepts_frame())
        self.assertFalse(gate.complete(2, "SIM_QR_01"))
        self.assertFalse(gate.complete(1, "OTHER"))
        self.assertTrue(gate.complete(1, "SIM_QR_01"))
        self.assertFalse(gate.accepts_frame())
        self.assertIsNone(gate.take_frame())

    def test_core_and_standalone_launch_have_no_ar_or_gazebo_dependency(self):
        root = pathlib.Path(__file__).parents[1]
        core = (root / "src" / "raicom_workbench_vision" / "payload_pick.py").read_text().lower()
        launch = (root / "launch" / "camera_payload_pick.launch").read_text().lower()
        node = (root / "scripts" / "camera_payload_pick_detector.py").read_text()
        self.assertNotIn("import gazebo", core)
        self.assertNotIn("from gazebo", core)
        self.assertNotIn("markerpose", core)
        self.assertNotIn('pkg="ar_track_alvar"', launch)
        self.assertNotIn('pkg="gazebo_ros"', launch)
        self.assertIn("CameraPayloadPickRequest", node)
        self.assertIn("self.processing_lock", node)
        self.assertIn("output.request_id = request_id", node)
        self.assertNotIn("tcp_pick_pose", node)


if __name__ == "__main__":
    unittest.main()
