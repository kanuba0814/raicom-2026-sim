import pathlib
import unittest

import yaml

from raicom_swiftpro_task_sim.payload_pick import (
    build_motion_plan,
    cartesian_line,
    execute_motion_plan,
    request_from_config,
    sequence,
)


class CameraPayloadPickContractTest(unittest.TestCase):
    def setUp(self):
        self.root = pathlib.Path(__file__).parents[1]
        self.config = yaml.safe_load(
            (self.root / "config" / "camera_payload_pick_executor.yaml").read_text()
        )
        self.script = (
            self.root / "scripts" / "swiftpro_sim_camera_payload_pick.py"
        ).read_text()

    def test_exact_mapping_and_mandatory_rear_order(self):
        qr01 = request_from_config(self.config, "SIM_QR_01")
        qr04 = request_from_config(self.config, "SIM_QR_04")
        self.assertEqual((qr01.source_slot, qr01.rear_slot), (1, 1))
        self.assertEqual((qr04.source_slot, qr04.rear_slot), (4, 2))
        self.assertIn("REAR_PLACE", sequence(qr01, True))
        self.assertNotIn("--place-rear", self.script)

    def test_shared_request_contract_accepts_configured_real_payload_name(self):
        config = {"payloads": {"QR_COMPETITION_A": {"source_slot": 3, "rear_slot": 2}}}
        request = request_from_config(config, "QR_COMPETITION_A")
        self.assertEqual((request.payload, request.source_slot, request.rear_slot),
                         ("QR_COMPETITION_A", 3, 2))

    def test_cartesian_line_is_straight_and_bounded(self):
        points = cartesian_line((0.10, -0.20, 0.04), (0.13, -0.16, 0.08), 0.01)
        self.assertGreater(len(points), 1)
        self.assertEqual(points[-1], (0.13, -0.16, 0.08))
        start = (0.10, -0.20, 0.04)
        direction = (0.03, 0.04, 0.04)
        length2 = sum(value * value for value in direction)
        previous = start
        for point in points:
            delta = tuple(point[i] - start[i] for i in range(3))
            scale = sum(delta[i] * direction[i] for i in range(3)) / length2
            perpendicular = tuple(delta[i] - scale * direction[i] for i in range(3))
            self.assertLess(max(abs(value) for value in perpendicular), 1e-12)
            self.assertLessEqual(__import__("math").dist(previous, point), 0.0100001)
            previous = point

    def test_platform_neutral_transaction_requires_verified_settle_and_vacuum(self):
        class FakeBackend:
            def __init__(self):
                self.calls = []
                self.fail_stage = None
            def set_arm_extended(self, value): self.calls.append(("extended", value))
            def preflight(self, stages): self.calls.append(("preflight", len(stages))); return True
            def move_and_settle(self, target, stage):
                self.calls.append(("move_and_settle", stage)); return stage != self.fail_stage
            def set_vacuum_verified(self, value): self.calls.append(("vacuum_verified", value)); return True
            def home_and_settle(self): self.calls.append(("home_and_settle",)); return True

        plan = build_motion_plan((0.2, 0.0, 0.05), (0.0, 0.0, 1.0),
                                 (0.1, 0.1, 0.12), (0.1, 0.1, 0.04),
                                 0.04, 0.04, 0.01)
        backend = FakeBackend()
        result = execute_motion_plan(backend, plan)
        self.assertEqual((result.status, result.reason, result.arm_extended),
                         ("REFEREE_HOLD", "COMPLETE", False))
        self.assertIn(("vacuum_verified", True), backend.calls)
        self.assertEqual(backend.calls.count(("vacuum_verified", False)), 2)
        self.assertIn(("home_and_settle",), backend.calls)

        backend = FakeBackend()
        backend.fail_stage = "SOURCE_LIFT_01"
        held = execute_motion_plan(backend, plan)
        self.assertEqual((held.status, held.reason, held.grasped, held.arm_extended),
                         ("SAFE_HOLD", "POST_GRASP_MOTION_FAILED", True, True))
        self.assertEqual(backend.calls.count(("vacuum_verified", False)), 1)
        self.assertLess(backend.calls.index(("vacuum_verified", False)),
                        backend.calls.index(("vacuum_verified", True)))
        self.assertNotIn(("home_and_settle",), backend.calls)

    def test_executor_live_safety_contract(self):
        lowered = self.script.lower()
        self.assertNotIn("gazebo_msgs", lowered)
        self.assertNotIn("/gazebo/", lowered)
        self.assertIn('ARM_EXTENDED_TOPIC = "/raicom/safety/arm_extended"', self.script)
        self.assertIn("transformVector3", self.script)
        self.assertIn("stamp > after_s", self.script)
        self.assertIn("build_motion_plan(", self.script)
        self.assertIn("execute_motion_plan(self, plan)", self.script)
        self.assertIn("CAMERA_PAYLOAD_SNAPSHOT_FROZEN", self.script)
        self.assertIn("self.target_samples = []", self.script)
        self.assertIn("CameraPayloadPickRequest(", self.script)
        self.assertIn("sample.request_id != request_id", self.script)
        self.assertIn("stamp <= requested_at", self.script)
        self.assertIn("request_payload", self.script)
        self.assertIn("target.tcp_pick_point", self.script)
        self.assertNotIn("tcp_pick_pose", self.script)
        self.assertIn("snapshot_max_frames", self.script)
        self.assertIn("retaining vacuum", self.script)
        self.assertIn("if not self.vacuum(False):", self.script)
        self.assertIn("def move_and_settle", self.script)
        self.assertIn("def set_vacuum_verified", self.script)
        self.assertIn("def home_and_settle", self.script)


if __name__ == "__main__":
    unittest.main()
