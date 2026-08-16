#!/usr/bin/env python3
"""Package-local regression gates for the opt-in SIM motion adapter."""
import math
import unittest

from raicom_swiftpro_task_sim.kinematics import DEG, display_projection
from raicom_swiftpro_task_sim.motion_adapter import (
    CONTROLLER_JOINT_NAMES,
    FEEDBACK_JOINT_NAMES,
    MAX_VELOCITY_RAD_S,
    STEP_SECONDS,
    JointObservation,
    MotionStatus,
    SwiftproSimMotionAdapter,
    plan_motion,
)


def observation(stamp=10.0, independent=(90 * DEG, 60 * DEG, 50 * DEG),
                velocities=None, include_virtual=True):
    names = FEEDBACK_JOINT_NAMES
    positions = display_projection(independent)
    if include_virtual:
        names += ("virtual_end_joint",)
        positions += (0.0,)
    return JointObservation(names, positions,
                            velocities or (0.0,) * len(names), stamp)


class MotionAdapterTest(unittest.TestCase):
    def test_passive_wrist_projection_keeps_tool_pitch_invariant(self):
        projection = display_projection((90 * DEG, 60 * DEG, 50 * DEG))
        self.assertAlmostEqual(
            projection[1] + projection[2] + projection[7],
            0.0,
            places=12,
        )

    def test_plan_starts_all_channels_from_measured_joint_positions(self):
        measured = list(display_projection((90 * DEG, 60 * DEG, 50 * DEG)))
        measured[7] += 1.0
        start = JointObservation(
            FEEDBACK_JOINT_NAMES,
            tuple(measured),
            (0.0,) * len(FEEDBACK_JOINT_NAMES),
            10.0,
        )
        result = plan_motion((70 * DEG, 75 * DEG, 45 * DEG), start, 10.1)
        self.assertEqual(result.status, MotionStatus.READY)
        first = result.plan.commands[0]
        self.assertLessEqual(
            abs(first[7] - measured[7]) / STEP_SECONDS,
            MAX_VELOCITY_RAD_S + 1e-9,
        )

    def test_feedback_superset_and_all_projected_channels_are_rate_limited(self):
        start = (0.0, 0.0, 0.0)
        target = (math.radians(200), math.radians(135), math.radians(100))
        result = plan_motion(target, observation(independent=start), 10.1)
        self.assertEqual(result.status, MotionStatus.READY)
        commands = (display_projection(start),) + result.plan.commands
        for previous, command in zip(commands, commands[1:]):
            self.assertLessEqual(
                max(abs(a - b) / STEP_SECONDS for a, b in zip(previous, command)),
                MAX_VELOCITY_RAD_S + 1e-9,
            )

    def test_mid_batch_cancel_publishes_fresh_nine_joint_hold(self):
        adapter = SwiftproSimMotionAdapter()
        clock = [10.0]
        cancelled = [False]
        calls = []

        def publish(topic, value):
            calls.append((topic, value))
            if len(calls) == 3:
                cancelled[0] = True

        result = adapter.execute(
            (90 * DEG, 60 * DEG, 50 * DEG), observation(), 20.0, publish,
            lambda _: clock.__setitem__(0, clock[0] + STEP_SECONDS),
            lambda: observation(clock[0]), lambda: cancelled[0], lambda: clock[0],
        )
        self.assertEqual(result.status, MotionStatus.CANCELLED)
        self.assertEqual(len(calls), 12)
        self.assertEqual(
            [topic for topic, _ in calls[-9:]],
            ["/swiftpro/%s_position_controller/command" % name
             for name in CONTROLLER_JOINT_NAMES],
        )

    def test_mid_batch_deadline_publishes_fresh_nine_joint_hold(self):
        adapter = SwiftproSimMotionAdapter()
        clock = [10.0]
        calls = []

        def publish(topic, value):
            calls.append((topic, value))
            if len(calls) == 3:
                clock[0] = 20.0

        result = adapter.execute(
            (90 * DEG, 60 * DEG, 50 * DEG), observation(), 20.0, publish,
            lambda _: clock.__setitem__(0, clock[0] + STEP_SECONDS),
            lambda: observation(clock[0]), lambda: False, lambda: clock[0],
        )
        self.assertEqual((result.status, result.reason),
                         (MotionStatus.TIMEOUT, "TRAJECTORY_TIMEOUT"))
        self.assertEqual(len(calls), 12)

    def test_settle_requires_distinct_stamps_over_complete_window(self):
        adapter = SwiftproSimMotionAdapter()
        clock = [10.0]
        repeated = observation(10.4)
        result = adapter.execute(
            (90 * DEG, 60 * DEG, 50 * DEG), observation(), 20.0,
            lambda *_: None,
            lambda _: clock.__setitem__(0, clock[0] + STEP_SECONDS),
            lambda: repeated, lambda: False, lambda: clock[0],
        )
        self.assertEqual(result.status, MotionStatus.FAILED)
        self.assertIn(result.reason, ("NONINCREASING_SETTLE_STAMP", "UNSAFE_HOLD"))


if __name__ == "__main__":
    unittest.main()
