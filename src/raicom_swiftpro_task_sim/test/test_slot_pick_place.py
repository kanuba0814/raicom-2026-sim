#!/usr/bin/env python3
"""Regression gates for pure SIM-only camera-slot pick/place planning."""
import dataclasses
import math
import unittest

from raicom_swiftpro_task_sim.config import CONFIG
from raicom_swiftpro_task_sim.slot_pick_place import (
    SlotAction, SlotObservation, SlotSequenceStatus, plan_slot_pick_place,
)


BASE = CONFIG.frames["base"]
SLOTS = ((0.20, 0.00, 9.0), (0.20, 0.10, 8.0), (0.18, -0.10, 7.0), (0.15, 0.10, 6.0))


def observation(**overrides):
    values = {"frame_id": BASE, "slots_xyz_m": SLOTS, "observed_at_s": 10.0, "valid": True}
    values.update(overrides)
    return SlotObservation(**values)


class SlotPickPlaceTest(unittest.TestCase):
    def test_camera_slots_supply_only_lateral_alignment_and_config_supplies_heights(self):
        result = plan_slot_pick_place(observation(), 0, 10.1)
        self.assertEqual((result.status, result.reason, result.placement_slot_index),
                         (SlotSequenceStatus.READY, "READY", 1))
        self.assertEqual([step.name for step in result.steps], [
            "ALIGN_SOURCE", "DESCEND_SOURCE", "VACUUM_ON", "LIFT_SOURCE",
            "ALIGN_DESTINATION", "DESCEND_DESTINATION", "VACUUM_OFF", "LIFT_DESTINATION",
        ])
        self.assertEqual([step.action for step in result.steps], [
            SlotAction.MOVE, SlotAction.MOVE, SlotAction.VACUUM_ON, SlotAction.MOVE,
            SlotAction.MOVE, SlotAction.MOVE, SlotAction.VACUUM_OFF, SlotAction.MOVE,
        ])
        move_targets = {step.name: step.target_xyz_m for step in result.steps if step.action is SlotAction.MOVE}
        self.assertEqual(move_targets["ALIGN_SOURCE"], (0.20, 0.00, CONFIG.slot_pick_place["observation_height_m"]))
        self.assertEqual(move_targets["DESCEND_SOURCE"], (0.20, 0.00, CONFIG.slot_pick_place["grasp_height_m"]))
        self.assertEqual(move_targets["ALIGN_DESTINATION"], (0.20, 0.10, CONFIG.slot_pick_place["observation_height_m"]))
        self.assertEqual(move_targets["DESCEND_DESTINATION"], (0.20, 0.10, CONFIG.slot_pick_place["place_height_m"]))
        self.assertTrue(all(step.target_independent_rad is not None for step in result.steps if step.action is SlotAction.MOVE))

    def test_two_configured_destinations_are_selected_by_choice_not_world_coordinate(self):
        first = plan_slot_pick_place(observation(), 0, 10.1)
        second = plan_slot_pick_place(observation(), 1, 10.1)
        self.assertEqual((first.placement_slot_index, second.placement_slot_index), (1, 2))
        self.assertEqual(first.steps[4].target_xyz_m[:2], SLOTS[1][:2])
        self.assertEqual(second.steps[4].target_xyz_m[:2], SLOTS[2][:2])

    def test_invalid_stale_or_wrong_frame_camera_slots_fail_before_any_actions(self):
        cases = (
            (observation(valid=False), 10.1, "SLOTS_INVALID"),
            (observation(frame_id="camera"), 10.1, "SLOT_FRAME_MISMATCH"),
            (observation(observed_at_s=9.0), 10.1, "SLOTS_STALE"),
            (observation(slots_xyz_m=SLOTS[:3]), 10.1, "SLOT_COUNT_MISMATCH"),
        )
        for slots, now, reason in cases:
            with self.subTest(reason=reason):
                result = plan_slot_pick_place(slots, 0, now)
                self.assertNotEqual(result.status, SlotSequenceStatus.READY)
                self.assertEqual(result.reason, reason)
                self.assertEqual(result.steps, ())

    def test_invalid_destination_and_unreachable_approach_fail_closed(self):
        for choice in (-1, 2, True, "0"):
            with self.subTest(choice=choice):
                invalid = plan_slot_pick_place(observation(), choice, 10.1)
                self.assertEqual((invalid.status, invalid.reason, invalid.steps),
                                 (SlotSequenceStatus.INVALID, "INVALID_PLACEMENT_CHOICE", ()))
        unreachable_slots = ((0.01, 0.00, 0.0),) + SLOTS[1:]
        unreachable = plan_slot_pick_place(observation(slots_xyz_m=unreachable_slots), 0, 10.1)
        self.assertEqual(unreachable.status, SlotSequenceStatus.UNREACHABLE)
        self.assertTrue(unreachable.reason.startswith("ALIGN_SOURCE_"))
        self.assertEqual(unreachable.steps, ())
        destination_unreachable = SLOTS[:1] + ((0.01, 0.00, 0.0),) + SLOTS[2:]
        unreachable = plan_slot_pick_place(
            observation(slots_xyz_m=destination_unreachable), 0, 10.1)
        self.assertEqual(unreachable.status, SlotSequenceStatus.UNREACHABLE)
        self.assertTrue(unreachable.reason.startswith("ALIGN_DESTINATION_"))
        self.assertEqual(unreachable.steps, ())

    def test_malformed_observations_and_custom_configs_never_produce_actions(self):
        cases = (
            (observation(observed_at_s=10.2), 10.1, "SLOTS_STALE"),
            (observation(observed_at_s=math.nan), 10.1, "SLOTS_INVALID"),
            (observation(slots_xyz_m=((math.nan, 0.0, 0.0),) + SLOTS[1:]), 10.1, "SLOT_NONFINITE"),
            (observation(slots_xyz_m=((0.2, 0.0),) + SLOTS[1:]), 10.1, "SLOT_NONFINITE"),
        )
        for slots, now, reason in cases:
            with self.subTest(reason=reason):
                result = plan_slot_pick_place(slots, 0, now)
                self.assertEqual(result.reason, reason)
                self.assertEqual(result.steps, ())
        malformed_configs = (
            None,
            dataclasses.replace(CONFIG, slot_pick_place={}),
            dataclasses.replace(CONFIG, slot_pick_place={
                **CONFIG.slot_pick_place, "placement_slot_indices": (1, 1),
            }),
            dataclasses.replace(CONFIG, frames={}),
        )
        for config in malformed_configs:
            with self.subTest(config=config):
                result = plan_slot_pick_place(observation(), 0, 10.1, config)
                self.assertEqual(
                    (result.status, result.reason, result.steps),
                    (SlotSequenceStatus.INVALID, "INVALID_CONFIG", ()),
                )


if __name__ == "__main__":
    unittest.main()
