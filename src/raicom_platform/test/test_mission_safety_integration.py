import threading
import time
import unittest

from raicom_core.mission import MissionCore, TransitionError
from raicom_core.models import MissionState, OperationMode, SafeHoldEvidence, Task
from raicom_core.runner import MissionRunner
from raicom_platform.api import FakeBackend, OperationResult
from raicom_platform.safety_composition import CoordinatorMissionSafetyAdapter
from raicom_safety.safety_coordinator import AckStatus, AdapterStopAck, CoordinatorMode, SafetyCoordinator, TrustedStopTarget


TARGET = TrustedStopTarget("sim", ("base",))


def acknowledged(token):
    return AdapterStopAck(TARGET, token, AckStatus.ACKED, time.monotonic(), "test")


def running_mission():
    mission = MissionCore(settlement_threshold=1)
    mission.precheck_passed(); mission.wait_for_start(); mission.start()
    mission.assign_task(Task("block", "block"))
    return mission


class MissionSafetyIntegrationTest(unittest.TestCase):
    def test_authoritative_coordinator_maps_only_sim_and_fake(self):
        for mode, evidence in ((CoordinatorMode.SIM, SafeHoldEvidence.CONFIRMED_SIM),
                               (CoordinatorMode.FAKE, SafeHoldEvidence.CONFIRMED_FAKE)):
            coordinator = SafetyCoordinator({TARGET: acknowledged}, mode=mode)
            try:
                result = CoordinatorMissionSafetyAdapter(coordinator).safe_hold(time.monotonic() + 1)
                self.assertEqual(evidence, result.evidence)
            finally:
                coordinator.shutdown()
        shadow = SafetyCoordinator({TARGET: acknowledged}, mode=CoordinatorMode.SHADOW)
        try:
            self.assertFalse(CoordinatorMissionSafetyAdapter(shadow).safe_hold(time.monotonic() + 1).confirmed)
        finally:
            shadow.shutdown()

    def test_bare_confirmed_boolean_is_not_safe_hold_evidence(self):
        class BareConfirmedBackend(FakeBackend):
            def safe_hold(self, deadline_s):
                return type("BareConfirmation", (), {"confirmed": True})()
        mission = running_mission()
        backend = BareConfirmedBackend(navigation_result=OperationResult.FAILED)
        MissionRunner(mission, backend).execute_active_task()
        self.assertEqual(MissionState.ESTOP, mission.state)

    def test_wedged_ack_does_not_delay_mission_estop(self):
        entered = threading.Event()
        def wedge(token):
            entered.set()
            threading.Event().wait(30)
            return acknowledged(token)
        coordinator = SafetyCoordinator({TARGET: wedge}, mode=CoordinatorMode.SIM)
        try:
            mission = running_mission()
            runner = MissionRunner(mission, FakeBackend(), safety_coordinator=CoordinatorMissionSafetyAdapter(coordinator))
            runner.estop("test")
            self.assertEqual(MissionState.ESTOP, mission.state)
            self.assertTrue(entered.wait(1))
        finally:
            coordinator.shutdown(0.01)

    def test_latch_exception_cannot_prevent_estop(self):
        class BrokenCoordinator:
            def latch_estop(self, deadline_s): raise RuntimeError("latch broke")
        mission = running_mission()
        runner = MissionRunner(mission, FakeBackend(), safety_coordinator=BrokenCoordinator())
        runner.estop()
        self.assertEqual(MissionState.ESTOP, mission.state)
        self.assertIn("coordinator=RuntimeError", mission.last_transition_reason)

    def test_failure_exceptions_cancel_once_hold_and_estop(self):
        class ExplodingBackend(FakeBackend):
            def goto(self, waypoint, deadline_s): raise RuntimeError("operation")
            def cancel_active_operation(self):
                self.cancel_calls += 1
                raise RuntimeError("cancel")
            def safe_hold(self, deadline_s): raise RuntimeError("hold")
            def emergency_stop(self): raise RuntimeError("backend estop")
        backend = ExplodingBackend()
        mission = running_mission()
        MissionRunner(mission, backend).execute_active_task()
        self.assertEqual(1, backend.cancel_calls)
        self.assertEqual(MissionState.ESTOP, mission.state)
        self.assertIn("hold_exception", mission.last_transition_reason)

    def test_evidence_exception_cancels_once_and_holds(self):
        class EvidenceBackend(FakeBackend):
            def grasp_evidence(self, target): raise RuntimeError("evidence")
        backend = EvidenceBackend()
        mission = running_mission()
        # pass navigation/perception/grasp then make evidence fail
        MissionRunner(mission, backend).execute_active_task()
        self.assertEqual(1, backend.cancel_calls)
        self.assertEqual(MissionState.RUNNING, mission.state)
        self.assertEqual(1, backend.safe_hold_calls)

    def test_estop_cannot_be_demoted(self):
        mission = running_mission()
        mission.estop()
        for action in (lambda: mission.safe_stop("no"), lambda: mission.stop_unconfirmed("no"),
                       lambda: mission.select_mode(OperationMode.DISPATCH, True)):
            with self.assertRaises(TransitionError):
                action()
        self.assertEqual(MissionState.ESTOP, mission.state)


if __name__ == "__main__":
    unittest.main()
