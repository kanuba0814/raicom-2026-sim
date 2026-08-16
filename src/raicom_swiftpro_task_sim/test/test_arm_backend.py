#!/usr/bin/env python3
"""Package-local regression gate for the non-dispatching real adapter seam."""
import unittest

from raicom_swiftpro_task_sim.arm_backend import (
    ArmMoveRequest,
    BackendStatus,
    RealSwiftProBackend,
)


class RealBackendTest(unittest.TestCase):
    def test_default_is_disabled_and_not_wired(self):
        request = ArmMoveRequest("swiftpro_real_base", (0.2, 0.0, 0.05), 9.9, 40.0)
        result = RealSwiftProBackend().move(request, 10.0)
        self.assertEqual((result.status, result.reason),
                         (BackendStatus.NOT_WIRED, "REAL_ADAPTER_DISABLED"))

    def test_enabled_adapter_prepares_mm_but_still_cannot_dispatch(self):
        default = RealSwiftProBackend()
        config = type(default.config)(
            True,
            default.config.implicit_base_frame,
            default.config.official_worst_case_s,
            default.config.workspace_m,
        )
        backend = RealSwiftProBackend(config)
        request = ArmMoveRequest("swiftpro_real_base", (0.2, 0.0, 0.05), 9.9, 40.0)
        prepared = backend.prepare(request, 10.0)
        self.assertEqual(prepared.status, BackendStatus.REQUEST_PREPARED)
        self.assertEqual(
            (prepared.vendor_request.x_mm, prepared.vendor_request.y_mm,
             prepared.vendor_request.z_mm),
            (200.0, 0.0, 50.0),
        )
        result = backend.move(request, 10.0)
        self.assertEqual(
            (result.status, result.reason, result.settlement),
            (BackendStatus.NOT_WIRED, "COMMISSIONING_DISPATCH_NOT_IMPLEMENTED",
             "NOT_MEASURED"),
        )


if __name__ == "__main__":
    unittest.main()
