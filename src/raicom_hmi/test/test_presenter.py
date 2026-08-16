import pytest
from raicom_hmi.presenter import MAX_RENDERED_UTF8_BYTES, present, render_for_log, semantic_request


def test_presenter_exposes_queue_waits_and_conservative_manual_pages():
    view = present({"mode": "AUTO", "state": "RUNNING", "dispatch_queue": ["pick", "place"],
                    "readiness_reasons": ["localization stale"], "referee": {"state": "WAITING"},
                    "safety": {"state": "STOP_UNCONFIRMED", "estop": False},
                    "capabilities": {"base": {"source": "NOT_CONFIGURED", "mode": "MANUAL", "verification": "HW_UNVERIFIED", "authorization": "NOT_AUTHORIZED"}}})
    assert view["auto"]["dispatch_queue"] == ["pick", "place"]
    assert view["referee"]["waiting"] is True
    assert view["safety"]["stop_unconfirmed"] is True
    assert view["capabilities"][0]["indicator"] != "green"
    assert view["manual"]["device_commands"] == "DISABLED"
    assert view["calibration"]["verification"] == "HW_UNVERIFIED"


def test_oversized_or_deep_input_has_conservative_colors_and_bounded_rendering():
    oversized = '{"mode":"READY","padding":"' + ("x" * 9000) + '"}'
    view = present(oversized)
    assert view["auto"]["mode"] == "UNKNOWN"
    assert view["safety"]["indicator"] == "amber"
    assert all(capability["indicator"] != "green" for capability in view["capabilities"])

    deep = '{"mode":"READY","nested":' + ("[" * 16) + "0" + ("]" * 16) + "}"
    assert present(deep)["safety"]["indicator"] == "amber"
    assert len(render_for_log(oversized).encode("utf-8")) <= MAX_RENDERED_UTF8_BYTES


def test_only_allowlisted_requests_are_represented_not_dispatched():
    assert semantic_request("SOFT_ESTOP") == {"request": "SOFT_ESTOP", "actuation": "NOT_DISPATCHED"}
    with pytest.raises(ValueError):
        semantic_request("BASE_MOVE")
