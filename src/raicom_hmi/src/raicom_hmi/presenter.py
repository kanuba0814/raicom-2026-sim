"""Pure HMI view/controller model.  It deliberately cannot command robot devices."""
from __future__ import annotations

import json
from raicom_state_server.mission import normalize_mission_snapshot

MAX_RENDERED_UTF8_BYTES = 8192

ALLOWED_SEMANTIC_REQUESTS = frozenset({
    "START", "PAUSE", "SOFT_ESTOP", "ACK", "REFEREE_ACCEPT", "REFEREE_REJECT",
})
_UNSAFE_VALUES = frozenset({"UNKNOWN", "MISSING", "HW_UNVERIFIED", "NOT_CONFIGURED", "NOT_AUTHORIZED"})


def _indicator(value):
    """Return a conservative HMI color name; unknown state is never green."""
    return "green" if value not in _UNSAFE_VALUES and value in {"OK", "READY", "SIM_TESTED", "AUTHORIZED", "SIM"} else "amber"


def present(snapshot):
    """Convert a semantic snapshot into deterministic, non-actuating HMI data."""
    mission = normalize_mission_snapshot(snapshot)
    capabilities = []
    for name in sorted(mission["capabilities"]):
        raw = mission["capabilities"][name]
        values = [raw["source"], raw["mode"], raw["verification"], raw["authorization"]]
        capabilities.append({"name": name, **raw, "indicator": "green" if all(_indicator(value) == "green" for value in values) else "amber"})
    safety = mission["safety"]
    safety_state = safety["state"]
    stop_unconfirmed = safety_state == "STOP_UNCONFIRMED"
    estop = safety["estop"] is True or safety_state == "ESTOP"
    return {
        "auto": {
            "mode": mission["mode"], "state": mission["state"], "task": mission["task"],
            "primitive": mission["primitive"], "dispatch_queue": mission["dispatch_queue"],
            "readiness_reasons": mission["readiness_reasons"],
            "semantic_requests": sorted(ALLOWED_SEMANTIC_REQUESTS),
        },
        "safety": {**safety, "stop_unconfirmed": stop_unconfirmed, "estop_active": estop,
                   "indicator": "amber" if stop_unconfirmed or estop or safety_state in _UNSAFE_VALUES else _indicator(safety_state)},
        "referee": {**mission["referee"], "waiting": mission["referee"]["state"] == "WAITING"},
        "capabilities": capabilities,
        "manual": {"state": "NOT_CONFIGURED", "verification": "HW_UNVERIFIED", "device_commands": "DISABLED"},
        "calibration": {"state": "NOT_CONFIGURED", "verification": "HW_UNVERIFIED", "device_commands": "DISABLED"},
    }


def render_for_log(snapshot):
    """Serialize a bounded HMI view, retaining only safe UNKNOWN values on overflow."""
    rendered = json.dumps(present(snapshot), sort_keys=True, separators=(",", ":"))
    if len(rendered.encode("utf-8")) <= MAX_RENDERED_UTF8_BYTES:
        return rendered
    return json.dumps({"auto": {"mode": "UNKNOWN", "state": "UNKNOWN"},
                       "safety": {"state": "UNKNOWN", "indicator": "amber"}},
                      sort_keys=True, separators=(",", ":"))


def semantic_request(request):
    """Validate an operator intent without dispatching it to ROS or hardware."""
    if request not in ALLOWED_SEMANTIC_REQUESTS:
        raise ValueError("unsupported semantic request: {}".format(request))
    return {"request": request, "actuation": "NOT_DISPATCHED"}
