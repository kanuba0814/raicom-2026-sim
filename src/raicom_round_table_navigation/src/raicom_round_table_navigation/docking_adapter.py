"""ROS-independent fail-closed admission, map identity, and command filtering."""
from dataclasses import dataclass
import hashlib
import math
import os
import struct

SIM_WORKING_ASSUMPTION_CLEARANCE_UNCERTAINTY_M = 0.01


@dataclass(frozen=True)
class AdmissionSnapshot:
    enabled: bool
    backend: str
    team: str
    expected_map_image: str
    expected_grid_hash: str
    map_identity_recorded: bool
    map_identity_matches: bool
    observation_fresh: bool
    observation_stamp_fresh: bool
    valid_fresh: bool
    valid_paired: bool
    scan_fresh: bool
    scan_stamp_fresh: bool
    map_fresh: bool
    tf_fresh: bool
    map_valid: bool
    gate_is_unique_cmd_vel_publisher: bool
    relative_publishers_are_self_only: bool
    move_base_absent: bool
    nav_has_no_publishers: bool


def validate_admission(snapshot):
    """Return a stable fail-closed reason, or ``ADMITTED``."""
    if not snapshot.enabled:
        return "DISABLED"
    if snapshot.backend != "SIM":
        return "BACKEND_NOT_SIM"
    if snapshot.team not in ("blue", "red"):
        return "TEAM_NOT_EXPLICIT"
    image_name = os.path.basename(snapshot.expected_map_image or "").lower()
    if not image_name or snapshot.team not in image_name:
        return "MAP_IMAGE_TEAM_MISMATCH"
    if not snapshot.expected_grid_hash or not snapshot.map_identity_recorded:
        return "MAP_IDENTITY_CONTRACT_MISSING"
    checks = (
        (snapshot.map_identity_matches, "MAP_IDENTITY_MISMATCH"),
        (snapshot.observation_fresh, "OBSERVATION_STALE"),
        (snapshot.observation_stamp_fresh, "OBSERVATION_STAMP_STALE"),
        (snapshot.valid_fresh, "VALID_STALE"),
        (snapshot.valid_paired, "VALID_OBSERVATION_UNPAIRED"),
        (snapshot.scan_fresh, "SCAN_STALE"),
        (snapshot.scan_stamp_fresh, "SCAN_STAMP_STALE"),
        (snapshot.map_fresh, "MAP_STALE"),
        (snapshot.tf_fresh, "TF_STALE"),
        (snapshot.map_valid, "MAP_INVALID"),
        (snapshot.gate_is_unique_cmd_vel_publisher, "CMD_VEL_GATE_NOT_UNIQUE"),
        (snapshot.relative_publishers_are_self_only, "RELATIVE_TOPIC_CONFLICT"),
        (snapshot.move_base_absent, "MOVE_BASE_PRESENT"),
        (snapshot.nav_has_no_publishers, "NAV_PUBLISHER_PRESENT"),
    )
    for passed, reason in checks:
        if not passed:
            return reason
    return "ADMITTED"


def map_grid_from_message(map_message):
    """Convert a ROS-like OccupancyGrid to a guard grid, rejecting bad metadata."""
    from .boundary_guard import OccupancyGridData
    info = map_message.info
    if (getattr(map_message.header, "frame_id", "") != "map" or info.resolution <= 0.0 or
            info.width <= 0 or info.height <= 0 or len(map_message.data) != info.width * info.height or
            any(not isinstance(cell, int) or cell < -1 or cell > 100 for cell in map_message.data)):
        return None
    orientation = info.origin.orientation
    yaw = math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                     1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))
    values = (info.resolution, info.origin.position.x, info.origin.position.y,
              orientation.x, orientation.y, orientation.z, orientation.w, yaw)
    if not all(math.isfinite(value) for value in values) or abs(yaw) > 1e-6:
        return None
    return OccupancyGridData(info.origin.position.x, info.origin.position.y, info.resolution,
                             info.width, info.height, list(map_message.data))


def canonical_occupancy_grid_sha256(map_message):
    """Return the canonical ``sha256:<hex>`` identity for a valid OccupancyGrid.

    The byte sequence is platform-independent: UTF-8 frame with length, unsigned
    dimensions, IEEE-754 big-endian doubles for resolution/origin, then signed
    one-byte occupancy cells. Invalid grids have no identity (``None``).
    """
    if map_grid_from_message(map_message) is None:
        return None
    info = map_message.info
    frame = map_message.header.frame_id.encode("utf-8")
    origin = info.origin
    payload = [struct.pack("!I", len(frame)), frame,
               struct.pack("!QQd", info.width, info.height, info.resolution),
               struct.pack("!7d", origin.position.x, origin.position.y, origin.position.z,
                           origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w),
               struct.pack("!%db" % len(map_message.data), *map_message.data)]
    return "sha256:" + hashlib.sha256(b"".join(payload)).hexdigest()


def base_velocity_to_map(vx_mps, vy_mps, yaw_rad):
    """Rotate a base-frame translational suggestion into map coordinates."""
    return (math.cos(yaw_rad) * vx_mps - math.sin(yaw_rad) * vy_mps,
            math.sin(yaw_rad) * vx_mps + math.cos(yaw_rad) * vy_mps)


def angular_stopping_horizon_s(wz_rps, reaction_time_s, deceleration_rps2):
    """Conservative angular stopping horizon: reaction + v/(2 * deceleration)."""
    if (not all(isinstance(v, (int, float)) and math.isfinite(v) for v in
                (wz_rps, reaction_time_s, deceleration_rps2)) or reaction_time_s < 0.0 or deceleration_rps2 <= 0.0):
        raise ValueError("invalid angular stopping inputs")
    return reaction_time_s + abs(wz_rps) / (2.0 * deceleration_rps2)


def safe_suggestion(suggestion, phase):
    """Accept only pure ALIGN rotation or pure forward APPROACH translation."""
    values = (suggestion.vx_mps, suggestion.vy_mps, suggestion.wz_rps)
    if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in values):
        return False, "SUGGESTION_INVALID"
    phase_name = getattr(phase, "value", phase)
    if phase_name == "ALIGN_CENTER":
        return ((suggestion.vx_mps == 0.0 and suggestion.vy_mps == 0.0),
                "SAFE" if suggestion.vx_mps == 0.0 and suggestion.vy_mps == 0.0 else "ALIGN_REQUIRES_ROTATION_ONLY")
    if phase_name == "APPROACH_CENTER":
        return ((suggestion.vx_mps >= 0.0 and suggestion.vy_mps == 0.0 and suggestion.wz_rps == 0.0),
                "SAFE" if suggestion.vx_mps >= 0.0 and suggestion.vy_mps == 0.0 and suggestion.wz_rps == 0.0 else "APPROACH_REQUIRES_FORWARD_ONLY")
    return (suggestion.vx_mps == 0.0 and suggestion.vy_mps == 0.0 and suggestion.wz_rps == 0.0,
            "SAFE" if suggestion.vx_mps == 0.0 and suggestion.vy_mps == 0.0 and suggestion.wz_rps == 0.0 else "TERMINAL_REQUIRES_ZERO")


class ObservationDropoutGate:
    """Pure consecutive-frame gate: every bad frame stops, repeated bad frames latch."""
    def __init__(self, latch_after_frames=2, recover_after_frames=3):
        if latch_after_frames < 1 or recover_after_frames < 1:
            raise ValueError("observation gate frame counts must be positive")
        self.latch_after_frames = latch_after_frames
        self.recover_after_frames = recover_after_frames
        self.bad_frames = 0
        self.good_frames = 0

    def update(self, valid):
        if valid:
            self.bad_frames = 0
            self.good_frames += 1
            return "ALLOW" if self.good_frames >= self.recover_after_frames else "HOLD_ZERO"
        self.good_frames = 0
        self.bad_frames += 1
        return "LATCH" if self.bad_frames >= self.latch_after_frames else "HOLD_ZERO"

    def reset(self):
        self.bad_frames = 0
        self.good_frames = 0


class FaultLatch:
    def __init__(self):
        self.latched = False
        self.reason = ""

    def fault(self, reason):
        self.latched = True
        self.reason = reason

    def reset(self):
        self.latched = False
        self.reason = ""
