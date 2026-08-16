"""Pure, fail-closed geometry for the SIM workbench marker layout."""
from __future__ import annotations

from dataclasses import dataclass
from math import acos, atan2, hypot, isfinite, sqrt
from typing import Dict, Sequence, Tuple

Vector = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


@dataclass(frozen=True)
class MarkerObservation:
    marker_id: int
    frame_id: str
    stamp: float
    position: Vector
    orientation: Quaternion


@dataclass(frozen=True)
class SlotEstimate:
    valid: bool
    stable_frames: int
    separation_m: float
    normal_disagreement_rad: float
    frame_id: str
    slots: Tuple[Tuple[Vector, Quaternion], ...]
    detail: str


@dataclass(frozen=True)
class MarkerBaselinePose:
    heading_rad: float
    signed_lateral_m: float
    along_from_first_m: float
    separation_m: float


def marker_baseline_pose(first: Vector, second: Vector) -> MarkerBaselinePose | None:
    """Measure a planar marker baseline relative to the observation frame origin.

    The fixed arm observation pose is a caller-side precondition. Positive
    lateral is left of the directed first-to-second line; ``along_from_first_m``
    locates the perpendicular projection of the frame origin from ``first``.
    """
    values = first + second
    if len(first) != 3 or len(second) != 3 or not all(isfinite(value) for value in values):
        return None
    dx, dy = second[0] - first[0], second[1] - first[1]
    separation = hypot(dx, dy)
    if separation < 1e-9:
        return None
    ux, uy = dx / separation, dy / separation
    along = -(first[0] * ux + first[1] * uy)
    lateral = ux * (-first[1]) - uy * (-first[0])
    return MarkerBaselinePose(atan2(uy, ux), lateral, along, separation)


def _dot(a: Vector, b: Vector) -> float:
    return sum(x * y for x, y in zip(a, b))


def _cross(a: Vector, b: Vector) -> Vector:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _norm(a: Vector) -> float:
    return sqrt(_dot(a, a))


def _unit(a: Vector) -> Vector | None:
    length = _norm(a)
    if not isfinite(length) or length < 1e-9:
        return None
    return tuple(value / length for value in a)  # type: ignore[return-value]


def _neg(a: Vector) -> Vector:
    return (-a[0], -a[1], -a[2])


def _rotate(q: Quaternion, v: Vector) -> Vector:
    x, y, z, w = q
    # q * (v, 0) * conjugate(q), for a normalized q.
    uv = _cross((x, y, z), v)
    uuv = _cross((x, y, z), uv)
    return tuple(v[i] + 2.0 * (w * uv[i] + uuv[i]) for i in range(3))  # type: ignore[return-value]


def _quaternion_from_axes(x_axis: Vector, y_axis: Vector, z_axis: Vector) -> Quaternion:
    m00, m01, m02 = x_axis[0], y_axis[0], z_axis[0]
    m10, m11, m12 = x_axis[1], y_axis[1], z_axis[1]
    m20, m21, m22 = x_axis[2], y_axis[2], z_axis[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = sqrt(trace + 1.0) * 2.0
        q = ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s)
    elif m00 > m11 and m00 > m22:
        s = sqrt(1.0 + m00 - m11 - m22) * 2.0
        q = (0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    elif m11 > m22:
        s = sqrt(1.0 + m11 - m00 - m22) * 2.0
        q = ((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s)
    else:
        s = sqrt(1.0 + m22 - m00 - m11) * 2.0
        q = ((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s)
    return q  # already unit within floating precision


class WorkbenchLocalizer:
    """Localize fixed slots only after consecutive valid SIM marker observations."""

    def __init__(self, config: Dict[str, object]) -> None:
        self.config = config
        self.stable_frames = 0
        self._last_stamps: Tuple[float, float] | None = None
        self._last_frame = ""
        self._last_slots: Tuple[Tuple[Vector, Quaternion], ...] | None = None
        slot_ids = [slot["id"] for slot in self.config["slots"]]  # type: ignore[index,union-attr]
        if slot_ids != [1, 2, 3, 4]:
            raise ValueError("slots must be explicitly ordered as IDs 1,2,3,4")

    def _invalid(self, detail: str, separation: float = 0.0, disagreement: float = 0.0, frame: str = "") -> SlotEstimate:
        self.stable_frames = 0
        return SlotEstimate(False, 0, separation, disagreement, frame, (), detail)

    def process(self, markers: Sequence[MarkerObservation], now: float) -> SlotEstimate:
        marker_ids = self.config["marker_ids"]  # type: ignore[index]
        top_id, bottom_id = marker_ids["top"], marker_ids["bottom"]  # type: ignore[index]
        selected = [item for item in markers if item.marker_id in (top_id, bottom_id)]
        if len([item for item in selected if item.marker_id == top_id]) != 1 or len([item for item in selected if item.marker_id == bottom_id]) != 1:
            return self._invalid("missing or duplicate configured marker")
        top = next(item for item in selected if item.marker_id == top_id)
        bottom = next(item for item in selected if item.marker_id == bottom_id)
        if top.frame_id != bottom.frame_id or not top.frame_id:
            return self._invalid("markers must share a non-empty frame")
        if top.frame_id != self.config["output_frame"]:
            return self._invalid("unexpected marker output frame", frame=top.frame_id)
        if top.stamp <= 0.0 or bottom.stamp <= 0.0 or top.stamp > now or bottom.stamp > now:
            return self._invalid("invalid marker timestamp", frame=top.frame_id)
        if now - top.stamp > self.config["stale_after_s"] or now - bottom.stamp > self.config["stale_after_s"]:  # type: ignore[operator]
            return self._invalid("stale marker observation", frame=top.frame_id)
        if abs(top.stamp - bottom.stamp) > self.config["max_marker_skew_s"]:  # type: ignore[operator]
            return self._invalid("marker timestamps excessively skewed", frame=top.frame_id)
        if self._last_stamps is not None and (top.stamp <= self._last_stamps[0] or bottom.stamp <= self._last_stamps[1]):
            return self._invalid("marker observation is not fresh", frame=top.frame_id)
        values = (*top.position, *bottom.position, *top.orientation, *bottom.orientation, top.stamp, bottom.stamp)
        if not all(isfinite(value) for value in values):
            return self._invalid("non-finite marker observation", frame=top.frame_id)
        top_q_norm, bottom_q_norm = _norm(top.orientation), _norm(bottom.orientation)
        if abs(top_q_norm - 1.0) > self.config["quaternion_norm_tolerance"] or abs(bottom_q_norm - 1.0) > self.config["quaternion_norm_tolerance"]:  # type: ignore[operator]
            return self._invalid("invalid marker quaternion norm", frame=top.frame_id)
        q_top, q_bottom = _unit(top.orientation), _unit(bottom.orientation)
        if q_top is None or q_bottom is None:
            return self._invalid("invalid marker quaternion", frame=top.frame_id)
        delta = tuple(bottom.position[i] - top.position[i] for i in range(3))
        separation = _norm(delta)
        if not self.config["min_separation_m"] <= separation <= self.config["max_separation_m"]:  # type: ignore[operator]
            return self._invalid("implausible marker separation", separation, frame=top.frame_id)
        normal_axis = tuple(self.config["marker_normal_axis"])  # type: ignore[arg-type]
        expected_normal = _unit(tuple(self.config["expected_normal_in_output_frame"]))  # type: ignore[arg-type]
        q_top, q_bottom = _unit(top.orientation), _unit(bottom.orientation)
        if q_top is None or q_bottom is None or expected_normal is None:
            return self._invalid("invalid marker quaternion or normal anchor", separation, frame=top.frame_id)
        top_normal = _unit(_rotate(q_top, normal_axis))
        bottom_normal = _unit(_rotate(q_bottom, normal_axis))
        if top_normal is None or bottom_normal is None:
            return self._invalid("degenerate marker normal", separation, frame=top.frame_id)
        # Anchor each normal explicitly in the configured output frame, not merely to each other.
        if _dot(top_normal, expected_normal) < 0.0:
            top_normal = _neg(top_normal)
        if _dot(bottom_normal, expected_normal) < 0.0:
            bottom_normal = _neg(bottom_normal)
        disagreement = acos(max(-1.0, min(1.0, _dot(top_normal, bottom_normal))))
        if disagreement > self.config["max_normal_disagreement_rad"]:  # type: ignore[operator]
            return self._invalid("marker normal disagreement", separation, disagreement, top.frame_id)
        normal = _unit(tuple(top_normal[i] + bottom_normal[i] for i in range(3)))
        if normal is None or _dot(normal, expected_normal) < self.config["min_normal_anchor_dot"]:  # type: ignore[operator]
            return self._invalid("ambiguous normal sign", separation, disagreement, top.frame_id)
        # Project the top-to-bottom baseline into the surface plane; reject a tilted baseline.
        normal_component = _dot(delta, normal)
        raw_down = tuple(delta[i] - normal_component * normal[i] for i in range(3))
        down = _unit(raw_down)
        if down is None or abs(normal_component) / separation > self.config["max_baseline_normal_ratio"]:  # type: ignore[operator]
            return self._invalid("baseline excessively out of surface plane", separation, disagreement, top.frame_id)
        # Right-handed surface axes: x=left, y=down, z=outward; x cross y = z.
        left = _unit(_cross(down, normal))
        if left is None:
            return self._invalid("degenerate surface frame", separation, disagreement, top.frame_id)
        normal = _unit(_cross(left, down))
        if normal is None or _dot(_cross(left, down), normal) < 1.0 - 1e-8:
            return self._invalid("non-right-handed surface frame", separation, disagreement, top.frame_id)
        orientation = _quaternion_from_axes(left, down, normal)
        slots = []
        for slot in self.config["slots"]:  # type: ignore[union-attr]
            position = tuple(top.position[i] + left[i] * slot["left_m"] + down[i] * slot["down_m"] + normal[i] * self.config["surface_offset_m"] for i in range(3))  # type: ignore[index,operator]
            slots.append((position, orientation))
        current_slots = tuple(slots)
        stable_delta = self._last_slots is None
        if self._last_slots is not None:
            max_translation = max(_norm(tuple(current_slots[i][0][axis] - self._last_slots[i][0][axis] for axis in range(3))) for i in range(4))
            q_now, q_previous = current_slots[0][1], self._last_slots[0][1]
            quaternion_dot = abs(sum(q_now[i] * q_previous[i] for i in range(4)))
            angular_delta = 2.0 * acos(max(-1.0, min(1.0, quaternion_dot)))
            stable_delta = max_translation <= self.config["max_slot_translation_delta_m"] and angular_delta <= self.config["max_surface_angle_delta_rad"]  # type: ignore[operator]
        if self._last_frame != top.frame_id or not stable_delta:
            self.stable_frames = 0
        self._last_stamps = (top.stamp, bottom.stamp)
        self._last_frame = top.frame_id
        self._last_slots = current_slots
        self.stable_frames += 1
        required = self.config["required_stable_frames"]  # type: ignore[index]
        detail = "stable" if self.stable_frames >= required else ("unstable derived pose" if not stable_delta else "awaiting stable frames")
        return SlotEstimate(self.stable_frames >= required, self.stable_frames, separation, disagreement, top.frame_id, current_slots, detail)
