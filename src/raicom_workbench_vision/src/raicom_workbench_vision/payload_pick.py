"""Camera-only QR payload projection; deliberately contains no Gazebo API."""
from __future__ import annotations
from dataclasses import dataclass
from math import isfinite, sqrt
from typing import Iterable, Protocol, Sequence, Tuple

Vector = Tuple[float, float, float]


@dataclass
class PayloadRequestGate:
    """Bound QR work and projection output to one explicit camera request."""
    allowlisted_payloads: Tuple[str, ...]
    max_frames: int
    payload: str = ""
    request_id: int = 0
    remaining_frames: int = 0

    def begin(self, request_id: object, payload: object) -> bool:
        if (self.accepts_frame() or not isinstance(request_id, int)
                or request_id <= 0 or not isinstance(payload, str)
                or payload not in self.allowlisted_payloads or self.max_frames <= 0):
            return False
        self.payload = payload
        self.request_id = request_id
        self.remaining_frames = self.max_frames
        return True

    def accepts_frame(self) -> bool:
        return bool(self.payload) and self.request_id > 0 and self.remaining_frames > 0

    def take_frame(self) -> tuple[int, str] | None:
        """Reserve one frame for the current generation."""
        if not self.accepts_frame():
            return None
        token = (self.request_id, self.payload)
        self.remaining_frames -= 1
        if self.remaining_frames == 0:
            self.payload = ""
            self.request_id = 0
        return token

    def complete(self, request_id: int, payload: str) -> bool:
        """Close only the request generation that produced the accepted result."""
        if request_id != self.request_id or payload != self.payload:
            return False
        self.payload = ""
        self.request_id = 0
        self.remaining_frames = 0
        return True

@dataclass(frozen=True)
class CameraInfoObservation:
    k: Tuple[float, ...]; frame_id: str; stamp: float; width: int; height: int

@dataclass(frozen=True)
class SupportPlane:
    frame_id: str; stamp: float; point: Vector; normal: Vector

@dataclass(frozen=True)
class Footprint:
    payload: str; center_px: Tuple[float, float]; corners_px: Tuple[Tuple[float, float], ...]; quality: float; detector: str

class FootprintDetector(Protocol):
    def detect(self, image: object) -> Sequence[Footprint]: ...

@dataclass(frozen=True)
class PayloadPick:
    valid: bool; frame_id: str; stamp: float; payload: str = ""; detector: str = ""
    pixel_center: Tuple[float, float] = (0.0, 0.0); surface_point: Vector = (0.0, 0.0, 0.0)
    support_normal: Vector = (0.0, 0.0, 0.0); tcp_point: Vector = (0.0, 0.0, 0.0)
    quality: float = 0.0; stable_frames: int = 0; detail: str = ""

def _dot(a: Vector, b: Vector) -> float: return sum(x * y for x, y in zip(a, b))
def _unit(a: Vector) -> Vector | None:
    length = sqrt(_dot(a, a))
    return None if not isfinite(length) or length < 1e-9 else tuple(x / length for x in a)  # type: ignore[return-value]
class PayloadProjector:
    """Independent per-label stability; every input image produces each label's state."""
    def __init__(self, config: dict) -> None:
        self.config = config; self._last_image_stamp = 0.0
        self._state = {payload: (None, 0) for payload in config["allowlisted_payloads"]}

    def reset(self, payload: str) -> bool:
        """Start a new bounded request without carrying stability from an old one."""
        if payload not in self._state:
            return False
        self._state[payload] = (None, 0)
        self._last_image_stamp = 0.0
        return True

    def _invalid(self, payload: str, stamp: float, detail: str, frame: str) -> PayloadPick:
        self._state[payload] = (None, 0)
        return PayloadPick(False, frame, stamp, payload=payload, detail=detail)

    def process(self, footprints: Iterable[Footprint], support_plane: SupportPlane, camera: CameraInfoObservation, image_stamp: float, image_frame: str, image_width: int, image_height: int, now: float) -> dict[str, PayloadPick]:
        payloads = self.config["allowlisted_payloads"]
        def all_invalid(reason: str): return {p: self._invalid(p, image_stamp, reason, image_frame) for p in payloads}
        if not all(isfinite(x) for x in (image_stamp, now)) or image_stamp <= 0 or image_stamp > now: return all_invalid("INVALID_IMAGE_STAMP")
        if image_stamp <= self._last_image_stamp: return all_invalid("NONINCREASING_IMAGE_STAMP")
        self._last_image_stamp = image_stamp
        if not image_frame or image_frame != self.config["output_frame"]: return all_invalid("IMAGE_FRAME_MISMATCH")
        if now-image_stamp > self.config["stale_after_s"]: return all_invalid("STALE_IMAGE")
        if (len(camera.k) != 9 or not all(isfinite(x) for x in camera.k) or camera.frame_id != image_frame or camera.width != image_width or camera.height != image_height or camera.stamp <= 0 or camera.stamp > now or now-camera.stamp > self.config["stale_after_s"]): return all_invalid("INVALID_CAMERA_INFO")
        if abs(camera.stamp-image_stamp) > self.config["max_input_skew_s"]: return all_invalid("SKEWED_INPUTS")
        if (not isinstance(support_plane, SupportPlane)
                or support_plane.frame_id != image_frame
                or support_plane.stamp <= 0.0
                or support_plane.stamp > now
                or abs(support_plane.stamp - image_stamp) > self.config["max_input_skew_s"]
                or not all(isfinite(value) for value in support_plane.point + support_plane.normal)):
            return all_invalid("INVALID_CALIBRATED_SUPPORT_PLANE")
        normal = _unit(support_plane.normal)
        if normal is None:
            return all_invalid("INVALID_CALIBRATED_SUPPORT_PLANE")
        point = support_plane.point
        camera_dot = _dot(normal, tuple(-value for value in point))
        if abs(camera_dot) < self.config["min_normal_camera_dot"]:
            return all_invalid("AMBIGUOUS_SUPPORT_NORMAL")
        if camera_dot < 0.0:
            normal = tuple(-value for value in normal)
        found = {p: [] for p in payloads}
        for footprint in footprints:
            if footprint.payload in found: found[footprint.payload].append(footprint)
        output = {}
        fx, fy, cx, cy = camera.k[0], camera.k[4], camera.k[2], camera.k[5]
        if fx <= 0 or fy <= 0: return all_invalid("INVALID_CAMERA_INFO")
        for payload in payloads:
            candidates = found[payload]
            if len(candidates) != 1: output[payload] = self._invalid(payload, image_stamp, "AMBIGUOUS_DUPLICATE_LABEL" if candidates else "PAYLOAD_ABSENT", image_frame); continue
            item = candidates[0]; values = (*item.center_px, item.quality, *(v for c in item.corners_px for v in c))
            if len(item.corners_px) != 4 or not item.detector or not all(isfinite(v) for v in values) or item.quality < self.config["min_quality"]: output[payload] = self._invalid(payload, image_stamp, "MALFORMED_FOOTPRINT", image_frame); continue
            ray = _unit(((item.center_px[0]-cx)/fx, (item.center_px[1]-cy)/fy, 1.0)); denominator = _dot(normal, ray) if ray else 0
            distance = _dot(normal, point)/denominator if denominator else 0
            if ray is None or abs(denominator) < self.config["min_ray_plane_dot"] or distance <= 0 or not isfinite(distance): output[payload] = self._invalid(payload, image_stamp, "RAY_PLANE_FAILURE", image_frame); continue
            surface = tuple(distance*x for x in ray); tcp = tuple(surface[i]+normal[i]*self.config["tcp_clearance_m"] for i in range(3))
            old_center, stable = self._state[payload]
            stable = stable+1 if old_center is not None and sqrt(sum((item.center_px[i]-old_center[i])**2 for i in range(2))) <= self.config["max_center_delta_px"] else 1
            self._state[payload] = (item.center_px, stable); valid = stable >= self.config["required_stable_frames"]
            output[payload] = PayloadPick(valid, image_frame, image_stamp, payload, item.detector, item.center_px, surface, normal, tcp, item.quality, stable, "STABLE" if valid else "AWAITING_STABLE_FRAMES")
        return output
