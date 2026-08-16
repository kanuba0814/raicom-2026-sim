"""Deterministic replay-only detector backend.

This module deliberately consumes recorded detector outputs (or image metadata paired
with such outputs).  It does not load an inference model or subscribe to ROS; live
camera integration remains an explicit unavailable seam until its contract is
validated.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from types import MappingProxyType
from typing import Any, Callable, Dict, Iterable, Mapping, Optional, Tuple, Union


_MAX_CLASS_MAPPING_ENTRIES = 64
_MAX_CLASS_NAME_LENGTH = 128
_MAX_MODEL_PATH_LENGTH = 512
_MODEL_VERIFICATIONS = frozenset({"NOT_FIELD_VALIDATED", "UNVERIFIED", "SIM_TESTED"})

from .api import DetectionResult, DetectorBackend, ObservationState, PerceptionTarget


@dataclass(frozen=True)
class ReplayDetection:
    """One raw detector output recorded for a frame; coordinates stay in pixels."""

    raw_class: str
    confidence: float
    bbox_xyxy: Tuple[float, float, float, float]

    def __post_init__(self) -> None:
        if not self.raw_class or not isinstance(self.raw_class, str):
            raise ValueError("raw_class must be a non-empty string")
        if not math.isfinite(self.confidence) or not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be in [0, 1]")
        if len(self.bbox_xyxy) != 4 or not all(math.isfinite(value) for value in self.bbox_xyxy):
            raise ValueError("bbox_xyxy must contain four finite pixel coordinates")
        x1, y1, x2, y2 = self.bbox_xyxy
        if x2 < x1 or y2 < y1:
            raise ValueError("bbox_xyxy must be ordered")


@dataclass(frozen=True)
class ReplayFrame:
    """Recorded frame metadata and optional precomputed detector outputs.

    ``image_path`` is provenance only: this backend never performs image inference.
    """

    frame_id: str
    timestamp_s: float
    detections: Optional[Tuple[ReplayDetection, ...]] = ()
    image_path: Optional[str] = None
    camera_available: bool = True
    malformed: bool = False
    malformed_reason: str = ""

    def __post_init__(self) -> None:
        if not self.frame_id or not math.isfinite(self.timestamp_s):
            raise ValueError("frame_id and finite timestamp_s are required")
        if self.detections is not None and not isinstance(self.detections, tuple):
            raise ValueError("detections must be a tuple or None")


@dataclass(frozen=True)
class ReplayConfig:
    """Migration configuration, including explicit detector-to-material mappings."""

    class_mapping: Mapping[str, str]
    minimum_confidence: float = 0.0
    minimum_stability: float = 1.0
    required_consecutive_frames: int = 1
    maximum_frame_age_s: float = 1.0
    model_path: str = ""
    model_sha256: str = ""
    model_verification: str = "NOT_FIELD_VALIDATED"

    def __post_init__(self) -> None:
        if not isinstance(self.class_mapping, Mapping) or not 1 <= len(self.class_mapping) <= _MAX_CLASS_MAPPING_ENTRIES:
            raise ValueError("class_mapping must contain 1..{} entries".format(_MAX_CLASS_MAPPING_ENTRIES))
        copied_mapping: Dict[str, str] = {}
        for raw, material in self.class_mapping.items():
            if (not isinstance(raw, str) or not isinstance(material, str)
                    or not 1 <= len(raw) <= _MAX_CLASS_NAME_LENGTH
                    or not 1 <= len(material) <= _MAX_CLASS_NAME_LENGTH):
                raise TypeError("class_mapping keys and values must be bounded non-empty strings")
            copied_mapping[raw] = material
        # Copy at the public validation boundary: a frozen dataclass alone does not
        # prevent a caller from changing a retained mutable mapping. String leaves
        # make the resulting mapping deeply immutable.
        object.__setattr__(self, "class_mapping", MappingProxyType(copied_mapping))
        _finite_number(self.minimum_confidence, "minimum_confidence", 0.0, 1.0)
        _finite_number(self.minimum_stability, "minimum_stability", 0.0, 1.0)
        if (isinstance(self.required_consecutive_frames, bool)
                or not isinstance(self.required_consecutive_frames, int)
                or not 1 <= self.required_consecutive_frames <= 1000):
            raise ValueError("required_consecutive_frames must be within 1..1000")
        _finite_number(self.maximum_frame_age_s, "maximum_frame_age_s", 0.0, 3600.0)
        if (not isinstance(self.model_path, str) or len(self.model_path) > _MAX_MODEL_PATH_LENGTH
                or any(ord(character) < 32 or ord(character) == 127 for character in self.model_path)
                or (self.model_path and (Path(self.model_path).is_absolute()
                                         or ".." in Path(self.model_path).parts))):
            raise ValueError("model_path must be a bounded relative development path")
        if (not isinstance(self.model_sha256, str)
                or (self.model_sha256 and (len(self.model_sha256) != 64
                                           or any(c not in "0123456789abcdef" for c in self.model_sha256)))):
            raise ValueError("model_sha256 must be lowercase hexadecimal when supplied")
        if self.model_verification not in _MODEL_VERIFICATIONS:
            raise ValueError("model_verification is unsupported or promotes FIELD_VALIDATED verification")

    @classmethod
    def from_json(cls, path: Union[str, Path]) -> "ReplayConfig":
        """Load JSON-only config to avoid a YAML/runtime dependency."""
        try:
            with Path(path).open(encoding="utf-8") as config_file:
                values = json.load(config_file)
            return cls(**values)
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as error:
            raise ValueError("invalid replay detector configuration: %s" % error) from error


def _finite_number(value: Any, name: str, minimum: float, maximum: float) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise TypeError("{} must be a finite number".format(name))
    if not minimum <= value <= maximum:
        raise ValueError("{} is out of range".format(name))


class ReplayDetector(DetectorBackend):
    """Replay backend with deterministic freshness, confidence, and temporal gates."""

    def __init__(
        self,
        config: ReplayConfig,
        frames: Optional[Mapping[str, ReplayFrame]] = None,
        now: Optional[Callable[[], float]] = None,
        model_available: bool = True,
    ) -> None:
        self.config = config
        self._frames: Dict[str, ReplayFrame] = dict(frames or {})
        self._now = now
        self._model_available = model_available
        self._consecutive_by_class: Dict[str, int] = {}

    @property
    def provenance(self) -> Mapping[str, str]:
        return {
            "model_path": self.config.model_path,
            "model_sha256": self.config.model_sha256,
            "verification": self.config.model_verification,
            "backend": "DETERMINISTIC_REPLAY",
        }

    def detect(self, image: object) -> DetectionResult:
        if not self._model_available:
            self._break_stability()
            return DetectionResult(ObservationState.UNAVAILABLE, reason="MODEL_UNAVAILABLE")
        frame = self._resolve_frame(image)
        if frame is None:
            self._break_stability()
            return DetectionResult(ObservationState.UNKNOWN, reason="FRAME_MISSING")
        if not frame.camera_available:
            self._break_stability()
            return DetectionResult(ObservationState.UNKNOWN, reason="CAMERA_UNAVAILABLE")
        if frame.malformed:
            self._break_stability()
            return DetectionResult(ObservationState.UNKNOWN, reason=frame.malformed_reason or "FRAME_MALFORMED")
        if self._now is not None and self._now() - frame.timestamp_s > self.config.maximum_frame_age_s:
            self._break_stability()
            return DetectionResult(ObservationState.UNKNOWN, reason="FRAME_STALE")
        if frame.detections is None:
            self._break_stability()
            return DetectionResult(ObservationState.UNAVAILABLE, reason="RECORDED_OUTPUT_UNAVAILABLE")
        if any(not isinstance(detection, ReplayDetection) for detection in frame.detections):
            self._break_stability()
            return DetectionResult(ObservationState.UNKNOWN, reason="DETECTION_MALFORMED")

        unmapped = [detection.raw_class for detection in frame.detections if detection.raw_class not in self.config.class_mapping]
        if unmapped:
            self._break_stability()
            return DetectionResult(ObservationState.UNKNOWN, reason="UNMAPPED_CLASS:%s" % unmapped[0])

        accepted = [detection for detection in frame.detections if detection.confidence >= self.config.minimum_confidence]
        current_classes = {self.config.class_mapping[detection.raw_class] for detection in accepted}
        self._update_consecutive_counts(current_classes)
        targets = []
        for detection in accepted:
            class_name = self.config.class_mapping[detection.raw_class]
            count = self._consecutive_by_class[class_name]
            stability = min(1.0, count / float(self.config.required_consecutive_frames))
            if count < self.config.required_consecutive_frames or stability < self.config.minimum_stability:
                continue
            x1, y1, x2, y2 = detection.bbox_xyxy
            targets.append(PerceptionTarget(
                class_name=class_name,
                confidence=detection.confidence,
                image_center_px=((x1 + x2) / 2.0, (y1 + y2) / 2.0),
                inside_pick_roi=False,
                stability=stability,
            ))
        if targets:
            return DetectionResult.detected(targets)
        return DetectionResult(ObservationState.NOT_DETECTED, reason="NO_CONFIDENT_STABLE_MAPPED_TARGET")

    def _resolve_frame(self, image: object) -> Optional[ReplayFrame]:
        if isinstance(image, ReplayFrame):
            return image
        if isinstance(image, str):
            return self._frames.get(image)
        return None

    def _update_consecutive_counts(self, current_classes: Iterable[str]) -> None:
        current = set(current_classes)
        for class_name in set(self._consecutive_by_class).union(current):
            self._consecutive_by_class[class_name] = self._consecutive_by_class.get(class_name, 0) + 1 if class_name in current else 0

    def _break_stability(self) -> None:
        for class_name in self._consecutive_by_class:
            self._consecutive_by_class[class_name] = 0


class UnavailableLiveCameraDetector(DetectorBackend):
    """ROS/live camera seam intentionally unavailable pending a verified contract."""

    def detect(self, image: object) -> DetectionResult:
        return DetectionResult(ObservationState.UNAVAILABLE, reason="LIVE_CAMERA_CONTRACT_UNVERIFIED")
