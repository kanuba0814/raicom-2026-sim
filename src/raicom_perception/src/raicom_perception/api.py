"""Detector-neutral perception contracts with explicit unavailable/unknown states."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from pathlib import Path
from typing import Iterable, Optional, Tuple


class ObservationState(str, Enum):
    DETECTED = "DETECTED"
    NOT_DETECTED = "NOT_DETECTED"
    UNKNOWN = "UNKNOWN"
    UNAVAILABLE = "UNAVAILABLE"


@dataclass(frozen=True)
class PerceptionTarget:
    class_name: str
    confidence: float
    image_center_px: Tuple[float, float]
    inside_pick_roi: bool
    stability: float
    estimated_velocity_px_s: Optional[Tuple[float, float]] = None

    def __post_init__(self) -> None:
        values = (self.confidence, self.stability, *self.image_center_px)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("target values must be finite")
        if not 0.0 <= self.confidence <= 1.0 or not 0.0 <= self.stability <= 1.0:
            raise ValueError("confidence and stability must be in [0, 1]")

    def suitable_for_pick(self, minimum_confidence: float, minimum_stability: float) -> bool:
        return (
            self.confidence >= minimum_confidence
            and self.stability >= minimum_stability
            and self.inside_pick_roi
        )


@dataclass(frozen=True)
class DetectionResult:
    state: ObservationState
    targets: Tuple[PerceptionTarget, ...] = ()
    reason: str = ""

    @classmethod
    def detected(cls, targets: Iterable[PerceptionTarget]) -> "DetectionResult":
        collected = tuple(targets)
        if not collected:
            raise ValueError("DETECTED requires at least one target")
        return cls(ObservationState.DETECTED, collected)


@dataclass(frozen=True)
class CodedMarkerResult:
    state: ObservationState
    payload: Optional[str] = None
    marker_id: Optional[int] = None
    image_center_px: Optional[Tuple[float, float]] = None
    reason: str = ""

    def __post_init__(self) -> None:
        if self.state == ObservationState.DETECTED and self.payload is None and self.marker_id is None:
            raise ValueError("detected marker requires payload or marker_id")


@dataclass(frozen=True)
class ModelManifest:
    path: Path
    sha256: str
    classes: Tuple[str, ...]
    verification: str

    def __post_init__(self) -> None:
        if len(self.sha256) != 64 or any(character not in "0123456789abcdef" for character in self.sha256):
            raise ValueError("sha256 must be lowercase hexadecimal")
        if not self.classes or len(set(self.classes)) != len(self.classes):
            raise ValueError("model classes must be non-empty and unique")
        if self.verification not in ("CANDIDATE_PHONE_VALIDATED", "SIM_TESTED", "FIELD_VALIDATED"):
            raise ValueError("unsupported verification level")


class DetectorBackend:
    """Semantic detector boundary; implementations must not expose model APIs."""

    def detect(self, image: object) -> DetectionResult:
        raise NotImplementedError


class UnavailableDetector(DetectorBackend):
    def __init__(self, reason: str) -> None:
        self._reason = reason

    def detect(self, image: object) -> DetectionResult:
        return DetectionResult(ObservationState.UNAVAILABLE, reason=self._reason)
