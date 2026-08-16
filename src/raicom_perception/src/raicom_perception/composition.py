"""SIM-only replay composition boundary; it never produces arm targets."""
from __future__ import annotations
from .api import DetectionResult
from .replay import ReplayDetector, ReplayFrame

class SimReplayPerceptionAdapter:
    """Accept only an explicitly supplied recorded frame and preserve its state."""
    def __init__(self, detector: ReplayDetector) -> None:
        self._detector = detector

    def perceive(self, frame: ReplayFrame) -> DetectionResult:
        if not isinstance(frame, ReplayFrame):
            raise TypeError("SIM replay perception requires an explicit ReplayFrame")
        return self._detector.detect(frame)
