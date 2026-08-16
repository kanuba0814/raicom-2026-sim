"""Perception contracts for RAICOM 2026.

Imports are lazy so replay-only consumers do not execute optional SIM composition.
"""
from __future__ import annotations

from importlib import import_module
from typing import Any

_API_EXPORTS = frozenset({
    "CodedMarkerResult", "DetectionResult", "DetectorBackend", "ModelManifest",
    "ObservationState", "PerceptionTarget", "UnavailableDetector",
})
_REPLAY_EXPORTS = frozenset({
    "ReplayConfig", "ReplayDetection", "ReplayDetector", "ReplayFrame",
    "UnavailableLiveCameraDetector",
})
_COMPOSITION_EXPORTS = frozenset({"SimReplayPerceptionAdapter"})

__all__ = sorted(_API_EXPORTS | _REPLAY_EXPORTS | _COMPOSITION_EXPORTS)


def __getattr__(name: str) -> Any:
    """Resolve public exports without importing optional composition at package load."""
    if name in _API_EXPORTS:
        module = import_module(".api", __name__)
    elif name in _REPLAY_EXPORTS:
        module = import_module(".replay", __name__)
    elif name in _COMPOSITION_EXPORTS:
        module = import_module(".composition", __name__)
    else:
        raise AttributeError("module {!r} has no attribute {!r}".format(__name__, name))
    value = getattr(module, name)
    globals()[name] = value
    return value
