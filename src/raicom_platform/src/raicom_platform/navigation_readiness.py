"""Pure evaluation for a read-only navigation/alignment contract probe."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Iterable, Mapping, Tuple


class ContractStatus(str, Enum):
    NOT_CONFIGURED = "NOT_CONFIGURED"
    INTERFACE_COMPATIBLE = "INTERFACE_COMPATIBLE"
    UNAVAILABLE = "UNAVAILABLE"


@dataclass(frozen=True)
class NavigationReadiness:
    """Observed graph evidence only; this is not a navigation validation result."""

    status: ContractStatus
    expected_topics: Tuple[str, ...]
    missing_topics: Tuple[str, ...]
    expected_frames: Tuple[str, ...]
    missing_frames: Tuple[str, ...]
    observed_frames: Tuple[str, ...]


def assess_navigation_readiness(
    published_topics: Mapping[str, str],
    observed_frames: Iterable[str],
    expected_topics: Iterable[str],
    expected_frames: Iterable[str],
) -> NavigationReadiness:
    """Compare passive ROS-graph observations with a supplied candidate contract.

    The caller must declare expectations. With none declared, the outcome is
    ``NOT_CONFIGURED`` rather than a misleading green result. Topic types are
    intentionally captured by the caller for diagnostics but not treated as
    proof that an action or controller is safe to activate.
    """
    declared_topics = _normalized_unique(expected_topics)
    declared_frames = _normalized_unique(expected_frames)
    observed_frame_set = set(_normalized_unique(observed_frames))
    missing_topics = tuple(topic for topic in declared_topics if topic not in published_topics)
    missing_frames = tuple(frame for frame in declared_frames if frame not in observed_frame_set)

    if not declared_topics and not declared_frames:
        status = ContractStatus.NOT_CONFIGURED
    elif missing_topics or missing_frames:
        status = ContractStatus.UNAVAILABLE
    else:
        status = ContractStatus.INTERFACE_COMPATIBLE

    return NavigationReadiness(
        status=status,
        expected_topics=declared_topics,
        missing_topics=missing_topics,
        expected_frames=declared_frames,
        missing_frames=missing_frames,
        observed_frames=tuple(sorted(observed_frame_set)),
    )


def _normalized_unique(values: Iterable[str]) -> Tuple[str, ...]:
    normalized = []
    for value in values:
        if not isinstance(value, str):
            continue
        stripped = value.strip()
        if stripped and stripped not in normalized:
            normalized.append(stripped)
    return tuple(normalized)
