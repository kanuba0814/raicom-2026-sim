"""Pure fail-closed health policy for the live-SLAM safety watchdog."""
from __future__ import annotations

import math
from typing import Optional, Sequence


class SlamHealth:
    """Require recent scan, odometry, map and a finite map-to-base transform."""

    def __init__(self, timeout_s: float) -> None:
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("timeout must be finite and positive")
        self._timeout_s = timeout_s
        self._last_scan_s: Optional[float] = None
        self._last_odom_s: Optional[float] = None
        self._last_map_s: Optional[float] = None

    def observe_scan(self, now_s: float) -> None:
        self._last_scan_s = now_s

    def observe_odom(self, now_s: float) -> None:
        self._last_odom_s = now_s

    def observe_map(self, now_s: float) -> None:
        self._last_map_s = now_s

    def valid(
        self,
        now_s: float,
        transform_values: Optional[Sequence[float]],
        transform_stamp_s: Optional[float] = None,
    ) -> bool:
        """Require initialized map plus fresh scan, odom and stamped TF.

        GMapping's occupancy grid is progress evidence, not a localization
        heartbeat. Post-motion map revision is enforced by the survey
        coordinator; this policy keeps continuous pose health fail-closed.
        """
        realtime_observations = (self._last_scan_s, self._last_odom_s)
        return (
            all(observation is not None and now_s - observation <= self._timeout_s
                for observation in realtime_observations)
            and self._last_map_s is not None
            and transform_values is not None
            and transform_stamp_s is not None
            and now_s - transform_stamp_s <= self._timeout_s
            and all(math.isfinite(value) for value in transform_values)
        )
