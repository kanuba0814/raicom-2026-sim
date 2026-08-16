#!/usr/bin/env python3
"""Pure occupancy-grid frontier selection for the SIM_ONLY survey coordinator.

Unknown cells are never traversable.  A frontier is a known-free cell adjacent
(8-connected) to unknown space; selected goals are reachable known-free cells
set back from that edge by the configured standoff distance.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Iterable, List, Sequence, Tuple

Cell = Tuple[int, int]  # (column, row)


@dataclass(frozen=True)
class GridSpec:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: Sequence[int]

    def cell_from_world(self, x: float, y: float) -> Cell:
        return (int(math.floor((x - self.origin_x) / self.resolution)),
                int(math.floor((y - self.origin_y) / self.resolution)))

    def world_from_cell(self, cell: Cell) -> Tuple[float, float]:
        return (self.origin_x + (cell[0] + 0.5) * self.resolution,
                self.origin_y + (cell[1] + 0.5) * self.resolution)

    def value(self, cell: Cell) -> int:
        return self.data[cell[1] * self.width + cell[0]]

    def contains(self, cell: Cell, edge_margin_cells: int = 0) -> bool:
        return (edge_margin_cells <= cell[0] < self.width - edge_margin_cells and
                edge_margin_cells <= cell[1] < self.height - edge_margin_cells)


@dataclass(frozen=True)
class FrontierGoal:
    cell: Cell
    frontier_cell: Cell
    world_x: float
    world_y: float


_ADJACENT_NEIGHBORS: Tuple[Cell, ...] = tuple(
    (dx, dy) for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dx or dy
)
_TRAVERSAL_NEIGHBORS: Tuple[Cell, ...] = ((1, 0), (-1, 0), (0, 1), (0, -1))


def _neighbors(cell: Cell, offsets: Sequence[Cell] = _ADJACENT_NEIGHBORS) -> Iterable[Cell]:
    for dx, dy in offsets:
        yield cell[0] + dx, cell[1] + dy


def _inflated_occupied(grid: GridSpec, occupied_threshold: int, radius_cells: int) -> set[Cell]:
    occupied = {
        (x, y) for y in range(grid.height) for x in range(grid.width)
        if grid.value((x, y)) >= occupied_threshold
    }
    if radius_cells == 0:
        return occupied
    inflated: set[Cell] = set()
    radius_squared = radius_cells * radius_cells
    for ox, oy in occupied:
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                cell = (ox + dx, oy + dy)
                if dx * dx + dy * dy <= radius_squared and grid.contains(cell):
                    inflated.add(cell)
    return inflated


def reachable_frontier_goals(
    grid: GridSpec,
    robot_world_x: float,
    robot_world_y: float,
    *,
    occupied_threshold: int = 50,
    inflation_radius_m: float = 0.20,
    edge_margin_m: float = 0.20,
    standoff_m: float = 0.25,
    seed_search_radius_m: float = 0.0,
) -> List[FrontierGoal]:
    """Return unique known-free goals reachable from the current robot TF seed.

    Traversal uses only occupancy value ``0``.  Occupied cells (at or above the
    threshold) are inflated, and all map-edge-margin cells are excluded.  The
    nearest known-free cell at least ``standoff_m`` from each frontier edge is
    used as its goal, which prevents deliberately commanding unknown space.
    """
    if grid.width <= 0 or grid.height <= 0 or grid.resolution <= 0.0:
        raise ValueError("grid dimensions and resolution must be positive")
    if len(grid.data) != grid.width * grid.height:
        raise ValueError("occupancy data length does not match grid dimensions")
    if min(inflation_radius_m, edge_margin_m, standoff_m, seed_search_radius_m) < 0.0:
        raise ValueError("frontier distances must be non-negative")

    edge_cells = int(math.ceil(edge_margin_m / grid.resolution))
    inflation_cells = int(math.ceil(inflation_radius_m / grid.resolution))
    standoff_cells = int(math.ceil(standoff_m / grid.resolution))
    inflated = _inflated_occupied(grid, occupied_threshold, inflation_cells)

    def traversable(cell: Cell) -> bool:
        return grid.contains(cell, edge_cells) and cell not in inflated and grid.value(cell) == 0

    robot_cell = grid.cell_from_world(robot_world_x, robot_world_y)
    seed_radius_cells = int(math.ceil(seed_search_radius_m / grid.resolution))
    seed_candidates = [
        (robot_cell[0] + dx, robot_cell[1] + dy)
        for dy in range(-seed_radius_cells, seed_radius_cells + 1)
        for dx in range(-seed_radius_cells, seed_radius_cells + 1)
        if dx * dx + dy * dy <= seed_radius_cells * seed_radius_cells
        and traversable((robot_cell[0] + dx, robot_cell[1] + dy))
    ]
    if not seed_candidates:
        return []
    seed = min(seed_candidates, key=lambda cell: (
        (cell[0] - robot_cell[0]) ** 2 + (cell[1] - robot_cell[1]) ** 2,
        cell[1], cell[0],
    ))
    reachable: set[Cell] = {seed}
    queue: deque[Cell] = deque([seed])
    while queue:
        current = queue.popleft()
        for neighbor in _neighbors(current, _TRAVERSAL_NEIGHBORS):
            if neighbor not in reachable and traversable(neighbor):
                reachable.add(neighbor)
                queue.append(neighbor)

    frontiers = [
        cell for cell in reachable
        if any(grid.contains(neighbor) and grid.value(neighbor) < 0 for neighbor in _neighbors(cell))
    ]
    # A multi-source distance field measures standoff through known reachable
    # free space only; unknown has no path and cannot become a goal.
    distance: dict[Cell, int] = {cell: 0 for cell in frontiers}
    queue = deque(frontiers)
    while queue:
        current = queue.popleft()
        for neighbor in _neighbors(current, _TRAVERSAL_NEIGHBORS):
            if neighbor in reachable and neighbor not in distance:
                distance[neighbor] = distance[current] + 1
                queue.append(neighbor)

    goals: List[FrontierGoal] = []
    used: set[Cell] = set()
    for frontier in frontiers:
        candidates = [cell for cell in reachable if distance.get(cell, -1) >= standoff_cells]
        if not candidates:
            continue
        goal = min(candidates, key=lambda cell: ((cell[0] - frontier[0]) ** 2 + (cell[1] - frontier[1]) ** 2,
                                                   (cell[0] - seed[0]) ** 2 + (cell[1] - seed[1]) ** 2))
        if goal not in used:
            x, y = grid.world_from_cell(goal)
            goals.append(FrontierGoal(goal, frontier, x, y))
            used.add(goal)
    return sorted(goals, key=lambda item: (item.cell[0] - seed[0]) ** 2 + (item.cell[1] - seed[1]) ** 2)
