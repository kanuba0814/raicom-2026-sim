import math

from raicom_round_table_navigation.boundary_guard import (
    Footprint, OccupancyGridData, guard_footprint, guard_rotation_sweep,
)


def grid(cells=None, resolution=1.0, width=5, height=5):
    return OccupancyGridData(0.0, 0.0, resolution, width, height, cells if cells is not None else [0] * (width * height))


def test_free_grid_allows_full_translation_sweep():
    result = guard_footprint(grid(), Footprint(0.2, 0.2, 0.2, 0.2), 2.0, 2.0, 0.0, vx_mps=0.5, reaction_time_s=0.2, deceleration_mps2=1.0)
    assert result.safe


def test_unknown_cell_hard_stops():
    cells = [0] * 25
    cells[2 * 5 + 2] = -1
    result = guard_footprint(grid(cells), Footprint(0.1, 0.1, 0.1, 0.1), 2.0, 2.0, 0.0)
    assert not result.safe
    assert result.reason == "UNKNOWN_OR_OCCUPIED"


def test_occupied_middle_of_translation_path_hard_stops():
    cells = [0] * 25
    cells[2 * 5 + 3] = 100
    # Stops at x=4, but the occupied x=3 cell must be detected en route.
    result = guard_footprint(grid(cells), Footprint(0.1, 0.1, 0.1, 0.1), 2.0, 2.0, 0.0, vx_mps=1.0, reaction_time_s=1.5, deceleration_mps2=2.0)
    assert not result.safe
    assert result.reason == "UNKNOWN_OR_OCCUPIED"


def test_occupied_footprint_interior_hard_stops():
    cells = [0] * 100
    cells[5 * 10 + 5] = 100
    result = guard_footprint(grid(cells, resolution=0.1, width=10, height=10), Footprint(0.2, 0.2, 0.2, 0.2), 0.5, 0.5, 0.0)
    assert not result.safe
    assert result.reason == "UNKNOWN_OR_OCCUPIED"


def test_diagonal_corner_cell_intersection_is_not_missed_by_rotated_sampling():
    cells = [0] * 25
    # This cell intersects the rotated 0.1 m square near its diagonal corner.
    # Perimeter-only vertex samples do not land in it.
    cells[2 * 5 + 2] = 100
    result = guard_footprint(
        grid(cells), Footprint(0.05, 0.05, 0.05, 0.05),
        1.98, 1.98, math.pi / 4.0,
    )
    assert not result.safe
    assert result.reason == "UNKNOWN_OR_OCCUPIED"


def test_rotation_sweep_detects_obstacle_encountered_during_ninety_degree_turn():
    cells = [0] * 81
    # At yaw pi/4 a front-right corner of a 0.2 m rectangle sweeps this cell.
    cells[5 * 9 + 5] = 100
    result = guard_rotation_sweep(grid(cells, resolution=0.1, width=9, height=9), Footprint(0.2, 0.2, 0.2, 0.2), 0.4, 0.4, 0.0, 1.57079632679)
    assert not result.safe
    assert result.reason == "UNKNOWN_OR_OCCUPIED"


def test_caller_cannot_weaken_half_cell_sampling_limit():
    result = guard_footprint(
        grid(), Footprint(0.1, 0.1, 0.1, 0.1), 2.0, 2.0, 0.0,
        sample_spacing_m=0.6,
    )
    assert not result.safe
    assert result.reason == "SAMPLE_SPACING_INVALID"


def test_out_of_bounds_hard_stops():
    result = guard_footprint(grid(), Footprint(0.5, 0.5, 0.5, 0.5), 0.1, 0.1, 0.0)
    assert not result.safe
    assert result.reason == "OUT_OF_BOUNDS"
