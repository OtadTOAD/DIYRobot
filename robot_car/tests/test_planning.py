"""Phase F -- A* planner and forbidden zones."""

import numpy as np

from robot_car import config
from robot_car.core import path_planner as pp
from robot_car.core.forbidden_zones import ForbiddenZones


def _free_grid(n=50):
    return np.zeros((n, n), dtype=np.uint8)


def test_astar_straight_path():
    grid = _free_grid()
    path = pp.astar(grid, (5, 5), (15, 5))
    assert path is not None
    assert path[0] == (5, 5) and path[-1] == (15, 5)
    assert all(r == 5 for _, r in path)
    assert len(path) == 11


def test_astar_uses_diagonals():
    grid = _free_grid()
    path = pp.astar(grid, (5, 5), (10, 10))
    assert path is not None
    assert len(path) == 6                     # 5 diagonal steps + start


def test_astar_no_path_through_wall():
    grid = _free_grid()
    grid[:, 10] = config.GRID_OCCUPIED        # full vertical wall
    assert pp.astar(grid, (5, 5), (15, 5)) is None


def test_astar_routes_through_gap():
    grid = _free_grid()
    grid[:, 10] = config.GRID_OCCUPIED
    grid[25, 10] = config.GRID_FREE           # single doorway
    path = pp.astar(grid, (5, 5), (15, 5))
    assert path is not None
    assert (10, 25) in path                   # squeezes through the gap


def test_walkable_threshold_boundary():
    grid = _free_grid()
    grid[8, 8] = config.WALKABLE_THRESHOLD - 1
    grid[9, 9] = config.WALKABLE_THRESHOLD
    assert pp.is_walkable(grid, None, 8, 8) is True
    assert pp.is_walkable(grid, None, 9, 9) is False


def test_nearest_walkable_already_free():
    grid = _free_grid()
    assert pp.nearest_walkable(grid, (5, 5)) == (5, 5)


def test_nearest_walkable_escapes_obstacle_blob():
    grid = _free_grid()
    grid[3:10, 3:10] = config.GRID_OCCUPIED   # blob; (5, 5) is inside it
    cell = pp.nearest_walkable(grid, (5, 5))
    assert cell is not None
    col, row = cell
    assert pp.is_walkable(grid, None, col, row)
    # Nearest free cell from (5,5) is just outside the blob edge.
    assert max(abs(col - 5), abs(row - 5)) <= 3


def test_nearest_walkable_respects_forbidden_and_radius():
    grid = _free_grid(20)
    forbidden = np.ones((20, 20), dtype=bool)  # everything forbidden
    assert pp.nearest_walkable(grid, (10, 10), forbidden, max_radius=5) is None


def test_line_of_sight():
    grid = _free_grid()
    assert pp.line_of_sight(grid, (0, 0), (20, 0)) is True
    grid[0, 10] = config.GRID_OCCUPIED
    assert pp.line_of_sight(grid, (0, 0), (20, 0)) is False


def test_smoothing_collapses_staircase():
    grid = _free_grid()
    raw = [(0, 0), (1, 0), (1, 1), (2, 1), (2, 2), (3, 2), (3, 3)]
    smoothed = pp.smooth_path(raw, grid)
    assert smoothed[0] == (0, 0) and smoothed[-1] == (3, 3)
    assert len(smoothed) < len(raw)


def test_forbidden_zone_blocks_planner():
    grid = _free_grid()
    fz = ForbiddenZones(width=50, height=50)
    fz.add_zone(10, 0, 10, 49)                # vertical no-go column
    assert pp.astar(grid, (5, 5), (15, 5), forbidden=fz.mask) is None


def test_forbidden_zone_add_and_clear():
    fz = ForbiddenZones(width=50, height=50)
    fz.add_zone(5, 5, 8, 8)
    assert fz.is_blocked(6, 6) is True
    assert fz.is_blocked(0, 0) is False
    assert fz.count() == 16
    fz.clear_all()
    assert fz.count() == 0


def test_forbidden_zone_save_load(tmp_path):
    fz = ForbiddenZones(width=50, height=50)
    fz.add_zone(5, 5, 8, 8)
    path = str(tmp_path / "x.zones")
    fz.save(path)
    fz2 = ForbiddenZones(width=50, height=50)
    fz2.load(path)
    assert np.array_equal(fz2.mask, fz.mask)
