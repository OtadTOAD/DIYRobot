"""Phase E -- occupancy grid, ray casting, inflation, persistence, frontiers, scan match."""

import os
import tempfile

import numpy as np

from robot_car import config
from robot_car.core import slam
from robot_car.core.occupancy_grid import OccupancyGrid, bresenham

INF = float("inf")


def test_coordinate_roundtrip():
    g = OccupancyGrid()
    for x, y in [(0.0, 0.0), (1.23, -0.47), (-2.0, 2.0)]:
        col, row = g.world_to_grid(x, y)
        wx, wy = g.grid_to_world(col, row)
        assert abs(wx - x) <= g.resolution
        assert abs(wy - y) <= g.resolution


def test_origin_is_centre():
    g = OccupancyGrid()
    assert g.world_to_grid(0.0, 0.0) == (g.origin_col, g.origin_row)


def test_bresenham_straight_and_diagonal():
    assert bresenham(0, 0, 3, 0) == [(0, 0), (1, 0), (2, 0), (3, 0)]
    diag = bresenham(0, 0, 3, 3)
    assert diag[0] == (0, 0) and diag[-1] == (3, 3) and len(diag) == 4


def test_unknown_maps_to_fifty():
    g = OccupancyGrid()
    assert int(g.to_uint8()[g.origin_row, g.origin_col]) == 50


def test_update_cell_moves_toward_occupied():
    g = OccupancyGrid()
    c, r = g.origin_col, g.origin_row
    for _ in range(3):
        g.update_cell(c, r, config.P_OCCUPIED)
    assert g.to_uint8()[r, c] > config.INFLATION_OCCUPIED_THRESHOLD


def test_integrate_scan_marks_free_and_occupied():
    g = OccupancyGrid()
    distances = {"front": 50.0, "left": INF, "right": INF, "back": INF, "down": 15.0}
    g.integrate_scan((0.0, 0.0, 0.0), distances)
    grid = g.to_uint8()
    end_col, end_row = g.world_to_grid(0.5, 0.0)     # endpoint ~occupied
    mid_col, mid_row = g.world_to_grid(0.25, 0.0)    # along beam ~free
    assert grid[end_row, end_col] > 55
    assert grid[mid_row, mid_col] < 50


def test_inflation_thickens_walls():
    g = OccupancyGrid()
    c, r = g.origin_col, g.origin_row
    g.log_odds[r, c] = config.LOG_ODDS_CLAMP        # hard occupied single cell
    base = g.to_uint8()
    inflated = g.inflate()
    assert (inflated == config.GRID_OCCUPIED).sum() > (base > config.INFLATION_OCCUPIED_THRESHOLD).sum()
    assert inflated[r, c + config.INFLATION_ITERATIONS] == config.GRID_OCCUPIED


def test_save_load_roundtrip(tmp_path):
    g = OccupancyGrid(name="unit")
    g.log_odds[10, 10] = 5.0
    g.waypoints = {"desk": (1.0, -0.5)}
    path = os.path.join(tmp_path, "unit.map")
    g.save(path)
    loaded = OccupancyGrid.load(path)
    assert loaded.name == "unit"
    assert loaded.waypoints["desk"] == (1.0, -0.5)
    assert np.array_equal(loaded.to_uint8(), g.to_uint8())


def test_png_export_is_png(tmp_path):
    g = OccupancyGrid()
    data = g.to_png_bytes()
    assert data[:8] == b"\x89PNG\r\n\x1a\n"


def test_find_frontiers_on_free_island():
    g = OccupancyGrid()
    # A 5x5 known-free square surrounded by unknown.
    r0, c0 = 40, 40
    g.log_odds[r0:r0 + 5, c0:c0 + 5] = -config.LOG_ODDS_CLAMP
    frontiers = set(slam.find_frontiers(g.to_uint8()))
    assert len(frontiers) == 16                      # border cells of the 5x5
    assert (c0 + 2, r0 + 2) not in frontiers         # interior centre is not a frontier


def test_scan_match_recovers_perturbed_pose():
    g = OccupancyGrid()
    # Vertical wall at world x = 1.0 m spanning rows around the origin.
    wall_col, _ = g.world_to_grid(1.0, 0.0)
    rmid = g.origin_row
    g.log_odds[rmid - 10:rmid + 10, wall_col] = config.LOG_ODDS_CLAMP   # single-column wall
    grid_uint8 = g.to_uint8()

    distances = {"front": 100.0, "left": INF, "right": INF, "back": INF, "down": 15.0}
    # True pose is the origin facing +x; start the search from a 10 cm error.
    perturbed = (0.10, 0.0, 0.0)
    best_pose, score = slam.scan_match(g, grid_uint8, perturbed, distances)
    assert score >= 0.99
    assert abs(best_pose[0]) < 0.06                  # pulled back toward x = 0
