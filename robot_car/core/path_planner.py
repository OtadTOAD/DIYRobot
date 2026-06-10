"""A* path planning on the occupancy grid (F-12).

8-directional search with correctly weighted diagonals (cost sqrt 2) and an
admissible Euclidean heuristic, using a ``heapq`` open set and a NumPy boolean
closed set. A cell is walkable when its occupancy value is below ``WALKABLE_THRESHOLD``
and it is not inside a hard-blocked forbidden zone. Diagonal moves that would clip a
wall corner are disallowed.

Raw paths are then string-pulled (visibility-based pruning) into long straight
segments to cut motor jitter and turning. Replanning policy (wait-retry then full
replan) lives in the navigate behaviour, not here -- the planner stays pure.

All coordinates are grid cells ``(col, row)``; the grid is row-major ``grid[row, col]``.
"""

from __future__ import annotations

import heapq
import math

import numpy as np

from robot_car import config
from robot_car.core.occupancy_grid import bresenham

# (dcol, drow, cost)
_DIRECTIONS = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, config.DIAGONAL_COST), (1, -1, config.DIAGONAL_COST),
    (-1, 1, config.DIAGONAL_COST), (-1, -1, config.DIAGONAL_COST),
]


def heuristic(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def in_bounds(grid: np.ndarray, col: int, row: int) -> bool:
    h, w = grid.shape
    return 0 <= col < w and 0 <= row < h


def is_walkable(grid: np.ndarray, forbidden, col: int, row: int) -> bool:
    if not in_bounds(grid, col, row):
        return False
    if grid[row, col] >= config.WALKABLE_THRESHOLD:
        return False
    if forbidden is not None and forbidden[row, col]:
        return False
    return True


def nearest_walkable(grid: np.ndarray, cell, forbidden=None,
                     max_radius: int = config.NAV_ESCAPE_RADIUS_CELLS):
    """Closest walkable cell to ``cell`` within ``max_radius`` cells, or None.

    Scans concentric square rings outward and returns the Euclidean-nearest hit in
    the first ring that contains one, so a pose stuck inside an inflated obstacle
    or forbidden zone gets the shortest escape.
    """
    if is_walkable(grid, forbidden, *cell):
        return cell
    c0, r0 = cell
    for radius in range(1, max_radius + 1):
        best, best_d = None, math.inf
        for dc in range(-radius, radius + 1):
            for dr in range(-radius, radius + 1):
                if max(abs(dc), abs(dr)) != radius:
                    continue
                if is_walkable(grid, forbidden, c0 + dc, r0 + dr):
                    d = dc * dc + dr * dr
                    if d < best_d:
                        best, best_d = (c0 + dc, r0 + dr), d
        if best is not None:
            return best
    return None


def astar(grid: np.ndarray, start, goal, forbidden=None):
    """Return a list of (col, row) cells from start to goal, or None if no path."""
    if not is_walkable(grid, forbidden, *goal) or not is_walkable(grid, forbidden, *start):
        return None

    open_set = [(0.0, start)]
    came_from = {}
    g_score = {start: 0.0}
    closed = np.zeros(grid.shape, dtype=bool)

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            return _reconstruct(came_from, current)
        cc, cr = current
        if closed[cr, cc]:
            continue
        closed[cr, cc] = True

        for dc, dr, cost in _DIRECTIONS:
            nc, nr = cc + dc, cr + dr
            if not is_walkable(grid, forbidden, nc, nr) or closed[nr, nc]:
                continue
            # Don't cut diagonal corners between two obstacles.
            if dc != 0 and dr != 0:
                if not is_walkable(grid, forbidden, cc + dc, cr) or \
                   not is_walkable(grid, forbidden, cc, cr + dr):
                    continue
            tentative = g_score[current] + cost
            if tentative < g_score.get((nc, nr), math.inf):
                came_from[(nc, nr)] = current
                g_score[(nc, nr)] = tentative
                f = tentative + heuristic((nc, nr), goal)
                heapq.heappush(open_set, (f, (nc, nr)))
    return None


def _reconstruct(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return path[::-1]


def line_of_sight(grid: np.ndarray, a, b, forbidden=None) -> bool:
    """True if every cell on the segment a-b is walkable."""
    for c, r in bresenham(a[0], a[1], b[0], b[1]):
        if not is_walkable(grid, forbidden, c, r):
            return False
    return True


def smooth_path(path, grid: np.ndarray, forbidden=None):
    """Visibility-based pruning (string pulling) of a raw A* path."""
    if not path or len(path) <= 2:
        return list(path) if path else path
    smoothed = [path[0]]
    i = 0
    while i < len(path) - 1:
        furthest = i + 1
        for j in range(i + 2, len(path)):
            if line_of_sight(grid, path[i], path[j], forbidden):
                furthest = j
        smoothed.append(path[furthest])
        i = furthest
    return smoothed


def plan_path(grid: np.ndarray, start, goal, forbidden=None):
    """A* + smoothing convenience wrapper. Returns smoothed (col, row) cells or None."""
    raw = astar(grid, start, goal, forbidden)
    if raw is None:
        return None
    return smooth_path(raw, grid, forbidden)
