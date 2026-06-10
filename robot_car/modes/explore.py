"""Explore mode (F-14) -- frontier-based autonomous mapping.

Loop: find frontier regions (known-free cells bordering unknown space), cluster them
into connected regions, drive to the centroid of the nearest reachable region with
A*, and repeat. As the robot arrives new frontiers appear; when none remain the map
is complete -- obstacles are inflated and the map is saved. A clean stop (mode switch
/ shutdown) saves the partial map.
"""

from __future__ import annotations

import math
import time

import numpy as np
from scipy.ndimage import binary_dilation, center_of_mass, label

from robot_car import config, state
from robot_car.core import path_planner as pp
from robot_car.hardware import motors
from robot_car.modes.navigate import Navigator, REACHED, NO_PATH, ABORTED


def cluster_frontiers(grid_uint8: np.ndarray):
    """Return (col, row) centroids of connected frontier regions, largest first."""
    free = grid_uint8 < config.FRONTIER_FREE_THRESHOLD
    unknown = grid_uint8 == config.GRID_UNKNOWN
    mask = free & binary_dilation(unknown)
    labelled, n = label(mask)
    if n == 0:
        return []
    sizes = np.bincount(labelled.ravel())
    regions = []
    for lbl in range(1, n + 1):
        if sizes[lbl] < config.FRONTIER_MIN_CLUSTER:
            continue
        row, col = center_of_mass(mask, labelled, lbl)
        regions.append((sizes[lbl], (int(round(col)), int(round(row)))))
    regions.sort(reverse=True)             # largest regions first
    return [c for _, c in regions]


class Explorer:
    def __init__(self, context, save_name: str = "explored"):
        self.ctx = context
        self.navigator = Navigator(context)
        self.save_name = save_name

    def run(self, stop_event) -> None:
        state.set_mode("explore")
        self.ctx.slam.set_mapping(True)
        state.set_log("info", "Exploration started")
        failures = set()

        while not stop_event.is_set() and not state.stop_event.is_set():
            grid = state.get_grid()
            if grid is None:
                stop_event.wait(0.2)
                continue

            target = self._choose_target(grid, failures)
            if target is None:
                state.set_log("ok", "No frontiers left -- exploration complete")
                break

            status = self.navigator.run(target, stop_event)
            if status == ABORTED:
                break
            if status == NO_PATH:
                failures.add(target)
            # 'reached' -> loop and re-evaluate against the freshly updated map.

        motors.stop()
        self._finalize()

    def _choose_target(self, grid, failures):
        start = self.ctx.current_cell()
        plan_grid = self.ctx.planning_grid()
        centroids = cluster_frontiers(grid)
        # Nearest reachable region the robot hasn't already failed to reach.
        centroids.sort(key=lambda c: math.hypot(c[0] - start[0], c[1] - start[1]))
        for cen in centroids:
            if cen in failures:
                continue
            if pp.astar(plan_grid, start, cen, self.ctx.forbidden.mask) is not None:
                return cen
        return None

    def _finalize(self):
        """Inflate obstacles and persist the (possibly partial) map."""
        with self.ctx.slam._lock:
            inflated = self.ctx.slam.grid.inflate()
        state.set_grid(inflated)
        try:
            self.ctx.save_map(self.save_name)
        except Exception as exc:                      # pragma: no cover
            state.set_log("error", "Failed to save map: %s" % exc)
