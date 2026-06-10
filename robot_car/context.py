"""Shared robot context -- the objects the behaviours operate on.

Bundles the live :class:`SlamSystem` (occupancy grid + pose), the forbidden-zone
constraint layer and map file I/O so the explore / navigate / idle modes don't each
reach into globals. One instance is created at startup and passed to the modes and
the web server.
"""

from __future__ import annotations

import os

from robot_car import config, state
from robot_car.core.forbidden_zones import ForbiddenZones
from robot_car.core.occupancy_grid import OccupancyGrid
from robot_car.core.slam import SlamSystem


class RobotContext:
    def __init__(self, slam: SlamSystem | None = None):
        self.slam = slam or SlamSystem()
        self.forbidden = ForbiddenZones(self.slam.grid.width, self.slam.grid.height)
        os.makedirs(config.MAPS_DIR, exist_ok=True)

    # -- planning helpers ----------------------------------------------------
    def planning_grid(self):
        """Inflated occupancy grid with forbidden zones hard-blocked, for A*."""
        with self.slam._lock:
            return self.slam.grid.planning_grid(self.forbidden.mask)

    def current_cell(self):
        x, y, _ = state.get_pose()
        return self.slam.grid.world_to_grid(x, y)

    def grid_to_world(self, col, row):
        return self.slam.grid.grid_to_world(col, row)

    # -- map file I/O --------------------------------------------------------
    def _map_path(self, name):
        return os.path.join(config.MAPS_DIR, name + config.MAP_EXTENSION)

    def _zones_path(self, name):
        return os.path.join(config.MAPS_DIR, name + config.ZONES_EXTENSION)

    def list_maps(self):
        out = []
        for fn in sorted(os.listdir(config.MAPS_DIR)):
            if fn.endswith(config.MAP_EXTENSION):
                out.append(fn[: -len(config.MAP_EXTENSION)])
        return out

    def save_map(self, name: str) -> None:
        with self.slam._lock:
            self.slam.grid.name = name
            self.slam.grid.save(self._map_path(name))
        self.forbidden.save(self._zones_path(name))
        state.set_log("ok", "Saved map '%s'" % name)

    def load_map(self, name: str) -> None:
        grid = OccupancyGrid.load(self._map_path(name))
        with self.slam._lock:
            self.slam.grid = grid
            self.slam.set_mapping(False)
        self.forbidden = ForbiddenZones(grid.width, grid.height)
        zones_path = self._zones_path(name)
        if os.path.exists(zones_path):
            self.forbidden.load(zones_path)
        state.set_grid(grid.to_uint8())
        state.set_log("ok", "Loaded map '%s'" % name)

    @property
    def waypoints(self):
        return self.slam.grid.waypoints
