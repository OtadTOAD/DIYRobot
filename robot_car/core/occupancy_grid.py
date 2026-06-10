"""Probabilistic occupancy grid (F-09).

The map is a log-odds belief grid. Internally every cell holds a ``float32`` log-odds
value (0 == unknown/p=0.5); the published representation is a ``uint8`` 0..100 grid
(0 free, 50 unknown, 100 occupied) used by A*, the web UI and the saved map file.

Conventions (see feature_plan.md):
  * world coordinates are metres, ``(x, y)`` with +y up;
  * grid cells are ``(col, row)`` and the numpy array is row-major ``grid[row, col]``;
  * world origin ``(0, 0)`` maps to ``(MAP_ORIGIN_COL, MAP_ORIGIN_ROW)``.

Provides: coordinate conversion, Bresenham ray casting with a log-odds Bayesian
update, obstacle inflation (SciPy binary dilation), a planning grid (inflated +
forbidden zones), msgpack save/load, and PNG export.
"""

from __future__ import annotations

import io
import math
import time

import msgpack
import numpy as np
from PIL import Image
from scipy.ndimage import binary_dilation

from robot_car import config


def bresenham(x0: int, y0: int, x1: int, y1: int):
    """Integer grid cells along the line (x0,y0)->(x1,y1), inclusive of both ends."""
    cells = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        cells.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return cells


def _prob_to_log_odds(p: float) -> float:
    return math.log(p / (1.0 - p))


_L_FREE = _prob_to_log_odds(config.P_FREE)
_L_OCC = _prob_to_log_odds(config.P_OCCUPIED)


class OccupancyGrid:
    def __init__(self, width=None, height=None, resolution=None,
                 origin_col=None, origin_row=None, name="unnamed"):
        self.width = int(width or config.MAP_WIDTH_CELLS)
        self.height = int(height or config.MAP_HEIGHT_CELLS)
        self.resolution = float(resolution or config.MAP_RESOLUTION)
        self.origin_col = int(config.MAP_ORIGIN_COL if origin_col is None else origin_col)
        self.origin_row = int(config.MAP_ORIGIN_ROW if origin_row is None else origin_row)
        self.name = name
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        self.waypoints: dict[str, tuple[float, float]] = {}

    # -- coordinate conversion ----------------------------------------------
    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        col = int(round(x / self.resolution)) + self.origin_col
        row = self.origin_row - int(round(y / self.resolution))
        return col, row

    def grid_to_world(self, col: int, row: int) -> tuple[float, float]:
        x = (col - self.origin_col) * self.resolution
        y = (self.origin_row - row) * self.resolution
        return x, y

    def in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self.width and 0 <= row < self.height

    # -- belief update -------------------------------------------------------
    def update_cell(self, col: int, row: int, p_evidence: float) -> None:
        if not self.in_bounds(col, row):
            return
        lo = self.log_odds[row, col] + _prob_to_log_odds(p_evidence)
        self.log_odds[row, col] = float(
            np.clip(lo, -config.LOG_ODDS_CLAMP, config.LOG_ODDS_CLAMP)
        )

    def integrate_scan(self, pose: tuple, distances: dict) -> list:
        """Ray-cast each horizontal sensor reading into the grid.

        Returns the list of (col, row) cells whose value changed (for delta web
        updates). ``distances`` are in cm; the downward sensor is ignored here.
        """
        x, y, theta = pose
        rcol, rrow = self.world_to_grid(x, y)
        max_range_m = config.SENSOR_MAX_RANGE_CM / 100.0
        touched = []

        for sensor, angle in config.SENSOR_ANGLES.items():
            reading_cm = distances.get(sensor, float("inf"))
            hit = math.isfinite(reading_cm)
            d = (reading_cm / 100.0) if hit else max_range_m
            beam = theta + angle
            ex = x + d * math.cos(beam)
            ey = y + d * math.sin(beam)
            ecol, erow = self.world_to_grid(ex, ey)

            cells = bresenham(rcol, rrow, ecol, erow)
            for i, (c, r) in enumerate(cells):
                if not self.in_bounds(c, r):
                    break
                is_endpoint = (i == len(cells) - 1)
                p = config.P_OCCUPIED if (is_endpoint and hit) else config.P_FREE
                self.update_cell(c, r, p)
                touched.append((c, r))
        return touched

    # -- published representations ------------------------------------------
    def to_uint8(self) -> np.ndarray:
        """Convert the log-odds belief to a 0..100 occupancy grid."""
        prob = 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))
        return np.clip(prob * 100.0, 0, 100).astype(np.uint8)

    def inflate(self, grid_uint8: np.ndarray | None = None) -> np.ndarray:
        """Return a copy of the grid with confirmed obstacles dilated ~10 cm."""
        base = self.to_uint8() if grid_uint8 is None else grid_uint8.copy()
        occupied = base > config.INFLATION_OCCUPIED_THRESHOLD
        inflated_mask = binary_dilation(occupied, iterations=config.INFLATION_ITERATIONS)
        base[inflated_mask] = config.GRID_OCCUPIED
        return base

    def planning_grid(self, forbidden=None) -> np.ndarray:
        """Inflated occupancy grid with forbidden cells hard-blocked, for A*."""
        grid = self.inflate()
        if forbidden is not None:
            grid[forbidden] = config.GRID_OCCUPIED
        return grid

    # -- persistence ---------------------------------------------------------
    def to_msgpack(self) -> bytes:
        grid = self.to_uint8()
        payload = {
            "meta": {
                "name": self.name,
                "created_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "resolution": self.resolution,
                "width_cells": self.width,
                "height_cells": self.height,
                "origin_x": self.origin_col,
                "origin_y": self.origin_row,
            },
            "grid": grid.tobytes(),
            "waypoints": {k: list(v) for k, v in self.waypoints.items()},
        }
        return msgpack.packb(payload, use_bin_type=True)

    def save(self, path: str) -> None:
        with open(path, "wb") as fh:
            fh.write(self.to_msgpack())

    @classmethod
    def from_msgpack(cls, blob: bytes) -> "OccupancyGrid":
        data = msgpack.unpackb(blob, raw=False)
        meta = data["meta"]
        grid = cls(
            width=meta["width_cells"], height=meta["height_cells"],
            resolution=meta["resolution"], origin_col=meta["origin_x"],
            origin_row=meta["origin_y"], name=meta.get("name", "unnamed"),
        )
        uint8 = np.frombuffer(data["grid"], dtype=np.uint8).reshape(
            meta["height_cells"], meta["width_cells"]
        )
        grid.set_from_uint8(uint8)
        grid.waypoints = {k: tuple(v) for k, v in data.get("waypoints", {}).items()}
        return grid

    @classmethod
    def load(cls, path: str) -> "OccupancyGrid":
        with open(path, "rb") as fh:
            return cls.from_msgpack(fh.read())

    def set_from_uint8(self, uint8: np.ndarray) -> None:
        """Rebuild the log-odds belief from a 0..100 grid (e.g. after load)."""
        prob = np.clip(uint8.astype(np.float32) / 100.0, 1e-3, 1 - 1e-3)
        self.log_odds = np.log(prob / (1.0 - prob)).astype(np.float32)

    # -- visualisation -------------------------------------------------------
    def to_png_bytes(self, inflated: bool = False) -> bytes:
        grid = self.inflate() if inflated else self.to_uint8()
        # 0 (free) -> white, 100 (occupied) -> black, 50 (unknown) -> grey.
        img = (255 - grid.astype(np.float32) * 2.55).clip(0, 255).astype(np.uint8)
        out = io.BytesIO()
        Image.fromarray(img, mode="L").save(out, format="PNG")
        return out.getvalue()

    def export_png(self, path: str, inflated: bool = False) -> None:
        with open(path, "wb") as fh:
            fh.write(self.to_png_bytes(inflated=inflated))
