"""SLAM loop: mapping + scan-match localization + frontier detection (F-10).

A ~10 Hz daemon thread that, each cycle:
  1. advances the encoder odometry (dead reckoning),
  2. reads the ultrasonic sensors,
  3. refines the pose with correlation-window scan matching against the map,
  4. consumes the visual-odometry delta accumulated since the last cycle,
  5. fuses all three into the published robot pose (localization.py),
  6. feeds the fused pose back into odometry (correction), and
  7. ray-casts the readings into the occupancy grid.

Mapping runs only in 'explore' mode; in 'navigate'/'idle' the same loop runs as pure
localization on the fixed map. Frontier detection (:func:`find_frontiers`) is exposed
for the explore behaviour.
"""

from __future__ import annotations

import math
import threading
import time

import numpy as np
from scipy.ndimage import binary_dilation

from robot_car import config, state
from robot_car.core import localization
from robot_car.core.geometry import wrap_angle
from robot_car.core.occupancy_grid import OccupancyGrid
from robot_car.core.odometry import Odometry
from robot_car.hardware import sensors


# ---------------------------------------------------------------------------
# Scan matching (pure, testable)
# ---------------------------------------------------------------------------
def score_pose(grid: OccupancyGrid, grid_uint8: np.ndarray, pose, distances) -> float:
    """Fraction of finite sensor endpoints that land on a mapped obstacle cell.

    Higher means the candidate pose better explains the readings against the map.
    Endpoints are matched against a 1-cell neighbourhood to tolerate discretisation.
    """
    x, y, theta = pose
    finite = 0
    hits = 0
    for sensor, angle in config.SENSOR_ANGLES.items():
        reading_cm = distances.get(sensor, float("inf"))
        if not math.isfinite(reading_cm):
            continue
        finite += 1
        d = reading_cm / 100.0
        beam = theta + angle
        ex = x + d * math.cos(beam)
        ey = y + d * math.sin(beam)
        col, row = grid.world_to_grid(ex, ey)
        if _occupied_near(grid, grid_uint8, col, row):
            hits += 1
    if finite == 0:
        return 0.0
    return hits / finite


def _occupied_near(grid: OccupancyGrid, grid_uint8, col, row, radius=1) -> bool:
    thr = config.INFLATION_OCCUPIED_THRESHOLD
    for dc in range(-radius, radius + 1):
        for dr in range(-radius, radius + 1):
            c, r = col + dc, row + dr
            if grid.in_bounds(c, r) and grid_uint8[r, c] > thr:
                return True
    return False


def scan_match(grid: OccupancyGrid, grid_uint8, pose, distances):
    """Correlation-window search around ``pose``. Returns ``(best_pose, score)``."""
    best_pose = pose
    best_score = score_pose(grid, grid_uint8, pose, distances)

    rng = config.SCAN_MATCH_RANGE
    step = config.SCAN_MATCH_STEP
    arng = config.SCAN_MATCH_ANGLE
    astep = config.SCAN_MATCH_ANGLE_STEP

    dxs = np.arange(-rng, rng + 1e-9, step)
    dys = np.arange(-rng, rng + 1e-9, step)
    dthetas = np.arange(-arng, arng + 1e-9, astep)
    for dx in dxs:
        for dy in dys:
            for dth in dthetas:
                cand = (pose[0] + dx, pose[1] + dy, pose[2] + dth)
                s = score_pose(grid, grid_uint8, cand, distances)
                if s > best_score:
                    best_score = s
                    best_pose = cand
    return best_pose, best_score


# ---------------------------------------------------------------------------
# Frontier detection (pure, testable)
# ---------------------------------------------------------------------------
def frontier_mask(grid_uint8: np.ndarray) -> np.ndarray:
    """Boolean mask of known-free cells adjacent to unknown space."""
    free = grid_uint8 < config.FRONTIER_FREE_THRESHOLD
    unknown = grid_uint8 == config.GRID_UNKNOWN
    return free & binary_dilation(unknown)


def find_frontiers(grid_uint8: np.ndarray):
    """Return (col, row) of known-free cells adjacent to unknown space."""
    rows, cols = np.nonzero(frontier_mask(grid_uint8))
    return list(zip(cols.tolist(), rows.tolist()))


# ---------------------------------------------------------------------------
# SLAM system
# ---------------------------------------------------------------------------
class SlamSystem(threading.Thread):
    def __init__(self, grid: OccupancyGrid | None = None, mapping: bool = True):
        super().__init__(name="slam", daemon=True)
        self.grid = grid or OccupancyGrid()
        self.odometry = Odometry(state.get_pose())
        self.mapping = mapping
        self._lock = threading.RLock()
        self.cell_listener = None    # optional callback(list_of_(col,row,value))
        self._prev_pose = None       # fused pose last cycle, for the motion guard

    def set_mapping(self, enabled: bool) -> None:
        with self._lock:
            self.mapping = enabled

    def run(self) -> None:
        period = 1.0 / config.SLAM_HZ
        # Publish the initial grid so the UI has something to draw.
        state.set_grid(self.grid.to_uint8())
        while not state.stop_event.is_set():
            t0 = time.monotonic()
            self.step()
            elapsed = time.monotonic() - t0
            if elapsed < period:
                time.sleep(period - elapsed)

    def step(self) -> tuple:
        """One SLAM/localization cycle. Returns the fused pose."""
        base_pose = self.odometry.get_pose()
        enc_pose = self.odometry.update()
        distances = sensors.get_all_distances()

        grid_uint8 = self.grid.to_uint8()
        scan_pose, scan_score = scan_match(self.grid, grid_uint8, enc_pose, distances)
        scan_accepted = scan_score >= config.SCAN_MATCH_THRESHOLD

        # The VO delta covers the same interval as the encoder update, so it is
        # applied to the pose from *before* that update -- applying it on top of
        # enc_pose would count the cycle's motion twice. The delta is robot-frame
        # (forward, lateral, dtheta) from the forward camera; rotate it into the
        # world frame at the pre-update heading.
        vo_delta, vo_conf = state.consume_vo()
        cos_h, sin_h = math.cos(base_pose[2]), math.sin(base_pose[2])
        vo_pose = (base_pose[0] + vo_delta[0] * cos_h - vo_delta[1] * sin_h,
                   base_pose[1] + vo_delta[0] * sin_h + vo_delta[1] * cos_h,
                   base_pose[2] + vo_delta[2])
        slip = localization.detect_slip(self.odometry.last_distance, vo_delta, vo_conf)

        w_e, w_s, w_v = localization.compute_weights(scan_score, scan_accepted, vo_conf, slip)
        fused = localization.fuse_poses(enc_pose, scan_pose, vo_pose, w_e, w_s, w_v)

        # Feed the correction back so the next dead-reckoning step builds on it.
        self.odometry.set_pose(fused)
        state.set_pose(fused)

        if self.mapping and self._scan_motion_ok(self._prev_pose, fused):
            with self._lock:
                touched = self.grid.integrate_scan(fused, distances)
                grid_uint8 = self.grid.to_uint8()
            state.set_grid(grid_uint8)
            self._emit_cells(touched, grid_uint8)
        self._prev_pose = fused
        return fused

    @staticmethod
    def _scan_motion_ok(prev_pose, pose) -> bool:
        """True when little enough happened since last cycle to trust the snapshot.

        A scan is ray-cast as one instantaneous geometry; if the robot swung through
        a large angle (in-place turn) between 10 Hz cycles, far endpoints smear into
        phantom arcs of obstacle. Skipping those cycles keeps the map clean.
        """
        if prev_pose is None:
            return True
        d_trans = math.hypot(pose[0] - prev_pose[0], pose[1] - prev_pose[1])
        d_rot = abs(wrap_angle(pose[2] - prev_pose[2]))
        return (d_rot <= config.MAP_MAX_ROTATION_PER_SCAN and
                d_trans <= config.MAP_MAX_TRANSLATION_PER_SCAN)

    def _emit_cells(self, touched, grid_uint8):
        if not self.cell_listener or not touched:
            return
        seen = set()
        updates = []
        for c, r in touched:
            if (c, r) in seen:
                continue
            seen.add((c, r))
            updates.append((c, r, int(grid_uint8[r, c])))
        try:
            self.cell_listener(updates)
        except Exception:
            pass
