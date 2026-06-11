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
from scipy.ndimage import binary_dilation, distance_transform_edt

from robot_car import config, state
from robot_car.core import localization
from robot_car.core.geometry import sensor_origin, wrap_angle
from robot_car.core.occupancy_grid import OccupancyGrid
from robot_car.core.odometry import Odometry
from robot_car.hardware import sensors


# ---------------------------------------------------------------------------
# Likelihood-field scan matching (pure, testable)
# ---------------------------------------------------------------------------
class LikelihoodField:
    """Distance-to-nearest-obstacle field; scores a beam endpoint by a Gaussian of
    that distance. Continuous (no plateaus) and flat where the map is featureless."""

    def __init__(self, grid: OccupancyGrid, grid_uint8: np.ndarray):
        self.grid = grid
        occupied = grid_uint8 > config.INFLATION_OCCUPIED_THRESHOLD
        self.flat = not occupied.any()
        self.dist = (None if self.flat
                     else distance_transform_edt(~occupied) * grid.resolution)

    def likelihood(self, ex: float, ey: float) -> float:
        if self.flat:
            return 0.0
        col, row = self.grid.world_to_grid(ex, ey)
        if not self.grid.in_bounds(col, row):
            return 0.0
        d = self.dist[row, col]
        return math.exp(-(d * d) / (2.0 * config.SCAN_MATCH_SIGMA_M ** 2))


def score_pose(field: LikelihoodField, pose, distances) -> float:
    """Mean endpoint likelihood for a pose (0 when no beams hit or the field is flat)."""
    total = beams = 0
    for sensor, angle in config.SENSOR_ANGLES.items():
        reading_cm = distances.get(sensor, float("inf"))
        if not math.isfinite(reading_cm):
            continue
        beams += 1
        ox, oy = sensor_origin(pose, sensor)
        beam, d = pose[2] + angle, reading_cm / 100.0
        total += field.likelihood(ox + d * math.cos(beam), oy + d * math.sin(beam))
    return total / beams if beams else 0.0


def scan_match(field: LikelihoodField, pose, distances):
    """Window search around ``pose``. Returns ``(best_pose, best_score, base_score)``;
    the caller accepts only a meaningful gain over ``base_score`` (the pose's own)."""
    base = score_pose(field, pose, distances)
    best_pose, best_score = pose, base
    rng, step = config.SCAN_MATCH_RANGE, config.SCAN_MATCH_STEP
    arng, astep = config.SCAN_MATCH_ANGLE, config.SCAN_MATCH_ANGLE_STEP
    for dx in np.arange(-rng, rng + 1e-9, step):
        for dy in np.arange(-rng, rng + 1e-9, step):
            for dth in np.arange(-arng, arng + 1e-9, astep):
                cand = (pose[0] + dx, pose[1] + dy, pose[2] + dth)
                s = score_pose(field, cand, distances)
                if s > best_score:
                    best_pose, best_score = cand, s
    return best_pose, best_score, base


def _damp(a, b, k):
    """Move pose ``a`` a fraction ``k`` toward ``b`` (angle blended across the wrap)."""
    return (a[0] + k * (b[0] - a[0]),
            a[1] + k * (b[1] - a[1]),
            a[2] + k * wrap_angle(b[2] - a[2]))


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
        self._field = None           # cached likelihood field (P0-4)
        self._field_time = 0.0
        self._field_dirty = True

    def set_mapping(self, enabled: bool) -> None:
        with self._lock:
            self.mapping = enabled

    def run(self) -> None:
        period = 1.0 / config.SLAM_HZ
        # Publish the initial grid so the UI has something to draw.
        state.set_grid(self.grid.to_uint8())
        while not state.stop_event.is_set():
            t0 = time.monotonic()
            state.beat("slam")
            self.step()
            elapsed = time.monotonic() - t0
            if elapsed < period:
                time.sleep(period - elapsed)

    def step(self) -> tuple:
        """One SLAM/localization cycle. Returns the fused pose."""
        base_pose = self.odometry.get_pose()
        enc_pose = self.odometry.update()
        readings = sensors.get_all_readings()
        distances = {name: r.distance_cm for name, r in readings.items()}

        grid_uint8 = self.grid.to_uint8()
        field = self._ensure_field(grid_uint8)
        best_pose, best_score, base_score = scan_match(field, enc_pose, distances)
        scan_accepted = best_score - base_score >= config.SCAN_MATCH_MIN_GAIN
        scan_pose = (_damp(enc_pose, best_pose, config.SCAN_MATCH_DAMPING)
                     if scan_accepted else enc_pose)

        # Monocular forward VO has no reliable scale, so it feeds only heading and
        # slip detection -- never x/y (P1-5). dtheta is applied to the pre-update
        # heading, the interval the delta actually covers.
        vo_delta, vo_conf = state.consume_vo()
        vo_pose = (enc_pose[0], enc_pose[1], base_pose[2] + vo_delta[2])
        slip = localization.detect_slip(self.odometry.last_distance, vo_delta, vo_conf)

        w_e, w_s, w_v = localization.compute_weights(best_score, scan_accepted, vo_conf, slip)
        fused = localization.fuse_poses(enc_pose, scan_pose, vo_pose, w_e, w_s, w_v)

        # Feed the correction back so the next dead-reckoning step builds on it.
        self.odometry.set_pose(fused)
        state.set_pose(fused)

        if self.mapping and self._scan_motion_ok(self._prev_pose, fused):
            # Only map sensors whose reading is fresh: a stale ping (the scheduler
            # sweeps each non-priority sensor at ~1.6 Hz) reflects a pose the robot
            # has since left, so integrating it would smear the map.
            fresh = {name: r.distance_cm for name, r in readings.items()
                     if not r.stale}
            with self._lock:
                touched = self.grid.integrate_scan(fused, fresh)
                grid_uint8 = self.grid.to_uint8()
            self._field_dirty = True
            state.set_grid(grid_uint8)
            self._emit_cells(touched, grid_uint8)
        self._prev_pose = fused
        return fused

    def _ensure_field(self, grid_uint8) -> LikelihoodField:
        """Rebuild the likelihood field on a map change or at most ~1 Hz (P0-4/P1-6)."""
        now = time.monotonic()
        if (self._field is None or self._field_dirty
                or now - self._field_time > config.SCAN_MATCH_FIELD_REFRESH_S):
            self._field = LikelihoodField(self.grid, grid_uint8)
            self._field_time, self._field_dirty = now, False
        return self._field

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
