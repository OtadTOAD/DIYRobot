"""2D differential-drive simulator -- the ground truth behind the sim backend.

When the robot is not running on a Raspberry Pi, the hardware backends
(``hardware/backends/sim.py``) talk to a single :class:`World` instance instead of
real GPIO. The world holds:

* a ground-truth map (walls + obstacles as line segments, plus cliff zones),
* the ground-truth robot pose, integrated from commanded wheel speeds,
* synthetic ultrasonic readings (ray vs. segment intersection + noise),
* synthetic encoder pulse counts (wheel travel + slip),
* a synthetic camera frame that translates/rotates with motion so the real
  Shi-Tomasi + Lucas-Kanade visual-odometry pipeline recovers the motion.

The same SLAM / A* / explore / navigate / web code therefore runs end-to-end on a
laptop. A background physics thread advances the world in real time; tests can also
drive it deterministically with :meth:`step`.
"""

from __future__ import annotations

import math
import threading
import time

import numpy as np

from robot_car import config


# ---------------------------------------------------------------------------
# Built-in worlds. A world is a list of wall segments ((x0, y0), (x1, y1)) in
# metres, plus optional cliff rectangles (x_min, y_min, x_max, y_max) where the
# floor "drops away" for the downward sensor.
# ---------------------------------------------------------------------------
def _rect_walls(x0, y0, x1, y1):
    return [
        ((x0, y0), (x1, y0)),
        ((x1, y0), (x1, y1)),
        ((x1, y1), (x0, y1)),
        ((x0, y1), (x0, y0)),
    ]


def _build_world(name: str):
    if name == "empty":
        return _rect_walls(-2.0, -2.0, 2.0, 2.0), []

    if name == "obstacles":
        walls = _rect_walls(-2.4, -2.4, 2.4, 2.4)
        walls += _rect_walls(0.6, 0.6, 1.2, 1.2)      # box obstacle
        walls += _rect_walls(-1.4, -0.4, -1.0, 0.8)   # tall obstacle
        return walls, []

    # default: 'room' -- a rectangular room with interior furniture and a doorway
    walls = []
    # Outer walls (4.8 x 4.8 m room centred on origin) with a doorway gap on +x.
    walls += [((-2.4, -2.4), (2.4, -2.4))]            # bottom
    walls += [((2.4, -2.4), (2.4, -0.4))]             # right lower
    walls += [((2.4, 0.4), (2.4, 2.4))]               # right upper (doorway 0.8 m)
    walls += [((2.4, 2.4), (-2.4, 2.4))]              # top
    walls += [((-2.4, 2.4), (-2.4, -2.4))]            # left
    # Interior "furniture" (origin is intentionally left clear so the robot's
    # default start pose is not inside an obstacle).
    walls += _rect_walls(-1.6, 1.0, -0.8, 1.6)        # desk (top-left)
    walls += _rect_walls(0.8, -1.8, 1.6, -1.0)        # cabinet (bottom-right)
    walls += _rect_walls(0.4, 0.4, 0.9, 0.9)          # small pillar (off-centre)
    return walls, []


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def _ray_segment_distance(ox, oy, dx, dy, ax, ay, bx, by):
    """Distance from ray origin (ox,oy) dir (dx,dy) to segment a-b, or None."""
    # Solve o + t*d = a + s*(b-a), t >= 0, 0 <= s <= 1
    ex, ey = bx - ax, by - ay
    denom = dx * ey - dy * ex
    if abs(denom) < 1e-12:
        return None
    t = ((ax - ox) * ey - (ay - oy) * ex) / denom
    s = ((ax - ox) * dy - (ay - oy) * dx) / denom
    if t >= 0 and 0.0 <= s <= 1.0:
        return t
    return None


def _point_segment_distance(px, py, ax, ay, bx, by):
    ex, ey = bx - ax, by - ay
    length2 = ex * ex + ey * ey
    if length2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * ex + (py - ay) * ey) / length2))
    cx, cy = ax + t * ex, ay + t * ey
    return math.hypot(px - cx, py - cy)


class World:
    """Ground-truth simulated environment and robot state."""

    def __init__(self, world_name: str | None = None, start_pose=None):
        self.walls, self.cliffs = _build_world(world_name or config.SIM_WORLD)
        sp = start_pose if start_pose is not None else config.SIM_START_POSE
        self.x, self.y, self.theta = float(sp[0]), float(sp[1]), float(sp[2])

        self._left_cmd = 0.0      # commanded wheel speed fraction (-1..1)
        self._right_cmd = 0.0
        self._left_travel = 0.0   # metres of accumulated travel (for encoders)
        self._right_travel = 0.0
        self._left_pulse_accum = 0.0
        self._right_pulse_accum = 0.0
        self._left_pulses = 0     # whole pulses not yet consumed by a read
        self._right_pulses = 0

        self._rng = np.random.default_rng(1234)
        self._lock = threading.RLock()
        self._thread: threading.Thread | None = None
        self._running = False
        self._last_time = None

        # Synthetic camera texture (static feature-rich scene we pan/rotate over).
        self._texture = self._make_texture()
        # An obstacle blob that can be injected into the camera's lower ROI.
        self.camera_obstacle = False

    # -- motor interface -----------------------------------------------------
    def set_motor(self, left: float, right: float) -> None:
        with self._lock:
            self._left_cmd = max(-1.0, min(1.0, float(left)))
            self._right_cmd = max(-1.0, min(1.0, float(right)))

    # -- physics -------------------------------------------------------------
    def step(self, dt: float) -> None:
        """Advance ground truth by ``dt`` seconds."""
        with self._lock:
            vl = self._left_cmd * config.MAX_SPEED_MPS
            vr = self._right_cmd * config.MAX_SPEED_MPS

            # Apply a little random slip to the actual wheel travel.
            slip_l = 1.0 - self._rng.uniform(0, config.SIM_ENCODER_SLIP)
            slip_r = 1.0 - self._rng.uniform(0, config.SIM_ENCODER_SLIP)
            dl = vl * dt * slip_l
            dr = vr * dt * slip_r

            v = (dl + dr) / 2.0
            dtheta = (dr - dl) / config.WHEEL_BASE

            new_theta = self.theta + dtheta
            new_x = self.x + v * math.cos(self.theta + dtheta / 2.0)
            new_y = self.y + v * math.sin(self.theta + dtheta / 2.0)

            # Reject translations that would drive through a wall.
            if not self._collides(new_x, new_y):
                self.x, self.y = new_x, new_y
            self.theta = new_theta

            # Encoders count |travel| regardless of direction (single channel).
            self._accumulate_pulses(abs(dl), abs(dr))

    def _collides(self, x, y) -> bool:
        margin = config.ROBOT_RADIUS
        for (ax, ay), (bx, by) in self.walls:
            if _point_segment_distance(x, y, ax, ay, bx, by) < margin:
                return True
        return False

    def _accumulate_pulses(self, dl, dr):
        self._left_travel += dl
        self._right_travel += dr
        self._left_pulse_accum += dl / config.DIST_PER_PULSE
        self._right_pulse_accum += dr / config.DIST_PER_PULSE
        whole_l = int(self._left_pulse_accum)
        whole_r = int(self._right_pulse_accum)
        self._left_pulse_accum -= whole_l
        self._right_pulse_accum -= whole_r
        self._left_pulses += whole_l
        self._right_pulses += whole_r

    # -- encoder interface ---------------------------------------------------
    def read_encoder_pulses(self) -> tuple[int, int]:
        """Return (left, right) pulses since the previous read, then reset."""
        with self._lock:
            l, r = self._left_pulses, self._right_pulses
            self._left_pulses = 0
            self._right_pulses = 0
            return l, r

    # -- sensor interface ----------------------------------------------------
    def read_distance_cm(self, sensor: str) -> float:
        with self._lock:
            if sensor == "down":
                return self._read_down_cm()
            angle = self.theta + config.SENSOR_ANGLES[sensor]
            dx, dy = math.cos(angle), math.sin(angle)
            best = None
            for (ax, ay), (bx, by) in self.walls:
                t = _ray_segment_distance(self.x, self.y, dx, dy, ax, ay, bx, by)
                if t is not None and (best is None or t < best):
                    best = t
            if best is None:
                return float("inf")
            dist_cm = best * 100.0 + self._rng.normal(0, config.SIM_SENSOR_NOISE_CM)
            if dist_cm > config.SENSOR_MAX_RANGE_CM:
                return float("inf")
            return max(1.0, dist_cm)

    def _read_down_cm(self) -> float:
        for (xmin, ymin, xmax, ymax) in self.cliffs:
            if xmin <= self.x <= xmax and ymin <= self.y <= ymax:
                return config.DROP_FLOOR_GONE_CM + 10.0   # floor gone
        return config.DROP_NORMAL_CM + self._rng.normal(0, 0.3)

    # -- camera interface ----------------------------------------------------
    def _make_texture(self) -> np.ndarray:
        import cv2
        rng = np.random.default_rng(7)
        # Low-resolution noise upscaled to large smooth blobs -> trackable gradients
        # and stable Shi-Tomasi corners (high-frequency per-pixel noise defeats LK).
        low = rng.integers(0, 255, size=(80, 80), dtype=np.uint8)
        tex = cv2.resize(low, (1600, 1600), interpolation=cv2.INTER_CUBIC)
        # Smooth gradients give Shi-Tomasi/LK plenty to track without the strong
        # Canny edges that sharp marks would create (which would trip the appearance
        # detector on every frame). The blobs alone are enough for the VO pipeline.
        return tex

    def render_camera(self) -> np.ndarray:
        """Render a synthetic top-down BGR frame that pans/rotates with the pose.

        World (x, y) maps to texture pixels at ``VO_PIXELS_PER_METRE`` so the visual
        odometry pipeline recovers true motion; the frame is rotated by ``-theta``.
        """
        with self._lock:
            x, y, theta, obstacle = self.x, self.y, self.theta, self.camera_obstacle
        h, w = config.CAMERA_HEIGHT, config.CAMERA_WIDTH
        cx, cy = self._texture.shape[1] / 2, self._texture.shape[0] / 2
        scale = config.VO_PIXELS_PER_METRE
        px = cx + x * scale
        py = cy - y * scale
        cos_t, sin_t = math.cos(-theta), math.sin(-theta)

        ys, xs = np.mgrid[0:h, 0:w].astype(np.float32)
        xs -= w / 2.0
        ys -= h / 2.0
        sample_x = (px + xs * cos_t - ys * sin_t).astype(np.int32) % self._texture.shape[1]
        sample_y = (py + xs * sin_t + ys * cos_t).astype(np.int32) % self._texture.shape[0]
        gray = self._texture[sample_y, sample_x]
        frame = np.dstack([gray, gray, gray])

        if obstacle:
            import cv2
            # A solid object with a continuous high-contrast silhouette low-centre,
            # so the appearance detector sees a closed contour regardless of the
            # floor texture behind it (a real obstacle has a continuous edge).
            y0 = int(h * 0.7)
            x0 = w // 2 - 60
            frame[y0:y0 + 80, x0:x0 + 120] = (20, 20, 20)
            cv2.rectangle(frame, (x0, y0), (x0 + 120, y0 + 80), (240, 240, 240), 3)
        return frame

    # -- lifecycle -----------------------------------------------------------
    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._last_time = time.monotonic()
        self._thread = threading.Thread(target=self._run, name="sim-physics", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        dt = 1.0 / config.SIM_TICK_HZ
        while self._running:
            now = time.monotonic()
            elapsed = now - self._last_time
            self._last_time = now
            self.step(min(elapsed, 0.1))
            time.sleep(dt)

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def get_truth_pose(self) -> tuple:
        with self._lock:
            return (self.x, self.y, self.theta)

    def set_truth_pose(self, x, y, theta) -> None:
        with self._lock:
            self.x, self.y, self.theta = float(x), float(y), float(theta)


# ---------------------------------------------------------------------------
# Process-wide singleton accessor
# ---------------------------------------------------------------------------
_world: World | None = None
_world_lock = threading.Lock()


def get_world() -> World:
    global _world
    with _world_lock:
        if _world is None:
            _world = World()
        return _world


def reset_world(world_name: str | None = None, start_pose=None) -> World:
    """Replace the singleton (used by tests and at startup)."""
    global _world
    with _world_lock:
        if _world is not None:
            _world.stop()
        _world = World(world_name=world_name, start_pose=start_pose)
        return _world
