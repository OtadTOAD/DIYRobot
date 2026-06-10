"""2D differential-drive simulator -- the ground truth behind the sim backend.

When the robot is not running on a Raspberry Pi, the hardware backends
(``hardware/backends/sim.py``) talk to a single :class:`World` instance instead of
real GPIO. The world holds:

* a ground-truth map (walls + obstacles as line segments, plus cliff zones),
* the ground-truth robot pose, integrated from commanded wheel speeds,
* synthetic ultrasonic readings (ray vs. segment intersection + noise),
* synthetic encoder pulse counts (wheel travel + slip),
* a first-person camera frame raycast from the wall segments (Wolfenstein-style:
  one ray per column, wall height ~ 1/distance, textured walls, shaded floor and
  ceiling), so the real Shi-Tomasi + Lucas-Kanade visual-odometry and appearance
  pipelines see what a forward-facing camera in this world would actually see.

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


def generate_bsp_world(seed: int | None = None):
    """Recursive-division floor plan: cut a square in two, leave a door, recurse.

    All coordinates snap to a lattice whose pitch is one door width, and every
    dividing wall gets exactly one one-cell door, so the resulting rooms are always
    fully connected (a wall is zero-width and can at most touch a door's edge).
    The lattice is offset half a cell from the world origin so no wall line can
    pass through the default start pose at (0, 0).
    """
    rng = np.random.default_rng(config.SIM_BSP_SEED if seed is None else seed)
    pitch = config.SIM_BSP_DOOR
    n = int(round(config.SIM_BSP_SIZE / pitch))
    min_cells = max(2, int(math.ceil(config.SIM_BSP_MIN_ROOM / pitch)))

    def to_world(c, r):
        return ((c - n / 2) * pitch + pitch / 2, (r - n / 2) * pitch + pitch / 2)

    segs = []   # ((c0, r0), (c1, r1)) in lattice coordinates

    def split(c0, r0, c1, r1):
        w, h = c1 - c0, r1 - r0
        can_v, can_h = w >= 2 * min_cells, h >= 2 * min_cells
        if not can_v and not can_h:
            return
        vertical = can_v and (not can_h or w > h or (w == h and rng.random() < 0.5))
        if vertical:
            c = int(rng.integers(c0 + min_cells, c1 - min_cells + 1))
            door = int(rng.integers(r0, r1))
            if door > r0:
                segs.append(((c, r0), (c, door)))
            if door + 1 < r1:
                segs.append(((c, door + 1), (c, r1)))
            split(c0, r0, c, r1)
            split(c, r0, c1, r1)
        else:
            r = int(rng.integers(r0 + min_cells, r1 - min_cells + 1))
            door = int(rng.integers(c0, c1))
            if door > c0:
                segs.append(((c0, r), (door, r)))
            if door + 1 < c1:
                segs.append(((door + 1, r), (c1, r)))
            split(c0, r0, c1, r)
            split(c0, r, c1, r1)

    split(0, 0, n, n)
    x0, y0 = to_world(0, 0)
    x1, y1 = to_world(n, n)
    walls = _rect_walls(x0, y0, x1, y1)
    walls += [(to_world(*a), to_world(*b)) for a, b in segs]
    return walls, []


def _build_world(name: str, seed: int | None = None):
    if name == "bsp":
        return generate_bsp_world(seed)

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

    def __init__(self, world_name: str | None = None, start_pose=None, seed=None):
        self.walls, self.cliffs = _build_world(world_name or config.SIM_WORLD, seed)
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

        # Wall texture for the first-person renderer + per-segment raycast arrays.
        self._texture = self._make_texture()
        self._wall_a, self._wall_b, self._wall_len, self._wall_uoff = (
            self._precompute_walls())
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
        # and stable Shi-Tomasi corners (high-frequency per-pixel noise defeats LK),
        # without the strong Canny edges that sharp marks would create (which would
        # trip the appearance detector on every frame). Columns wrap horizontally
        # along the walls; rows span the wall's vertical extent.
        low = rng.integers(60, 200, size=(16, 256)).astype(np.uint8)
        return cv2.resize(low, (4096, 256), interpolation=cv2.INTER_CUBIC)

    def _precompute_walls(self):
        a = np.array([seg[0] for seg in self.walls], dtype=np.float64)
        b = np.array([seg[1] for seg in self.walls], dtype=np.float64)
        length = np.hypot(b[:, 0] - a[:, 0], b[:, 1] - a[:, 1])
        # Cumulative offset gives each wall a distinct, world-anchored texture span.
        u_offset = np.concatenate([[0.0], np.cumsum(length)[:-1]])
        return a, b, length, u_offset

    def render_camera(self) -> np.ndarray:
        """Raycast a first-person BGR frame of the wall segments (Wolfenstein-style).

        One ray per image column; wall band height is proportional to 1/distance,
        walls sample a world-anchored smooth texture, and floor/ceiling are shaded
        by the distance each row sees, so brightness stays continuous across the
        wall base (no fake Canny edges) while motion produces geometrically correct
        optical flow for the forward-camera VO model.
        """
        with self._lock:
            x, y, theta, obstacle = self.x, self.y, self.theta, self.camera_obstacle
        h, w = config.CAMERA_HEIGHT, config.CAMERA_WIDTH
        fov = config.CAMERA_FOV
        half_wall = config.SIM_WALL_HALF_HEIGHT
        falloff = config.SIM_SHADE_FALLOFF
        a, b, length, u_off = self._wall_a, self._wall_b, self._wall_len, self._wall_uoff

        # One ray per column at equal angular steps; column 0 is leftmost in view.
        rel = fov / 2 - (np.arange(w) + 0.5) * fov / w            # (W,)
        ang = theta + rel
        dx, dy = np.cos(ang), np.sin(ang)
        ex, ey = b[:, 0] - a[:, 0], b[:, 1] - a[:, 1]             # (N,)
        ax0, ay0 = a[:, 0] - x, a[:, 1] - y
        denom = dx[:, None] * ey - dy[:, None] * ex               # (W, N)
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (ax0 * ey - ay0 * ex) / denom
            s = (ax0 * dy[:, None] - ay0 * dx[:, None]) / denom
        valid = (np.abs(denom) > 1e-12) & (t > 1e-9) & (s >= 0.0) & (s <= 1.0)
        t = np.where(valid, t, np.inf)
        idx = np.argmin(t, axis=1)                                # nearest wall per column
        cols = np.arange(w)
        tmin = t[cols, idx]
        hit = np.isfinite(tmin)
        s_hit = np.clip(np.where(hit, s[cols, idx], 0.0), 0.0, 1.0)
        d_perp = np.where(hit, tmin, 1e6) * np.cos(rel)           # fisheye correction

        f_px = (w / 2) / math.tan(fov / 2)                        # focal length, px
        half_px = f_px * half_wall / np.maximum(d_perp, 1e-3)     # (W,)
        horizon = h / 2.0
        top, bot = horizon - half_px, horizon + half_px

        # Floor/ceiling: each row below/above the horizon sees the ground/ceiling
        # plane at this distance; shading by it matches the wall shade at the wall
        # base exactly, so the wall-floor boundary is smooth.
        ys = np.arange(h, dtype=np.float64)[:, None]              # (H, 1)
        d_fc = f_px * half_wall / np.maximum(np.abs(ys - horizon), 0.5)
        base_gray = np.where(ys > horizon, config.SIM_FLOOR_GRAY, config.SIM_CEIL_GRAY)
        img = np.repeat(base_gray / (1.0 + d_fc * falloff), w, axis=1)

        # Wall band: texture by (distance along wall, height in band), faded into
        # the floor/ceiling gray near the band edges (again: no hard horizontal edge).
        wall_mask = (ys >= top) & (ys < bot)
        v = (ys - top) / np.maximum(2.0 * half_px, 1e-6)          # (H, W) 0..1 in band
        tex_h, tex_w = self._texture.shape
        u_px = ((u_off[idx] + s_hit * length[idx]) * config.SIM_WALL_TEX_PPM)
        u_px = u_px.astype(np.int64) % tex_w
        v_px = np.clip((v * tex_h).astype(np.int64), 0, tex_h - 1)
        tex = self._texture[v_px, np.broadcast_to(u_px, (h, w))]
        window = np.clip((0.5 - np.abs(v - 0.5)) * 6.0, 0.0, 1.0)
        wall_val = (tex * window + base_gray * (1.0 - window)) / (1.0 + d_perp * falloff)
        img = np.where(wall_mask & hit, wall_val, img)

        gray = np.clip(img, 0, 255).astype(np.uint8)
        frame = np.dstack([gray, gray, gray])

        if obstacle:
            import cv2
            # A solid object with a continuous high-contrast silhouette low-centre,
            # so the appearance detector sees a closed contour regardless of the
            # floor texture behind it (a real obstacle has a continuous edge).
            bw, bh = w // 4, h // 5
            x0, y0 = (w - bw) // 2, int(h * 0.7)
            frame[y0:y0 + bh, x0:x0 + bw] = (20, 20, 20)
            cv2.rectangle(frame, (x0, y0), (x0 + bw, y0 + bh), (240, 240, 240), 3)
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


def reset_world(world_name: str | None = None, start_pose=None, seed=None) -> World:
    """Replace the singleton (used by tests and at startup)."""
    global _world
    with _world_lock:
        if _world is not None:
            _world.stop()
        _world = World(world_name=world_name, start_pose=start_pose, seed=seed)
        return _world
