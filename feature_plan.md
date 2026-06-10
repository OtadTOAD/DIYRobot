# Feature Plan & Shared Interfaces
## AI Cargo Robot

---

## Shared Interfaces

These are agreed standards. Every module must follow them exactly. Do not redefine types, variable names, or function signatures locally.

---

### Coordinate conventions

```python
# World coordinates — always float, in metres
x: float  # horizontal position
y: float  # vertical position
theta: float  # heading in radians

# Grid coordinates — always int, in cell indices
col: int
row: int

# Pose tuple — always in this order
pose: tuple[float, float, float]  # (x, y, theta)
```

---

### Shared state — `state.py`

Single file. All threads import from here. All variables protected by locks.

```python
import threading
import numpy as np

# Localization
robot_pose: tuple = (0.0, 0.0, 0.0)      # (x, y, theta) — written: localization, read: navigation, web
pose_lock = threading.RLock()

# Mapping
occupancy_grid: np.ndarray = None          # written: SLAM, read: navigation, web
grid_lock = threading.RLock()

# Safety
blocked: bool = False                      # written: safety monitor, read: navigation
blocked_lock = threading.Lock()

# Camera → localization
vo_estimate: tuple = (0.0, 0.0, 0.0)     # (dx, dy, dtheta)
vo_confidence: float = 0.0
vo_lock = threading.Lock()

# Camera → safety
camera_advisory: bool = False             # written: camera, read: safety monitor
advisory_lock = threading.Lock()

# Mode
current_mode: str = 'idle'               # 'idle' | 'explore' | 'navigate'
mode_lock = threading.Lock()
```

---

### Coordinate conversion — `core/occupancy_grid.py`

```python
def world_to_grid(x: float, y: float) -> tuple[int, int]:
    """Convert world coordinates (metres) to grid cell (col, row)."""

def grid_to_world(col: int, row: int) -> tuple[float, float]:
    """Convert grid cell (col, row) to world coordinates (metres)."""
```

---

### Motor control — `hardware/motors.py`

```python
def set_speed(left: float, right: float) -> None:
    """Set motor speeds. Range 0.0 (stop) to 1.0 (full). Negative = reverse."""

def stop() -> None:
    """Stop all motors immediately."""
```

---

### Sensor readings — `hardware/sensors.py`

```python
def get_distance(sensor: str) -> float:
    """Return distance in cm. sensor = 'front'|'right'|'back'|'left'|'down'.
    Returns float('inf') on timeout."""

def get_all_distances() -> dict[str, float]:
    """Return dict of all 5 sensor readings in cm."""
```

---

### WebSocket event schema — `ui/server.py` → `ui/static/map.js`

```python
# Server → client
"pose_update"  → {"x": float, "y": float, "theta": float}
"path_update"  → [{"x": int, "y": int}, ...]        # list of grid cells
"cell_update"  → [{"x": int, "y": int, "value": int}, ...]  # changed cells only
"mode_change"  → {"mode": "idle" | "explore" | "navigate"}
"status_log"   → {"level": "ok" | "warn" | "info" | "error", "msg": str}

# Client → server
"set_destination" → {"x": int, "y": int}             # grid coordinates from click
"draw_zone"       → {"x1": int, "y1": int, "x2": int, "y2": int}
```

---

### Naming conventions

- Functions: `snake_case`
- Constants in `config.py`: `UPPER_SNAKE_CASE`
- Classes: `PascalCase`
- All distances internally in metres (convert cm from sensors immediately)
- All angles in radians
- Grid cells always `(col, row)` — never `(row, col)` or `(x, y)` for grid indices

---

## Features

Features are ordered by dependency — implement earlier features before later ones that depend on them.

---

### F-01 — Project scaffolding & shared state

**Category:** Infrastructure
**Complexity:** Low
**Files:** `config.py`, `state.py`, `main.py`, `requirements.txt`
**Depends on:** Nothing — implement first

Create full folder structure. `config.py` with all GPIO pins, speed limits, sensor thresholds, map resolution, timing constants — no magic numbers anywhere else in the codebase. `state.py` with all shared variables and locks as defined above. `requirements.txt`. `main.py` skeleton. `.gitignore`.

---

### F-02 — Motor control

**Category:** Hardware
**Complexity:** Low
**Files:** `hardware/motors.py`, `hardware/gpio_cleanup.py`
**Depends on:** F-01

L298N PWM control via pigpio on GPIO 12 (ENA) and GPIO 13 (ENB). Implement `set_speed(left, right)` with 0.0–1.0 range, negative for reverse. GPIO cleanup handler — motors stop and all pins reset on shutdown. Test: robot drives forward, reverse, turns left, turns right on command.

---

### F-03 — Ultrasonic sensors

**Category:** Hardware
**Complexity:** Low
**Files:** `hardware/sensors.py`
**Depends on:** F-01

Read all 5 HC-SR04 sensors via TRIG/ECHO GPIO pairs. TRIG pulse → wait for ECHO rising edge → measure duration → convert to cm. Timeout handling: return `float('inf')` if no echo within 30ms. Implement `get_distance(sensor)` and `get_all_distances()` as defined in interfaces. Verify voltage dividers on all ECHO pins before testing.

---

### F-04 — Encoder odometry

**Category:** Hardware
**Complexity:** Medium
**Files:** `core/odometry.py`
**Depends on:** F-01, F-02

Interrupt-driven pulse counting on GPIO 20 (left) and GPIO 21 (right). Differential drive kinematics each update cycle:

```
delta_left  = left_pulses  × dist_per_pulse
delta_right = right_pulses × dist_per_pulse
delta_c     = (delta_left + delta_right) / 2
delta_theta = (delta_right - delta_left) / wheel_base
x     += delta_c × cos(theta)
y     += delta_c × sin(theta)
theta += delta_theta
```

Writes updated `robot_pose` to `state.py` under `pose_lock` each cycle.

---

### F-05 — Safety monitor

**Category:** Hardware
**Complexity:** Medium
**Files:** `core/safety_monitor.py`
**Depends on:** F-01, F-02, F-03

Independent daemon thread at ~20 Hz. Priority hierarchy: this module's decisions override all navigation logic with no exceptions.

- Horizontal obstacle: any of front/left/right/back reads below `STOP_THRESHOLD` → `stop()`, set `blocked = True`
- Frontal block recovery: reverse briefly, pivot away, set `blocked = False` to allow retry
- Drop detection: downward > 23 cm → emergency stop, requires manual acknowledgement via web UI before resuming
- Drop detection: downward < 7 cm → stop and reverse
- Reads `camera_advisory` flag from `state.py` — when True, tighten obstacle thresholds

---

### F-06 — USB camera capture

**Category:** Camera
**Complexity:** Low
**Files:** `hardware/camera.py`
**Depends on:** F-01

Initialize `cv2.VideoCapture(0)` at 640×480, 30 fps. Verify `/dev/video0` is present — raise clear error if not. Capture loop as daemon thread. Write latest frame to shared variable under `threading.Lock`. This module is the base that F-07 and F-08 build on.

---

### F-07 — Visual odometry pipeline

**Category:** Camera
**Complexity:** High
**Files:** `hardware/camera.py`
**Depends on:** F-06

Shi-Tomasi corner detection via `cv2.goodFeaturesToTrack` (max 150 features, qualityLevel=0.01, minDistance=10). Lucas-Kanade pyramidal optical flow via `cv2.calcOpticalFlowPyrLK` (winSize 21×21, maxLevel 3). Estimate motion (dx, dy, dtheta) from average feature point movement between frames. Re-detect features when tracked count drops below 30.

Confidence scoring:
```python
def flow_confidence(n_tracked, flow):
    consistency = 1.0 - (np.std(flow) / (np.mean(np.abs(flow)) + 1e-5))
    return float(np.clip(n_tracked / 150 * consistency, 0, 1))
```

Writes `vo_estimate` and `vo_confidence` to `state.py` under `vo_lock`.

---

### F-08 — Appearance-based obstacle detection

**Category:** Camera
**Complexity:** Medium
**Files:** `hardware/camera.py`
**Depends on:** F-06

ROI: bottom 45% of frame only. Gaussian blur (5×5) → Canny edge detection (threshold 50/150) → `cv2.findContours`. Filter contours by minimum area. If significant contours appear in lower-centre region, set `camera_advisory = True` in `state.py` under `advisory_lock`. Advisory only — does not call `stop()` directly.

---

### F-09 — Occupancy grid

**Category:** Core AI
**Complexity:** High
**Files:** `core/occupancy_grid.py`
**Depends on:** F-01, F-03, F-04

NumPy uint8 2D array, all cells initialised to 50 (unknown).

Log-odds Bayesian update:
```python
def update_cell(log_odds_grid, col, row, p_evidence):
    lo = log_odds_grid[row, col] + math.log(p_evidence / (1 - p_evidence))
    log_odds_grid[row, col] = max(-10, min(10, lo))
```

Bresenham ray casting per sensor reading — mark cells along beam as free (p=0.3), endpoint as occupied (p=0.7).

Obstacle inflation after exploration:
```python
from scipy.ndimage import binary_dilation
inflated = binary_dilation(grid > 65, iterations=2)
grid[inflated] = 100
```

Implement `world_to_grid()` and `grid_to_world()` coordinate converters. msgpack save/load: grid bytes + metadata dict + waypoints dict. PNG export for web rendering.

---

### F-10 — SLAM loop & scan matching

**Category:** Core AI
**Complexity:** High
**Files:** `core/slam.py`
**Depends on:** F-09, F-04, F-03

10 Hz daemon thread. Each cycle: read all sensors → get current pose from `state.py` → ray cast each reading into the grid → update occupancy grid.

Correlation window scan matching for pose correction:
- Search ±15 cm, ±10° around current pose estimate
- Score each candidate by matching sensor readings against existing map cells
- Accept correction only when score exceeds `SCAN_MATCH_THRESHOLD`
- Write corrected pose to `state.py`

Frontier detection: scan grid for cells that are free (value < 35) and have at least one unknown neighbour (value == 50). Returns list of frontier cell coordinates for use by explore mode.

---

### F-11 — Localization fusion

**Category:** Core AI
**Complexity:** High
**Files:** `core/localization.py`
**Depends on:** F-04, F-10, F-07

Three-source weighted fusion running each SLAM cycle:

```python
def fuse_poses(enc_pose, scan_pose, vo_pose,
               enc_conf, scan_conf, vo_conf):
    total = enc_conf + scan_conf + vo_conf
    w_e = enc_conf / total
    w_s = scan_conf / total
    w_v = vo_conf  / total
    x     = w_e*enc_pose[0] + w_s*scan_pose[0] + w_v*vo_pose[0]
    y     = w_e*enc_pose[1] + w_s*scan_pose[1] + w_v*vo_pose[1]
    theta = w_e*enc_pose[2] + w_s*scan_pose[2] + w_v*vo_pose[2]
    return (x, y, theta)
```

Default confidences: encoders 0.55, scan matching 0.30, visual odometry 0.15. Adjust dynamically:
- Scan matching confidence drops when match score is below threshold
- VO confidence drops when `vo_confidence` from `state.py` is low
- Wheel slip: encoder reports motion > `SLIP_THRESHOLD` while `vo_estimate` reports near-zero → temporarily halve encoder confidence

Writes final fused `robot_pose` to `state.py` under `pose_lock`.

---

### F-12 — A* path planner

**Category:** Core AI
**Complexity:** High
**Files:** `core/path_planner.py`
**Depends on:** F-09, F-13

8-directional movement. Cardinal cost: 1.0. Diagonal cost: √2. Euclidean heuristic. `heapq` min-heap open set. NumPy boolean closed set. Walkable: `grid[row, col] < 65`.

Visibility-based path smoothing — walk path, use Bresenham to check line-of-sight between non-adjacent waypoints, remove intermediate nodes when clear.

Replanning: on `blocked = True`, wait 0.5s and retry up to 3 times. After 3 failures, run full A* from current pose.

Returns smoothed list of `(col, row)` grid cells.

---

### F-13 — Forbidden zones

**Category:** Core AI
**Complexity:** Low
**Files:** `core/forbidden_zones.py`
**Depends on:** F-09

Constraint layer stored as a separate NumPy boolean array, same dimensions as occupancy grid. Hard-block cells: A* treats them as occupied (unwalkable) regardless of grid value.

```python
def add_zone(x1: int, y1: int, x2: int, y2: int) -> None
def clear_all() -> None
def is_blocked(col: int, row: int) -> bool
def save(path: str) -> None
def load(path: str) -> None
```

Saved as `.zones` file alongside the `.map` file — never baked into the occupancy grid itself.

---

### F-14 — Explore mode

**Category:** Core AI
**Complexity:** Medium
**Files:** `modes/explore.py`
**Depends on:** F-10, F-12, F-02, F-05

Frontier-based exploration loop:
1. Get frontier cells from SLAM module
2. Group nearby frontiers into regions (BFS clustering)
3. Navigate to centroid of nearest reachable region using A*
4. As robot arrives, new frontiers discovered — repeat
5. When no frontiers remain: run obstacle inflation, save map to disk

Handles mid-exploration interruption cleanly — saves partial map on mode switch.

---

### F-15 — Navigate & idle modes

**Category:** Core AI
**Complexity:** Medium
**Files:** `modes/navigate.py`, `modes/idle.py`
**Depends on:** F-12, F-11, F-05, F-02

Navigate route execution loop:
1. Compute heading error to next waypoint: `atan2(dy, dx) - theta`
2. Rotate in place to align (use `set_speed` with opposite wheel directions)
3. Drive forward
4. Check `blocked` flag each iteration — pause immediately if True
5. Update pose from `state.py`
6. Advance to next waypoint when within `ARRIVAL_THRESHOLD` (8–10 cm)
7. On goal reached: stop, emit `status_log` event

Idle mode: `stop()`, set `current_mode = 'idle'`, loop waiting for mode change.

---

### F-16 — Flask backend & API endpoints

**Category:** Web UI
**Complexity:** Medium
**Files:** `ui/server.py`
**Depends on:** F-01, F-09, F-13, F-15

HTTP endpoints:

| Endpoint | Method | Action |
|---|---|---|
| `/` | GET | Serve `index.html` |
| `/map/list` | GET | List `.map` files in `maps/` |
| `/map/load` | POST | Load map by name |
| `/map/save` | POST | Save current map with name |
| `/map/export` | GET | Download PNG of current grid |
| `/mode` | POST | Request mode change |
| `/navigate` | POST | Set destination cell and start navigation |
| `/waypoints` | GET | List saved waypoints |
| `/waypoints/add` | POST | Add or update named waypoint |
| `/waypoints/delete` | POST | Delete waypoint by name |
| `/forbidden/add` | POST | Add forbidden zone rectangle |
| `/forbidden/clear` | POST | Clear all forbidden zones |

---

### F-17 — WebSocket & mode state machine

**Category:** Web UI
**Complexity:** Medium
**Files:** `ui/server.py`, `main.py`
**Depends on:** F-16, F-14, F-15

Flask-SocketIO setup. Emit outbound events on state changes using event schema defined in shared interfaces. Handle inbound `set_destination` and `draw_zone` events.

Mode state machine — valid transitions:

```
idle      → explore    ✅
idle      → navigate   ✅ (requires map loaded)
explore   → idle       ✅ (saves partial map)
explore   → navigate   ✅ (uses partial map)
navigate  → idle       ✅ (stops immediately)
navigate  → explore    ✅
```

Invalid transitions return error status. Each transition cleanly stops the current mode thread before starting the new one.

---

### F-18 — Map canvas & live rendering

**Category:** Web UI
**Complexity:** Medium
**Files:** `ui/static/index.html`, `ui/static/map.js`
**Depends on:** F-17

HTML5 Canvas. Layers drawn in order:
1. Base occupancy grid (white = free, dark = wall, grey = unknown)
2. Inflated obstacle regions (slightly darker shade)
3. Forbidden zones (semi-transparent red)
4. Planned path (dashed blue line)
5. Named waypoint markers with labels
6. Robot position as arrow with heading direction

On connect: request full grid PNG, draw as base layer. Subscribe to `pose_update`, `path_update`, `cell_update` — apply delta updates without full redraw. Pixel → grid coordinate conversion for click-to-navigate.

---

### F-19 — Control panel

**Category:** Web UI
**Complexity:** Medium
**Files:** `ui/static/ui.js`
**Depends on:** F-18

- Mode buttons (Idle / Explore / Navigate) with visual state feedback
- Destination: click map (pixel→grid) or named waypoint dropdown + Go button
- Waypoint list: add, delete, navigate-to per item
- Forbidden zone draw tool: click and drag rectangle on canvas
- Map management: load, save, switch, export PNG
- Status log: live `status_log` WebSocket events, auto-scroll

---

### F-20 — Thread orchestration & main entry point

**Category:** Infrastructure
**Complexity:** Medium
**Files:** `main.py`
**Depends on:** All features

Wire all threads and startup sequence:

```
1. Load config.py
2. Initialise GPIO
3. Start camera daemon thread (F-06)
4. Start safety monitor daemon thread (F-05)
5. Start SLAM daemon thread (F-10)
6. Start Flask/SocketIO server (F-16, F-17)
7. Enter idle mode (F-15)
```

SIGINT handler: set stop event → wait for threads to exit → `stop()` motors → GPIO cleanup → exit.

---

## Dependency Order

Implement in this sequence to avoid blockers:

```
F-01 → F-02, F-03, F-06
F-02 → F-04, F-05
F-03 → F-05
F-04 → F-09, F-11
F-05 → F-14, F-15
F-06 → F-07, F-08
F-07 → F-11
F-09 → F-10, F-12, F-13
F-10 → F-11, F-14
F-11 → F-15
F-12 → F-14, F-15
F-13 → F-12
F-14, F-15 → F-16, F-17
F-16, F-17 → F-18
F-18 → F-19
All → F-20
```

---

*Feature plan — AI Cargo Robot*
