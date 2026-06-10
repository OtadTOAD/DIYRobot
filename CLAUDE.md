# CLAUDE.md — AI Cargo Robot

Guidance for working in this repository. This file is the canonical plan + architecture
reference; the original design rationale lives in the seven `*.md` docs at the repo root.

## What this is

A classical-AI autonomous indoor cargo robot for the Raspberry Pi 2 (any newer Pi
also works; the design docs predate the board swap and say 4B), implemented in
Python under `robot_car/`. It does occupancy-grid SLAM, three-source localization
fusion, A* navigation, reactive safety, USB-camera visual odometry + appearance
detection, and a Flask/SocketIO web UI — entirely onboard, no ROS, no neural nets.

All 20 features from `feature_plan.md` (F-01…F-20) are implemented and tested.

## The one thing to understand first: backends

The code auto-detects its platform (`hardware/platform_detect.py`):

- **On a Raspberry Pi** → the **real** backend drives GPIO via `pigpio` and the USB
  camera via `cv2.VideoCapture` (`hardware/backends/real.py`).
- **Anywhere else** (laptop / CI) → the **simulator** backend (`hardware/backends/sim.py`)
  feeds synthetic sensor / encoder / camera data from a 2D world
  (`core/simulator.py`). The *entire* stack — SLAM, A*, explore, navigate, web UI,
  camera debug view — runs and is testable unchanged.

Force a backend with `ROBOT_BACKEND=pi|sim` (default `auto`). The public hardware
modules (`motors`, `sensors`, `camera`) never import GPIO libraries directly — they go
through `hardware/hal.py`. **Keep it that way**: anything Pi-specific belongs in the
real backend, behind the HAL.

## Run it

```bash
python -m venv .venv && . .venv/bin/activate
pip install -r robot_car/requirements.txt        # laptop / CI (no GPIO libs)
# pip install -r robot_car/requirements-pi.txt    # on the Pi (adds RPi.GPIO, pigpio)

python -m robot_car.main                          # idle; open http://localhost:5000
python -m robot_car.main --explore                # autonomous mapping
python -m robot_car.main --map living_room --waypoint desk
```

Web UI: live map, click-to-navigate, mode buttons, waypoints, forbidden zones, map
save/load/export, status log, and the **camera debug view** (`/camera/debug.mjpg`) —
an MJPEG stream of the annotated frames (tracked VO features + detected contours +
VO/confidence text). Set `SHOW_CAMERA_WINDOW=1` to also open a local `cv2.imshow`.

## Test it

```bash
pytest                       # full suite (≈60 tests incl. sim integration)
pytest robot_car/tests/test_planning.py -q        # one module
```

`tests/test_integration_sim.py` runs explore→save and navigate→reach-goal fully in the
simulator with live SLAM + safety threads.

## Architecture / layers

```
L1 Hardware   hardware/{motors,sensors,camera,gpio_cleanup}.py + backends/{real,sim}.py
L2 Core AI    core/{occupancy_grid,slam,localization,odometry,path_planner,
                     forbidden_zones,safety_monitor,simulator}.py
L3 Behaviours modes/{explore,navigate,idle}.py + controller.py (mode state machine)
L4 Interface  ui/server.py + ui/static/{index.html,map.js,ui.js}
Wiring        app.py (startup sequence) ← main.py (CLI)
Shared        config.py (all constants) · state.py (locked shared state) · context.py
```

Threads (`app.py` starts them): safety ~20 Hz · camera ~25 Hz · SLAM/localization
~10 Hz · mode worker (event-driven) · Flask-SocketIO (`async_mode='threading'`).
Shared state and locks live only in `state.py`; use its accessor helpers.

## Conventions (do not violate)

- **No magic numbers outside `config.py`.** Every threshold/pin/rate is a config constant.
- World coords are **metres**, `(x, y)` with +y up; angles are **radians**.
- Grid cells are `(col, row)`; numpy arrays are **row-major `grid[row, col]`** — never `[x, y]`.
- Sensor distances are **cm** (convert to metres immediately); `float('inf')` = no echo.
- Occupancy: internal **float32 log-odds**; published/saved as **uint8 0..100**
  (0 free, 50 unknown, 100 occupied). A* walkable = value `< 65` and not forbidden.
- Safety overrides everything: priority is **safety > camera advisory > SLAM > navigation**.

## Key decisions baked in (reconciled from the design docs)

- **Camera = USB / UVC** (`cv2.VideoCapture`), not the Pi CSI module — matches
  `camera_integration_design.md`; the other docs were reconciled to agree.
- **Motors = 4×AA 6 V** (TT motors max 6 V), not 7.4 V.
- **Target board = Raspberry Pi 2**, so CPU-bound rates are budgeted accordingly:
  camera 320×240 @ 10 Hz, safety 10 Hz. The GPIO pin map is unchanged (12/13 are
  hardware-PWM on every 40-pin Pi). The wiring notes' USB-C power remark is
  Pi-4-era — the Pi 2 powers over **Micro-USB**.
- **Inflation is applied at plan time** (`OccupancyGrid.planning_grid`) so scan matching
  still runs against true (un-inflated) walls; the saved `.map` holds the base grid.
- **Forbidden zones** are a separate boolean layer saved as `.zones` beside the `.map`.
- **VO** outputs a robot-frame `(forward, 0, dtheta)` delta from a partial-affine fit
  of the optical flow, read with a forward-camera model: yaw from the horizontal image
  shift (`CAMERA_FOV`), forward travel from the affine scale against the
  `VO_ASSUMED_DEPTH_M` depth prior (monocular VO has no absolute scale); the SLAM loop
  rotates it into the world frame. Confidence is the RANSAC inlier ratio. Weight 0.15,
  used mainly for slip detection.
- **The sim camera is a first-person raycaster** (Wolfenstein-style) over the world's
  wall segments — textured walls, distance-shaded floor/ceiling — so both camera
  pipelines run against geometrically correct imagery, not noise.
  `ROBOT_SIM_WORLD=bsp` (plus `ROBOT_SIM_SEED=n`) generates a multi-room floor plan by
  recursive division: cut a box in half, leave a one-door gap in every dividing wall
  (connectivity guaranteed), recurse; everything snaps to a door-width lattice offset
  half a cell so the (0,0) start pose stays clear.

## Build plan (followed; all phases complete)

A infra → B hardware+backends → C safety → D camera AI+debug → E mapping/SLAM/fusion
→ F planning(A*+forbidden) → G modes → H web → I orchestration → J tests/docs.
Each phase shipped with its own pytest module.

## Known limitations (by design — see `ai_cargo_robot_explanation.md` §15)

2D horizontal sensing in a 3D world; no SLAM loop closure; static map during navigation
(moving obstacles handled reactively only); single-floor; monocular VO has no depth;
single-channel LM393 encoders give no direction (inferred from the last motor command).

## Gotchas for future changes

- The simulator world (`core/simulator.py`) deliberately keeps the origin clear so the
  default start pose isn't inside an obstacle. If you add furniture, don't block `(0,0)`.
- `state.reset()` clears log listeners — tests rely on this to avoid cross-test leakage.
- `socketio.run(..., allow_unsafe_werkzeug=True)` is fine for the dev/Pi single-user case.
```
