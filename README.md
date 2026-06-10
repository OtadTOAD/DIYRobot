# AI Cargo Robot 🤖

A small autonomous indoor cargo robot for the **Raspberry Pi 2** (docs mention 4B; any 40-pin Pi works) — built on classical
AI, no ROS and no neural networks. It explores and maps an unknown room, localizes
itself, and drives to user-selected destinations while avoiding obstacles and floor
drops. A web UI shows the live map and lets you click-to-navigate.

It runs **identically on a laptop**: when it doesn't detect a Pi, it transparently
swaps in a 2D simulator backend, so you can run, demo and test the whole stack with no
hardware.

| | |
|---|---|
| Platform | Raspberry Pi 2 (or any Linux machine, via simulator) |
| Language | Python 3 |
| Navigation | Occupancy-grid SLAM + A* path planning |
| Localization | Encoder dead-reckoning + scan matching + visual odometry (weighted fusion) |
| Drive | Differential drive (L298N + 4× TT motors) |
| Sensors | 5× HC-SR04 ultrasonic + USB camera |
| UI | Flask + SocketIO, HTML5 canvas |

## Quick start (laptop / simulator)

```bash
python -m venv .venv && source .venv/bin/activate
pip install -r robot_car/requirements.txt

python -m robot_car.main            # then open http://localhost:5000
```

In the browser: click **Explore** to watch the map build, **Save** it, then switch to
**Navigate** and click anywhere on the map to send the robot there. The **Camera Debug**
panel streams the annotated camera frames (tracked optical-flow features + detected
obstacle contours).

CLI options:

```bash
python -m robot_car.main --explore                      # start mapping immediately
python -m robot_car.main --map living_room              # load a saved map, then idle
python -m robot_car.main --map living_room --waypoint desk   # load + drive to a waypoint
```

Force the backend with `ROBOT_BACKEND=sim` or `ROBOT_BACKEND=pi` (default: auto-detect).

## Running on the Raspberry Pi

1. Wire the hardware per [`hardware_wiring_notes.md`](hardware_wiring_notes.md)
   (note the **mandatory** HC-SR04 ECHO voltage dividers and the **separate** motor /
   Pi power supplies).
2. Install deps and start the pigpio daemon:
   ```bash
   pip install -r robot_car/requirements-pi.txt
   sudo pigpiod
   python -m robot_car.main
   ```
3. Open `http://<pi-ip>:5000` from any device on the same network.

## Tests

```bash
pytest          # ~60 unit + integration tests, all in the simulator
```

## Project layout

```
robot_car/
  hardware/   HAL + real (pigpio) and simulator backends
  core/       occupancy grid, SLAM, localization, A*, safety, odometry, simulator
  modes/      explore / navigate / idle behaviours
  ui/         Flask + SocketIO server and the web front-end
  config.py   every tunable constant      state.py  locked shared state
  app.py      startup wiring              main.py   CLI entry point
  tests/      pytest suite
```

## How it works (the short version)

- **Mapping** — each ultrasonic reading is ray-cast into a probabilistic occupancy
  grid with a log-odds Bayesian update; obstacles are inflated by ~10 cm before planning.
- **Localization** — encoder dead-reckoning is corrected by scan matching against the
  map and by camera visual odometry, fused with confidence-adaptive weights.
- **Exploration** — frontier-based: drive to the nearest boundary between known-free and
  unknown space, repeat until the room is fully mapped.
- **Navigation** — A* (8-directional, Euclidean heuristic) on the inflated grid, then
  visibility-based path smoothing; the route is executed with rotate-then-drive control
  and replans when blocked.
- **Safety** — an independent 20 Hz thread that can stop the motors and override
  navigation at any time (obstacles, cliffs).

Design details are in the `*_design.md` files; architecture and conventions for
contributors are in [`CLAUDE.md`](CLAUDE.md).

## Limitations

2D horizontal sensing, no SLAM loop closure, static map during navigation (moving
obstacles handled reactively), single floor, monocular camera (no depth). See
[`ai_cargo_robot_explanation.md`](ai_cargo_robot_explanation.md) §15.
