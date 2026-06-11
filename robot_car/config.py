"""Central configuration for the AI Cargo Robot.

Every tunable value lives here -- GPIO pins, speed limits, sensor thresholds,
map resolution, timing constants, fusion weights and simulator parameters.
No magic numbers should appear anywhere else in the codebase (see feature_plan.md).

All distances are in **metres**, all angles in **radians**, unless a name ends in
``_CM`` / ``_DEG`` / ``_MS``. Grid cells are addressed as ``(col, row)`` and numpy
arrays are row-major ``grid[row, col]``.
"""

import math
import os

# ---------------------------------------------------------------------------
# Backend selection
# ---------------------------------------------------------------------------
# 'auto' detects a Raspberry Pi at runtime (see hardware/platform_detect.py).
# Force with the environment variable ROBOT_BACKEND=pi|sim.
BACKEND = os.environ.get("ROBOT_BACKEND", "auto")

# ---------------------------------------------------------------------------
# GPIO pin allocation (BCM numbering) -- see hardware_wiring_notes.md
# ---------------------------------------------------------------------------
# Motor driver (L298N)
PIN_ENA = 12        # left channel PWM  (hardware PWM)
PIN_IN1 = 5         # left direction A
PIN_IN2 = 6         # left direction B
PIN_ENB = 13        # right channel PWM (hardware PWM)
PIN_IN3 = 19        # right direction A
PIN_IN4 = 26        # right direction B

# Ultrasonic sensors -- (TRIG, ECHO)
SENSOR_PINS = {
    "front": (17, 27),
    "right": (22, 23),
    "back":  (24, 25),
    "left":  (10, 9),
    "down":  (11, 8),
}

# Wheel encoders (LM393)
PIN_ENCODER_LEFT = 20
PIN_ENCODER_RIGHT = 21

# ---------------------------------------------------------------------------
# Drive train geometry
# ---------------------------------------------------------------------------
WHEEL_DIAMETER = 0.065          # m (65 mm wheels)
WHEEL_CIRCUMFERENCE = math.pi * WHEEL_DIAMETER
ENCODER_SLOTS = 20              # slots on the encoder disk
DIST_PER_PULSE = WHEEL_CIRCUMFERENCE / ENCODER_SLOTS   # ~0.0102 m
WHEEL_BASE = 0.15               # m, distance between left/right wheel centres
ROBOT_RADIUS = 0.12             # m, half the chassis footprint (collision + clearance)

MAX_SPEED_MPS = 0.25            # m/s at full PWM (used by the simulator + nav)
PWM_FREQUENCY = 1000            # Hz for L298N enable pins

# Motion command speeds (fraction of full PWM, 0..1)
DRIVE_SPEED = 0.45
TURN_SPEED = 0.40
REVERSE_SPEED = 0.40

# ---------------------------------------------------------------------------
# Ultrasonic sensor parameters
# ---------------------------------------------------------------------------
SENSOR_TIMEOUT_MS = 30          # echo timeout -> reading is float('inf')
SENSOR_MAX_RANGE_CM = 400.0     # HC-SR04 practical max range
SPEED_OF_SOUND_CM_S = 34300.0   # cm/s for echo time -> distance

# --- Sensor scheduler (single bus owner; non-blocking cache) -- see P0-1 -----
SENSOR_PING_SPACING_S = 0.06    # cross-talk gap between pings (~16 pings/s max)
SENSOR_STALE_AGE_S = 0.5        # readings older than this are flagged stale

# Mount angle of each horizontal sensor relative to robot heading (radians).
# Robot +x is forward. Positive angle is counter-clockwise (left).
SENSOR_ANGLES = {
    "front": 0.0,
    "left": math.pi / 2,
    "back": math.pi,
    "right": -math.pi / 2,
}
SENSOR_MOUNT_HEIGHT_CM = 12.0   # horizontal sensors mounted ~12 cm up

# Mount position of each sensor in the robot frame, metres (dx forward, dy left).
# Sensors sit on the chassis perimeter, so readings are cast from here, not the pose;
# otherwise every wall maps ~10 cm too far out and scan matching corrects against that
# distorted map (P0-5). Applied identically in mapping, scan matching and the sim.
_SENSOR_MOUNT_R = 0.10
SENSOR_OFFSETS = {
    "front": (_SENSOR_MOUNT_R, 0.0),
    "back":  (-_SENSOR_MOUNT_R, 0.0),
    "left":  (0.0, _SENSOR_MOUNT_R),
    "right": (0.0, -_SENSOR_MOUNT_R),
    "down":  (_SENSOR_MOUNT_R, 0.0),   # cliff sensor is front-mounted (see P0-2)
}

# Beam model (P1-4). A no-echo usually means an absorbing/oblique surface, not 4 m of
# free space, so a no-echo carves free only this far (confident long free lines punch
# holes in walls A* then routes through).
SENSOR_INF_FREE_RANGE_M = 1.0
# HC-SR04 cone is ~15 deg wide, so a hit's endpoint is marked occupied across this fan.
SENSOR_BEAM_HALF_FAN = math.radians(7.5)

# ---------------------------------------------------------------------------
# Safety thresholds
# ---------------------------------------------------------------------------
SAFETY_HZ = 10.0                # monitor decision-loop rate (reads the sensor cache)
STOP_THRESHOLD_CM = 18.0        # obstacle ahead (direction of travel) -> stop
ADVISORY_TIGHTEN_CM = 8.0       # extra front margin when camera advisory is set
# Left/right/back only block when a collision is imminent. Distances are measured
# from the robot centre, and planned paths legitimately skirt walls at ~SIDE+ range;
# sharing the front threshold would freeze the robot beside every wall.
SIDE_STOP_THRESHOLD_CM = 14.0   # ROBOT_RADIUS (12 cm) + 2 cm gap
DROP_NORMAL_CM = 15.0           # expected downward reading on flat floor
DROP_FLOOR_GONE_CM = 23.0       # > this -> floor disappeared (cliff)
DROP_OBSTACLE_CM = 7.0          # < this -> obstacle directly below/front

SAFETY_REVERSE_TIME_S = 0.6     # how long to back away on frontal block
SAFETY_PIVOT_TIME_S = 0.5       # how long to pivot away on frontal block
# Recovery is stepped one safety cycle at a time, never a blocking sleep (P0-2), so
# durations are whole cycles at SAFETY_HZ.
SAFETY_REVERSE_CYCLES = max(1, round(SAFETY_REVERSE_TIME_S * SAFETY_HZ))
SAFETY_PIVOT_CYCLES = max(1, round(SAFETY_PIVOT_TIME_S * SAFETY_HZ))
SAFETY_BACK_GATE_CM = SIDE_STOP_THRESHOLD_CM  # rear reading that aborts a reverse

# --- Dead-man / thread supervision (P0-3) ---------------------------------
# Core loops stamp a heartbeat; a stale safety heartbeat makes the motor layer refuse
# to drive, and the watchdog surfaces any stalled thread to the status log.
SAFETY_DEADMAN_TIMEOUT_S = 0.5  # motors coast to stop if safety heartbeat older
WATCHDOG_HZ = 5.0
WATCHDOG_STALE_S = 1.0          # report a thread stalled past this heartbeat age

# ---------------------------------------------------------------------------
# Manual control override / teleop (F-21)
# ---------------------------------------------------------------------------
# Dead-man: the client re-emits the held stick at MANUAL_CMD_RATE_HZ, and the manual
# behaviour applies it only while fresher than the timeout, else stops.
MANUAL_CMD_TIMEOUT_S = 0.4
MANUAL_CMD_RATE_HZ = 10.0       # client repeat rate; also served to the JS
MANUAL_LINEAR_SPEED = DRIVE_SPEED   # wheel speed at full forward/back stick
MANUAL_TURN_SPEED = TURN_SPEED      # wheel speed at full rotate stick

# ---------------------------------------------------------------------------
# Occupancy grid / mapping
# ---------------------------------------------------------------------------
MAP_RESOLUTION = 0.05           # m per cell (5 cm dev, 3 cm for deployment)
MAP_WIDTH_CELLS = 200           # 200 * 0.05 = 10 m wide
MAP_HEIGHT_CELLS = 200
# World origin (0, 0) maps to the centre cell.
MAP_ORIGIN_COL = MAP_WIDTH_CELLS // 2
MAP_ORIGIN_ROW = MAP_HEIGHT_CELLS // 2

GRID_FREE = 0
GRID_UNKNOWN = 50
GRID_OCCUPIED = 100

# Log-odds update parameters
LOG_ODDS_CLAMP = 10.0
P_FREE = 0.30                   # evidence for a cell along the beam
P_OCCUPIED = 0.70               # evidence for the beam endpoint cell

# Obstacle inflation must cover the robot radius plus a margin, or A* plans paths
# the chassis cannot actually fit through.
INFLATION_MARGIN_M = 0.03
INFLATION_ITERATIONS = math.ceil((ROBOT_RADIUS + INFLATION_MARGIN_M) / MAP_RESOLUTION)
INFLATION_OCCUPIED_THRESHOLD = 65   # cells above this are "confirmed occupied"

# ---------------------------------------------------------------------------
# Path planning (A*)
# ---------------------------------------------------------------------------
WALKABLE_THRESHOLD = 65         # cell is traversable when value < this
DIAGONAL_COST = math.sqrt(2)
ARRIVAL_THRESHOLD = 0.09        # m (~2 cells) waypoint arrival radius
HEADING_TOLERANCE = math.radians(12)   # rotate-in-place until within this
REPLAN_RETRY_LIMIT = 3
REPLAN_WAIT_S = 0.5
CONTROL_HZ = 20.0               # route-execution control loop rate
STEER_GAIN = 0.6                # proportional heading correction while driving
# Stall watchdog: abandon a goal if the robot fails to get measurably closer to
# it for this long (a goal it can physically never reach, or pose-jitter orbiting
# the target). Prevents the navigator looping forever "stuck on nothing".
NAV_PROGRESS_TIMEOUT_S = 8.0
NAV_PROGRESS_EPSILON = 0.05     # m of goal-distance reduction that counts as progress
# Off-path recovery. The executed path is string-pulled into long straight segments,
# so after a safety recovery maneuver or a SLAM pose correction the straight line from
# the *current* pose to the next waypoint may cut through a wall. When line of sight
# to the waypoint is lost, replan instead of driving blind.
NAV_REPLAN_MIN_INTERVAL_S = 1.0   # throttle for deviation-triggered replans
# If the fused pose lands inside an inflated obstacle or a forbidden zone, A* would
# refuse to plan (start unwalkable). Instead, plan from the nearest walkable cell
# within this radius and drive out through it.
NAV_ESCAPE_RADIUS_CELLS = 12      # 12 * 0.05 = 0.6 m search radius

# ---------------------------------------------------------------------------
# SLAM / scan matching
# ---------------------------------------------------------------------------
SLAM_HZ = 10.0
SCAN_MATCH_RANGE = 0.15         # +/- search window (m)
SCAN_MATCH_ANGLE = math.radians(10)
SCAN_MATCH_STEP = 0.05          # m step inside the window
SCAN_MATCH_ANGLE_STEP = math.radians(5)
# Likelihood-field scan matching (P0-4): each endpoint scores a Gaussian of its
# distance to the nearest mapped obstacle -- continuous, penalises near misses, and
# flat (no correction) where the map is featureless.
SCAN_MATCH_SIGMA_M = 0.10       # Gaussian width of the likelihood field (m)
SCAN_MATCH_MIN_GAIN = 0.05      # min mean-likelihood improvement to accept a correction
SCAN_MATCH_DAMPING = 0.5        # fraction of an accepted correction actually applied
SCAN_MATCH_FIELD_REFRESH_S = 1.0  # recompute the distance transform at most this often

FRONTIER_FREE_THRESHOLD = 35    # value < this counts as known-free for frontiers
FRONTIER_MIN_CLUSTER = 3        # ignore frontier blobs smaller than this many cells

# Motion-distortion guard for mapping. A scan is ray-cast as a single instantaneous
# snapshot, but the SLAM loop runs at ~10 Hz while the robot can turn ~7 deg/cycle
# in place -- far endpoints then sweep into phantom arcs ("lines") of fake obstacle.
# Skip integrating a scan when the pose moved/rotated more than this since the last
# cycle, so mapping only happens when the snapshot is geometrically trustworthy.
MAP_MAX_ROTATION_PER_SCAN = math.radians(4.0)
MAP_MAX_TRANSLATION_PER_SCAN = 0.20   # m of per-cycle pose change before skipping

# ---------------------------------------------------------------------------
# Localization fusion
# ---------------------------------------------------------------------------
WEIGHT_ENCODER = 0.55
WEIGHT_SCAN = 0.30
WEIGHT_VISUAL = 0.15
SLIP_THRESHOLD = 0.02           # m of commanded motion to test for slip
SLIP_ENCODER_PENALTY = 0.5      # encoder confidence multiplier on detected slip

# ---------------------------------------------------------------------------
# Camera -- sized for the Raspberry Pi 2 CPU budget (VO + Canny on every frame;
# 640x480 @ 25 Hz is a Pi 4 workload).
# ---------------------------------------------------------------------------
CAMERA_INDEX = 0
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
CAMERA_FPS = 15
CAMERA_HZ = 10.0                # processing loop target
# Horizontal field of view of the forward-facing camera. Used by the simulator's
# first-person renderer and by VO to convert horizontal image flow into yaw.
CAMERA_FOV = math.radians(60)

# Visual odometry (Shi-Tomasi + Lucas-Kanade)
VO_MAX_CORNERS = 150
VO_QUALITY_LEVEL = 0.01
VO_MIN_DISTANCE = 10
VO_BLOCK_SIZE = 7
VO_LK_WIN = (21, 21)
VO_LK_MAX_LEVEL = 3
VO_REDETECT_THRESHOLD = 30      # re-detect features when good count drops below
VO_MIN_TRACKED = 10             # below this, confidence is 0
# Monocular forward VO has no depth, so the affine scale factor between frames is
# converted to metres of forward travel against this assumed scene depth (distance
# to the dominant tracked surface). A calibration constant on real hardware; in the
# simulator it matches the front-wall distance of the test worlds.
VO_ASSUMED_DEPTH_M = 2.0

# Appearance-based obstacle detection
APPEARANCE_ROI_TOP = 0.55       # analyse from 55% down to the bottom of frame
APPEARANCE_BLUR = (5, 5)
APPEARANCE_CANNY_LOW = 50
APPEARANCE_CANNY_HIGH = 150
APPEARANCE_MIN_CONTOUR_AREA = 375   # px^2 at 320x240 (scale with frame area)

# Debug view
SHOW_CAMERA_WINDOW = os.environ.get("SHOW_CAMERA_WINDOW", "0") == "1"

# ---------------------------------------------------------------------------
# Web server
# ---------------------------------------------------------------------------
WEB_HOST = "0.0.0.0"
WEB_PORT = int(os.environ.get("ROBOT_PORT", "5000"))

# ---------------------------------------------------------------------------
# File paths
# ---------------------------------------------------------------------------
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
MAPS_DIR = os.path.join(_THIS_DIR, "maps")
MAP_EXTENSION = ".map"
ZONES_EXTENSION = ".zones"

# ---------------------------------------------------------------------------
# Simulator parameters (used only when the sim backend is active)
# ---------------------------------------------------------------------------
SIM_WORLD = os.environ.get("ROBOT_SIM_WORLD", "room")  # built-in world name or 'bsp'
SIM_TICK_HZ = 100.0             # physics integration rate
SIM_SENSOR_NOISE_CM = 0.8       # Gaussian noise stddev on ultrasonic readings
SIM_ENCODER_SLIP = 0.02         # fractional random wheel slip
SIM_START_POSE = (0.0, 0.0, 0.0)   # (x, y, theta)

# Generated 'bsp' world: recursive division of a square into rooms, one door per
# dividing wall. Positions snap to a lattice of one door width, with the world
# shifted half a cell so no wall line passes through the start pose at the origin.
SIM_BSP_SEED = int(os.environ.get("ROBOT_SIM_SEED", "42"))
SIM_BSP_SIZE = 7.0              # m, side of the square floor plan (map is 10 m)
SIM_BSP_DOOR = 0.7              # m, door width == lattice pitch
SIM_BSP_MIN_ROOM = 1.4          # m, chambers narrower than this are not split

# First-person camera renderer (raycaster over the world's wall segments).
SIM_WALL_HALF_HEIGHT = 0.25     # m, wall half-height above/below the camera axis
SIM_SHADE_FALLOFF = 0.35        # brightness = 1 / (1 + distance * falloff)
SIM_WALL_TEX_PPM = 200.0        # texture pixels per metre along a wall
SIM_FLOOR_GRAY = 130            # untextured floor/ceiling base brightness
SIM_CEIL_GRAY = 105
