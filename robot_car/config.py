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

# Mount angle of each horizontal sensor relative to robot heading (radians).
# Robot +x is forward. Positive angle is counter-clockwise (left).
SENSOR_ANGLES = {
    "front": 0.0,
    "left": math.pi / 2,
    "back": math.pi,
    "right": -math.pi / 2,
}
SENSOR_MOUNT_HEIGHT_CM = 12.0   # horizontal sensors mounted ~12 cm up

# ---------------------------------------------------------------------------
# Safety thresholds
# ---------------------------------------------------------------------------
STOP_THRESHOLD_CM = 18.0        # horizontal obstacle -> emergency stop
ADVISORY_TIGHTEN_CM = 8.0       # extra margin added when camera advisory is set
DROP_NORMAL_CM = 15.0           # expected downward reading on flat floor
DROP_FLOOR_GONE_CM = 23.0       # > this -> floor disappeared (cliff)
DROP_OBSTACLE_CM = 7.0          # < this -> obstacle directly below/front

SAFETY_REVERSE_TIME_S = 0.6     # how long to back away on frontal block
SAFETY_PIVOT_TIME_S = 0.5       # how long to pivot away on frontal block

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

# Obstacle inflation: 2 cells * 5 cm = 10 cm safety margin
INFLATION_ITERATIONS = 2
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

# ---------------------------------------------------------------------------
# SLAM / scan matching
# ---------------------------------------------------------------------------
SLAM_HZ = 10.0
SCAN_MATCH_RANGE = 0.15         # +/- search window (m)
SCAN_MATCH_ANGLE = math.radians(10)
SCAN_MATCH_STEP = 0.05          # m step inside the window
SCAN_MATCH_ANGLE_STEP = math.radians(5)
SCAN_MATCH_THRESHOLD = 0.35     # min normalised score to accept a correction

FRONTIER_FREE_THRESHOLD = 35    # value < this counts as known-free for frontiers
FRONTIER_MIN_CLUSTER = 3        # ignore frontier blobs smaller than this many cells

# ---------------------------------------------------------------------------
# Localization fusion
# ---------------------------------------------------------------------------
WEIGHT_ENCODER = 0.55
WEIGHT_SCAN = 0.30
WEIGHT_VISUAL = 0.15
SLIP_THRESHOLD = 0.02           # m of commanded motion to test for slip
SLIP_ENCODER_PENALTY = 0.5      # encoder confidence multiplier on detected slip

# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------
CAMERA_INDEX = 0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30
CAMERA_HZ = 25.0                # processing loop target

# Visual odometry (Shi-Tomasi + Lucas-Kanade)
VO_MAX_CORNERS = 150
VO_QUALITY_LEVEL = 0.01
VO_MIN_DISTANCE = 10
VO_BLOCK_SIZE = 7
VO_LK_WIN = (21, 21)
VO_LK_MAX_LEVEL = 3
VO_REDETECT_THRESHOLD = 30      # re-detect features when good count drops below
VO_MIN_TRACKED = 10             # below this, confidence is 0
# Pixels -> metres scale for translational flow. In the simulator this matches the
# synthetic top-down camera's render scale so VO recovers true motion; on real
# hardware it is a calibration constant for the forward-facing USB camera.
VO_PIXELS_PER_METRE = 300.0

# Appearance-based obstacle detection
APPEARANCE_ROI_TOP = 0.55       # analyse from 55% down to the bottom of frame
APPEARANCE_BLUR = (5, 5)
APPEARANCE_CANNY_LOW = 50
APPEARANCE_CANNY_HIGH = 150
APPEARANCE_MIN_CONTOUR_AREA = 1500

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
SIM_WORLD = os.environ.get("ROBOT_SIM_WORLD", "room")  # built-in world name
SIM_TICK_HZ = 100.0             # physics integration rate
SIM_SENSOR_NOISE_CM = 0.8       # Gaussian noise stddev on ultrasonic readings
SIM_ENCODER_SLIP = 0.02         # fractional random wheel slip
SIM_START_POSE = (0.0, 0.0, 0.0)   # (x, y, theta)
