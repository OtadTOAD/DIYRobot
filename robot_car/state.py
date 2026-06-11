"""Shared state for all threads.

Single source of truth for cross-thread data. Every variable here is protected
by its companion lock; always acquire the lock before reading or writing. Helper
accessors are provided so callers never have to remember which lock guards which
variable.

Locking discipline (see camera_integration_design.md section 5):
    robot_pose       RLock  -- written by localization, read by nav + web
    occupancy_grid   RLock  -- written by SLAM, read by nav + web
    blocked          Lock   -- written by safety, read by navigation
    vo_estimate      Lock   -- accumulated by camera, consumed by SLAM
    camera_advisory  Lock   -- written by camera, read by safety
    current_mode     Lock   -- written by mode controller, read everywhere
"""

import threading
import time

import numpy as np

# --- Localization -----------------------------------------------------------
robot_pose: tuple = (0.0, 0.0, 0.0)        # (x, y, theta) world metres / radians
pose_lock = threading.RLock()

# --- Mapping ----------------------------------------------------------------
occupancy_grid: "np.ndarray | None" = None  # published uint8 grid (0..100)
grid_lock = threading.RLock()

# --- Safety -----------------------------------------------------------------
blocked: bool = False                       # an obstacle is blocking motion
blocked_lock = threading.Lock()

drop_latched: bool = False                  # cliff detected, needs manual ack
drop_lock = threading.Lock()

# --- Camera -> localization -------------------------------------------------
vo_estimate: tuple = (0.0, 0.0, 0.0)        # (dx, dy, dtheta) accumulated since
vo_confidence: float = 0.0                  # the last consume_vo() (min over window)
vo_samples: int = 0                         # frames accumulated since consume
vo_lock = threading.Lock()

# --- Camera -> safety -------------------------------------------------------
camera_advisory: bool = False               # soft obstacle hint from camera
advisory_lock = threading.Lock()

# --- Camera frames (for the debug view) -------------------------------------
latest_frame: "np.ndarray | None" = None    # raw BGR frame
debug_frame: "np.ndarray | None" = None     # annotated BGR frame for /camera/debug
frame_lock = threading.Lock()

# --- Mode -------------------------------------------------------------------
current_mode: str = "idle"                  # 'idle' | 'explore' | 'navigate' | 'manual'
mode_lock = threading.Lock()

# --- Manual teleop command (F-21) -------------------------------------------
# (linear -1..1, angular -1..1, monotonic timestamp). Written by the web WS handler,
# read by the manual behaviour, which ignores it once older than MANUAL_CMD_TIMEOUT_S.
# The initial timestamp 0.0 reads as ancient, so motors stay stopped until commanded.
manual_cmd: tuple = (0.0, 0.0, 0.0)
manual_lock = threading.Lock()

# --- Global lifecycle -------------------------------------------------------
stop_event = threading.Event()              # set on shutdown; threads should exit

# --- Thread heartbeats (dead-man supervision, P0-3) -------------------------
# Each core loop calls beat(name) once per cycle; a stale heartbeat means the thread
# has died or hung. The motor layer gates on the 'safety' heartbeat; a watchdog
# surfaces any stalled thread. A name with no recorded beat is treated as "not
# running" (None age), so unit tests that never start the threads are not gated.
_heartbeats: dict = {}
_heartbeat_lock = threading.Lock()

# --- Status log (decoupled pub/sub) -----------------------------------------
# Any thread may publish a status message via set_log(); the web server registers
# a listener that forwards them as 'status_log' WebSocket events. This keeps core
# modules free of any Flask/SocketIO dependency.
_log_listeners = []
_log_lock = threading.Lock()


# ---------------------------------------------------------------------------
# Accessor helpers -- keep locking consistent across the codebase
# ---------------------------------------------------------------------------
def get_pose() -> tuple:
    with pose_lock:
        return robot_pose


def set_pose(pose: tuple) -> None:
    global robot_pose
    with pose_lock:
        robot_pose = (float(pose[0]), float(pose[1]), float(pose[2]))


def get_grid() -> "np.ndarray | None":
    with grid_lock:
        return None if occupancy_grid is None else occupancy_grid.copy()


def set_grid(grid: np.ndarray) -> None:
    global occupancy_grid
    with grid_lock:
        occupancy_grid = grid


def is_blocked() -> bool:
    with blocked_lock:
        return blocked


def set_blocked(value: bool) -> None:
    global blocked
    with blocked_lock:
        blocked = bool(value)


def is_drop_latched() -> bool:
    with drop_lock:
        return drop_latched


def set_drop_latched(value: bool) -> None:
    global drop_latched
    with drop_lock:
        drop_latched = bool(value)


def add_vo(delta: tuple, confidence: float) -> None:
    """Accumulate one per-frame VO delta (camera runs faster than SLAM).

    Confidence is the *minimum* over the window, not the latest frame's: one lucky
    frame must not launder a delta accumulated over several garbage ones (P1-5).
    """
    global vo_estimate, vo_confidence, vo_samples
    with vo_lock:
        vo_estimate = (vo_estimate[0] + delta[0],
                       vo_estimate[1] + delta[1],
                       vo_estimate[2] + delta[2])
        vo_confidence = (float(confidence) if vo_samples == 0
                         else min(vo_confidence, float(confidence)))
        vo_samples += 1


def consume_vo() -> tuple:
    """Return ((dx, dy, dtheta), confidence) accumulated since the last call, then
    reset -- a delta is never applied twice and a stalled camera decays to zero."""
    global vo_estimate, vo_confidence, vo_samples
    with vo_lock:
        out = (vo_estimate, vo_confidence if vo_samples else 0.0)
        vo_estimate = (0.0, 0.0, 0.0)
        vo_confidence = 0.0
        vo_samples = 0
        return out


def beat(name: str) -> None:
    """Record a heartbeat for a core thread (called once per loop cycle)."""
    with _heartbeat_lock:
        _heartbeats[name] = time.monotonic()


def heartbeat_age(name: str):
    """Seconds since the thread last beat, or ``None`` if it never has."""
    with _heartbeat_lock:
        ts = _heartbeats.get(name)
    return None if ts is None else max(0.0, time.monotonic() - ts)


def heartbeats() -> dict:
    """Snapshot of ``{name: age_seconds}`` for every thread that has beat."""
    now = time.monotonic()
    with _heartbeat_lock:
        return {name: max(0.0, now - ts) for name, ts in _heartbeats.items()}


def clear_heartbeats() -> None:
    with _heartbeat_lock:
        _heartbeats.clear()


def get_advisory() -> bool:
    with advisory_lock:
        return camera_advisory


def set_advisory(value: bool) -> None:
    global camera_advisory
    with advisory_lock:
        camera_advisory = bool(value)


def get_mode() -> str:
    with mode_lock:
        return current_mode


def set_mode(mode: str) -> None:
    global current_mode
    with mode_lock:
        current_mode = mode


def set_manual_cmd(linear: float, angular: float) -> None:
    """Store the latest teleop command, stamped now (F-21)."""
    global manual_cmd
    linear = max(-1.0, min(1.0, float(linear)))
    angular = max(-1.0, min(1.0, float(angular)))
    with manual_lock:
        manual_cmd = (linear, angular, time.monotonic())


def get_manual_cmd() -> tuple:
    with manual_lock:
        return manual_cmd


def reset_manual_cmd() -> None:
    """Forget any held command (entering manual mode mustn't latch a stale stick)."""
    global manual_cmd
    with manual_lock:
        manual_cmd = (0.0, 0.0, 0.0)


def set_latest_frame(frame) -> None:
    global latest_frame
    with frame_lock:
        latest_frame = frame


def get_latest_frame():
    with frame_lock:
        return None if latest_frame is None else latest_frame.copy()


def set_debug_frame(frame) -> None:
    global debug_frame
    with frame_lock:
        debug_frame = frame


def get_debug_frame():
    with frame_lock:
        return None if debug_frame is None else debug_frame.copy()


def add_log_listener(listener) -> None:
    """Register ``listener(level, msg)`` to receive status-log messages."""
    with _log_lock:
        _log_listeners.append(listener)


def set_log(level: str, msg: str) -> None:
    """Publish a status message to all listeners. Levels: ok|warn|info|error."""
    with _log_lock:
        listeners = list(_log_listeners)
    for listener in listeners:
        try:
            listener(level, msg)
        except Exception:
            pass


def clear_log_listeners() -> None:
    with _log_lock:
        _log_listeners.clear()


def reset() -> None:
    """Reset all shared state -- used by the test suite between cases."""
    global robot_pose, occupancy_grid, blocked, drop_latched
    global vo_estimate, vo_confidence, vo_samples, camera_advisory
    global latest_frame, debug_frame, current_mode, manual_cmd
    with pose_lock:
        robot_pose = (0.0, 0.0, 0.0)
    with grid_lock:
        occupancy_grid = None
    with blocked_lock:
        blocked = False
    with drop_lock:
        drop_latched = False
    with vo_lock:
        vo_estimate = (0.0, 0.0, 0.0)
        vo_confidence = 0.0
        vo_samples = 0
    with advisory_lock:
        camera_advisory = False
    with frame_lock:
        latest_frame = None
        debug_frame = None
    with mode_lock:
        current_mode = "idle"
    with manual_lock:
        manual_cmd = (0.0, 0.0, 0.0)
    clear_log_listeners()
    clear_heartbeats()
    stop_event.clear()
