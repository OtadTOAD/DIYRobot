"""Motor control -- public interface from feature_plan.md.

``set_speed(left, right)`` / ``stop()`` are the only entry points the rest of the
codebase uses. The last commanded value is recorded so odometry can infer wheel
direction (the single-channel LM393 encoders cannot sense it themselves).
"""

import threading

from robot_car.hardware import hal

_last_command = (0.0, 0.0)
_cmd_lock = threading.Lock()


def set_speed(left: float, right: float) -> None:
    """Set motor speeds. Range -1.0..1.0 (negative = reverse, 0 = stop)."""
    left = max(-1.0, min(1.0, float(left)))
    right = max(-1.0, min(1.0, float(right)))
    global _last_command
    with _cmd_lock:
        _last_command = (left, right)
    hal.get_backend().motor_set(left, right)


def stop() -> None:
    """Stop all motors immediately."""
    global _last_command
    with _cmd_lock:
        _last_command = (0.0, 0.0)
    hal.get_backend().motor_stop()


def get_last_command() -> tuple:
    """Return the last (left, right) speed command -- used for encoder direction."""
    with _cmd_lock:
        return _last_command
