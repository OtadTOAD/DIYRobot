"""Motor control -- public interface from feature_plan.md.

``set_speed(left, right)`` / ``stop()`` are the only entry points the rest of the
codebase uses.

The single-channel LM393 encoders cannot sense direction, so each wheel's pulses
are signed by the motor command that was active while they were produced. Pulses
are banked here on every command change; if they were instead signed by whatever
command odometry sees at read time, every reverse/pivot-to-stop transition would
misattribute up to a full read window of travel (a pivot window read as "both
wheels forward" injects several degrees of heading error).
"""

import threading

from robot_car.hardware import hal

_last_command = (0.0, 0.0)
_banked_pulses = [0.0, 0.0]      # signed pulses accumulated under past commands
_cmd_lock = threading.Lock()


def _sign(value: float) -> float:
    if value > 1e-6:
        return 1.0
    if value < -1e-6:
        return -1.0
    return 1.0  # coasting with no command -> assume forward


def _bank_pulses_locked() -> None:
    left, right = hal.get_backend().read_encoder_pulses()
    _banked_pulses[0] += _sign(_last_command[0]) * left
    _banked_pulses[1] += _sign(_last_command[1]) * right


def _command(left: float, right: float) -> None:
    global _last_command
    with _cmd_lock:
        _bank_pulses_locked()
        _last_command = (left, right)


def set_speed(left: float, right: float) -> None:
    """Set motor speeds. Range -1.0..1.0 (negative = reverse, 0 = stop)."""
    left = max(-1.0, min(1.0, float(left)))
    right = max(-1.0, min(1.0, float(right)))
    _command(left, right)
    hal.get_backend().motor_set(left, right)


def stop() -> None:
    """Stop all motors immediately."""
    _command(0.0, 0.0)
    hal.get_backend().motor_stop()


def get_last_command() -> tuple:
    """Return the last (left, right) speed command."""
    with _cmd_lock:
        return _last_command


def reset() -> None:
    """Clear the command and banked pulses -- used by the test suite."""
    global _last_command
    with _cmd_lock:
        _last_command = (0.0, 0.0)
        _banked_pulses[0] = _banked_pulses[1] = 0.0


def consume_signed_pulses() -> tuple:
    """Return signed (left, right) pulse counts since the last call, then reset."""
    with _cmd_lock:
        _bank_pulses_locked()
        out = (_banked_pulses[0], _banked_pulses[1])
        _banked_pulses[0] = _banked_pulses[1] = 0.0
        return out
