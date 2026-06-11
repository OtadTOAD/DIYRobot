"""Manual control override / teleop (F-21).

A fourth behaviour run by the same mode worker as idle/explore/navigate, so only one
thread ever drives the motors. Command flow is a dead-man: the browser re-emits the
held stick at ``MANUAL_CMD_RATE_HZ`` and this loop applies it only while fresher than
``MANUAL_CMD_TIMEOUT_S``, otherwise stopping. Safety still decides and latches but does
not own the motors here (see ``SafetyMonitor._owns_motors``); instead this loop gates
directionally -- a front block refuses forward but still allows reverse and rotation --
which is exactly the "drive it out of the stuck spot" case. A latched cliff freezes
all motion until acknowledged.
"""

from __future__ import annotations

import time

from robot_car import config, state
from robot_car.hardware import motors, sensors


def _clamp(v):
    return max(-1.0, min(1.0, v))


def mix_drive(linear: float, angular: float) -> tuple:
    """Mix a (linear, angular) stick into clamped (left, right) wheel speeds.
    ``angular > 0`` turns counter-clockwise, matching the navigator's convention."""
    drive = _clamp(linear) * config.MANUAL_LINEAR_SPEED
    turn = _clamp(angular) * config.MANUAL_TURN_SPEED
    return _clamp(drive - turn), _clamp(drive + turn)


def gate_direction(linear: float, front_cm: float, back_cm: float) -> float:
    """Zero a forward command into a blocked front, or a reverse into a blocked back;
    rotation is always allowed."""
    if linear > 0 and front_cm < config.STOP_THRESHOLD_CM:
        return 0.0
    if linear < 0 and back_cm < config.SIDE_STOP_THRESHOLD_CM:
        return 0.0
    return linear


class Manual:
    def __init__(self, context):
        self.ctx = context

    def run(self, stop_event) -> None:
        motors.stop()
        state.reset_manual_cmd()                 # don't latch a stale stick on entry
        state.set_log("info", "Manual control -- drive with the D-pad or WASD")
        period = 1.0 / config.CONTROL_HZ
        while not stop_event.is_set() and not state.stop_event.is_set():
            self.step()
            stop_event.wait(period)
        motors.stop()

    def step(self) -> None:
        """One teleop control cycle (extracted for testing)."""
        linear, angular, ts = state.get_manual_cmd()
        stale = time.monotonic() - ts > config.MANUAL_CMD_TIMEOUT_S
        if state.is_drop_latched() or stale:
            motors.stop()
            return

        readings = sensors.get_all_readings()
        linear = gate_direction(linear, readings["front"].distance_cm,
                                readings["back"].distance_cm)
        left, right = mix_drive(linear, angular)
        if left == 0.0 and right == 0.0:
            motors.stop()
        else:
            motors.set_speed(left, right)
