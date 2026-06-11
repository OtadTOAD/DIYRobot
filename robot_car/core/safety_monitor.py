"""Reactive safety layer (F-05) -- the highest-priority behaviour.

An independent ~10 Hz daemon whose decisions override navigation: it stops the motors
and raises ``state.blocked`` directly, so a planner bug can never disable collision
avoidance. The decision logic is the pure, unit-tested :func:`evaluate`.

The loop never blocks. Each cycle it reads the sensor cache, decides, and steps a
recovery state machine by one cycle (P0-2) -- the old blocking reverse/pivot sleeps
stopped the one thread whose job is reacting, reversing blind into what was behind it.
Now monitoring continues through every maneuver, the reverse phase is back-sensor
gated, and a cliff mid-maneuver pre-empts immediately. In manual mode the monitor
still decides and latches but does not own the motors -- teleop drives and gates
direction itself (F-21), so automatic recovery is suppressed.
"""

import threading
import time

from robot_car import config, state
from robot_car.hardware import motors, sensors

# Decision constants.
CLEAR = "clear"
LATCHED = "latched"
DROP_CLIFF = "drop_cliff"
DROP_OBSTACLE = "drop_obstacle"
BLOCK_FRONT = "block_front"
BLOCK_SIDE = "block_side"


def evaluate(distances: dict, advisory: bool, latched: bool) -> str:
    """Return the safety decision for a set of sensor distances (cm)."""
    if latched:
        return LATCHED

    down = distances.get("down", float("inf"))
    if down > config.DROP_FLOOR_GONE_CM:
        return DROP_CLIFF
    if down < config.DROP_OBSTACLE_CM:
        return DROP_OBSTACLE

    front_threshold = config.STOP_THRESHOLD_CM
    if advisory:
        front_threshold += config.ADVISORY_TIGHTEN_CM

    if distances.get("front", float("inf")) < front_threshold:
        return BLOCK_FRONT
    sides = min(
        distances.get("left", float("inf")),
        distances.get("right", float("inf")),
        distances.get("back", float("inf")),
    )
    if sides < config.SIDE_STOP_THRESHOLD_CM:
        return BLOCK_SIDE
    return CLEAR


class SafetyMonitor(threading.Thread):
    def __init__(self, hz: float = config.SAFETY_HZ):
        super().__init__(name="safety", daemon=True)
        self.period = 1.0 / hz
        # Pending recovery phases: a list of (kind, direction, cycles_left) tuples,
        # consumed one cycle per loop. Empty means "not recovering".
        self._recovery: list = []

    def run(self) -> None:
        while not state.stop_event.is_set():
            state.beat("safety")
            distances = sensors.get_all_distances()
            decision = evaluate(distances, state.get_advisory(), state.is_drop_latched())
            self.step(decision, distances)
            state.stop_event.wait(self.period)

    @staticmethod
    def _owns_motors() -> bool:
        """The monitor drives the motors except in manual mode (F-21), where the
        teleop behaviour is the sole driver and gates direction itself."""
        return state.get_mode() != "manual"

    # -- per-cycle decision + stepped recovery -------------------------------
    def step(self, decision: str, distances: dict | None = None) -> None:
        """Handle one cycle: a cliff/latch always wins; otherwise advance any active
        recovery, else act on the decision. Never blocks."""
        distances = distances or {}
        owns = self._owns_motors()

        # A cliff or an existing latch pre-empts everything, including a maneuver.
        if decision == LATCHED:
            self._recovery = []
            if owns:
                motors.stop()
            return
        if decision == DROP_CLIFF:
            self._recovery = []
            if owns:
                motors.stop()
            state.set_drop_latched(True)            # needs manual ack to clear
            state.set_blocked(True)
            state.set_log("error",
                          "Cliff detected -- emergency stop. Acknowledge to resume.")
            return

        # Mid-recovery: keep stepping it (monitoring continues every cycle).
        if self._recovery and owns:
            if self._step_recovery(distances):
                state.set_blocked(True)
            else:
                state.set_blocked(False)            # maneuver finished -- let nav retry
            return

        if decision == CLEAR:
            state.set_blocked(False)
            return

        # A new hazard: stop and block. In manual mode we stop here and let the
        # operator drive out (no automatic recovery to fight them).
        if owns:
            motors.stop()
        state.set_blocked(True)
        if not owns:
            return
        if decision == DROP_OBSTACLE:
            self._recovery = [("reverse", 0, config.SAFETY_REVERSE_CYCLES)]
        elif decision == BLOCK_FRONT:
            self._recovery = [("reverse", 0, config.SAFETY_REVERSE_CYCLES),
                              ("pivot", 1, config.SAFETY_PIVOT_CYCLES)]
        elif decision == BLOCK_SIDE:
            self._recovery = self._side_recovery(distances)

    def _side_recovery(self, distances: dict) -> list:
        """Pivot away from the closest side; nothing to do if only the back is close
        (driving forward clears it on its own)."""
        left = distances.get("left", float("inf"))
        right = distances.get("right", float("inf"))
        if min(left, right) < distances.get("back", float("inf")):
            return [("pivot", 1 if left < right else -1, config.SAFETY_PIVOT_CYCLES)]
        return []

    def _step_recovery(self, distances: dict) -> bool:
        """Drive one cycle of the recovery plan. Return True while still recovering."""
        while self._recovery:
            kind, direction, left = self._recovery[0]
            if left <= 0:
                self._recovery.pop(0)
                continue
            if kind == "reverse" and \
                    distances.get("back", float("inf")) < config.SAFETY_BACK_GATE_CM:
                # Something behind -- do not reverse into it; skip to the next phase.
                self._recovery.pop(0)
                continue
            if kind == "reverse":
                motors.set_speed(-config.REVERSE_SPEED, -config.REVERSE_SPEED)
            else:   # pivot: +1 turns right (CW), -1 turns left (CCW)
                motors.set_speed(direction * config.TURN_SPEED,
                                 -direction * config.TURN_SPEED)
            self._recovery[0] = (kind, direction, left - 1)
            return True
        motors.stop()
        return False


def acknowledge_drop() -> None:
    """Clear a latched cliff stop -- called from the web UI."""
    state.set_drop_latched(False)
    state.set_blocked(False)


def start_safety() -> SafetyMonitor:
    monitor = SafetyMonitor()
    monitor.start()
    return monitor
