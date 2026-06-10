"""Reactive safety layer (F-05) -- the highest-priority behaviour.

Runs as an independent ~20 Hz daemon thread. Its decisions override navigation with
no exceptions (camera_integration_design.md priority hierarchy): it stops the motors
and raises ``state.blocked`` directly, so a bug in the planner can never disable
collision avoidance.

Responsibilities:
  * Frontal stop + recovery   -- front reading below the stop threshold (tightened
    when the camera advisory flag is set): stop, reverse briefly, pivot away so
    navigation can retry from a clear-ish pose.
  * Side / back scrape stop    -- left/right/back below the (much tighter)
    collision-imminent threshold: stop and pivot away from the closest side.
    Every block must end in a recovery that unblocks -- a stationary hold next to
    a wall would deadlock navigation forever, since the wall never moves.
  * Drop / cliff detection     -- downward sensor reading above the "floor gone"
    threshold latches an emergency stop that requires manual acknowledgement from
    the web UI; below the "obstacle below" threshold triggers stop-and-reverse.

The decision logic is isolated in :func:`evaluate` (pure, unit tested); the thread
only translates a decision into motor actions and flag writes.
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

    def run(self) -> None:
        while not state.stop_event.is_set():
            distances = sensors.get_all_distances()
            decision = evaluate(distances, state.get_advisory(), state.is_drop_latched())
            self._act(decision, distances)
            time.sleep(self.period)

    # -- actions -------------------------------------------------------------
    def _act(self, decision: str, distances: dict | None = None) -> None:
        if decision == CLEAR:
            state.set_blocked(False)
            return

        # Everything else means "do not let navigation drive right now".
        motors.stop()
        state.set_blocked(True)

        if decision == DROP_CLIFF:
            state.set_drop_latched(True)            # needs manual ack to clear
            state.set_log("error", "Cliff detected -- emergency stop. Acknowledge to resume.")
        elif decision == DROP_OBSTACLE:
            self._reverse()
            state.set_blocked(False)
        elif decision == BLOCK_FRONT:
            self._reverse()
            self._pivot()
            state.set_blocked(False)                # allow navigation to retry
        elif decision == BLOCK_SIDE:
            self._recover_side(distances or {})
            state.set_blocked(False)
        elif decision == LATCHED:
            pass                                    # frozen until acknowledged

    def _recover_side(self, distances: dict) -> None:
        """Pivot away from the closest side; a back reading clears by itself when
        navigation resumes driving forward."""
        left = distances.get("left", float("inf"))
        right = distances.get("right", float("inf"))
        if min(left, right) < distances.get("back", float("inf")):
            self._pivot(direction=1 if left < right else -1)

    def _sleep_or_abort(self, seconds: float) -> None:
        """Sleep, but bail out immediately on shutdown."""
        state.stop_event.wait(seconds)

    def _reverse(self) -> None:
        motors.set_speed(-config.REVERSE_SPEED, -config.REVERSE_SPEED)
        self._sleep_or_abort(config.SAFETY_REVERSE_TIME_S)
        motors.stop()

    def _pivot(self, direction: int = 1) -> None:
        """Pivot in place; +1 turns right (clockwise), -1 turns left."""
        motors.set_speed(direction * config.TURN_SPEED, -direction * config.TURN_SPEED)
        self._sleep_or_abort(config.SAFETY_PIVOT_TIME_S)
        motors.stop()


def acknowledge_drop() -> None:
    """Clear a latched cliff stop -- called from the web UI."""
    state.set_drop_latched(False)
    state.set_blocked(False)


def start_safety() -> SafetyMonitor:
    monitor = SafetyMonitor()
    monitor.start()
    return monitor
