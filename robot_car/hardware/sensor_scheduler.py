"""Single ultrasonic sensor scheduler -- the one owner of the HC-SR04 bus.

Pinging two sensors at once makes them hear each other's echoes, and each blocking
read costs tens of milliseconds, so one thread round-robins ``ping_sensor`` calls
spaced by ``SENSOR_PING_SPACING_S`` and caches each result with a timestamp. The
direction-of-travel sensor is pinged every other slot to keep safety responsive.
Consumers read the cache (``sensors.get_*``) and never block on the bus. All of this
is backend-agnostic, so the simulator is sampled at the same cadence as the hardware.
"""

from __future__ import annotations

import threading
import time
from collections import namedtuple

from robot_car import config, state
from robot_car.hardware import hal

# Order the round-robin sweeps through. 'down' (cliff) is included so a drop is still
# sensed; the travel-direction sensor is interleaved on top of this rotation.
SWEEP_ORDER = ("front", "right", "back", "left", "down")

Reading = namedtuple("Reading", "distance_cm timestamp")


def _travel_priority() -> str | None:
    """Sensor in the current direction of travel, or None when not translating."""
    # Imported lazily to avoid a hardware<->motors import cycle at module load.
    from robot_car.hardware import motors
    left, right = motors.get_last_command()
    avg = (left + right) / 2.0
    if avg > 1e-3:
        return "front"
    if avg < -1e-3:
        return "back"
    return None


class SensorScheduler(threading.Thread):
    """Owns the sensor bus: pings round-robin and publishes a non-blocking cache."""

    def __init__(self):
        super().__init__(name="sensor-scheduler", daemon=True)
        self._cache: dict[str, Reading] = {}
        self._lock = threading.Lock()
        self._rr = 0                 # round-robin cursor into SWEEP_ORDER
        self._serve_priority = True  # alternate priority / round-robin slots

    # -- the bus -------------------------------------------------------------
    def _ping(self, sensor: str) -> Reading:
        cm = hal.get_backend().ping_sensor(sensor)
        reading = Reading(cm, time.monotonic())
        with self._lock:
            self._cache[sensor] = reading
        return reading

    def _next_sensor(self) -> str:
        """Pick the next sensor: alternate the travel sensor with the round-robin."""
        priority = _travel_priority()
        if priority is not None and self._serve_priority:
            self._serve_priority = False
            return priority
        self._serve_priority = True
        sensor = SWEEP_ORDER[self._rr % len(SWEEP_ORDER)]
        self._rr += 1
        return sensor

    def run(self) -> None:
        # Prime the cache so the first consumer cycle has real values, not 'inf'.
        for sensor in SWEEP_ORDER:
            if state.stop_event.is_set():
                return
            self._ping(sensor)
            time.sleep(config.SENSOR_PING_SPACING_S)
        while not state.stop_event.is_set():
            state.beat("sensors")
            self._ping(self._next_sensor())
            time.sleep(config.SENSOR_PING_SPACING_S)

    # -- consumer API --------------------------------------------------------
    def read(self, sensor: str) -> Reading:
        """Latest cached reading, pinging synchronously only if the cache is cold
        (before the thread's first sample, or in tests that never start it)."""
        with self._lock:
            reading = self._cache.get(sensor)
        if reading is None:
            reading = self._ping(sensor)
        return reading

    def read_all(self) -> dict[str, Reading]:
        return {name: self.read(name) for name in SWEEP_ORDER}


# ---------------------------------------------------------------------------
# Process-wide singleton (mirrors hal.get_backend()).
# ---------------------------------------------------------------------------
_scheduler: SensorScheduler | None = None
_scheduler_lock = threading.Lock()


def get_scheduler() -> SensorScheduler:
    global _scheduler
    with _scheduler_lock:
        if _scheduler is None:
            _scheduler = SensorScheduler()
        return _scheduler


def start_scheduler() -> SensorScheduler:
    scheduler = get_scheduler()
    if not scheduler.is_alive():
        scheduler.start()
    return scheduler


def reset_scheduler() -> None:
    """Drop the singleton (used by tests so a fresh backend/world is picked up)."""
    global _scheduler
    with _scheduler_lock:
        _scheduler = None
