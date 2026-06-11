"""Ultrasonic sensor reads -- public interface from feature_plan.md.

Distances are in **centimetres** (the raw HC-SR04 unit); ``float('inf')`` means no
echo. These are non-blocking cache reads served by the :mod:`sensor_scheduler` thread;
``get_reading`` / ``get_all_readings`` also expose each value's age so the SLAM loop
can withhold stale readings from the map.
"""

import time
from collections import namedtuple

from robot_car import config
from robot_car.hardware import sensor_scheduler

SENSOR_NAMES = ("front", "right", "back", "left", "down")

# A reading plus how long ago it was taken and whether that exceeds the stale age.
SensorReading = namedtuple("SensorReading", "distance_cm age stale")


def _to_reading(raw) -> SensorReading:
    age = max(0.0, time.monotonic() - raw.timestamp)
    return SensorReading(raw.distance_cm, age, age > config.SENSOR_STALE_AGE_S)


def get_distance(sensor: str) -> float:
    """Return the latest cached distance in cm for one sensor (``inf`` on timeout)."""
    if sensor not in config.SENSOR_PINS:
        raise ValueError("unknown sensor %r" % sensor)
    return sensor_scheduler.get_scheduler().read(sensor).distance_cm


def get_all_distances() -> dict:
    """Return ``{name: cm}`` for all five sensors from the cache."""
    readings = sensor_scheduler.get_scheduler().read_all()
    return {name: readings[name].distance_cm for name in SENSOR_NAMES}


def get_reading(sensor: str) -> SensorReading:
    """Return ``(distance_cm, age, stale)`` for one sensor."""
    if sensor not in config.SENSOR_PINS:
        raise ValueError("unknown sensor %r" % sensor)
    return _to_reading(sensor_scheduler.get_scheduler().read(sensor))


def get_all_readings() -> dict:
    """Return ``{name: SensorReading}`` for all five sensors."""
    readings = sensor_scheduler.get_scheduler().read_all()
    return {name: _to_reading(readings[name]) for name in SENSOR_NAMES}
