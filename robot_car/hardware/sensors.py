"""Ultrasonic sensor reads -- public interface from feature_plan.md.

Distances are returned in **centimetres** (the raw HC-SR04 unit); callers that work
in world coordinates convert to metres immediately. ``float('inf')`` means no echo
within the timeout (open space or out of range).
"""

from robot_car import config
from robot_car.hardware import hal

SENSOR_NAMES = ("front", "right", "back", "left", "down")


def get_distance(sensor: str) -> float:
    """Return distance in cm for one sensor, or ``float('inf')`` on timeout."""
    if sensor not in config.SENSOR_PINS:
        raise ValueError("unknown sensor %r" % sensor)
    return hal.get_backend().read_distance_cm(sensor)


def get_all_distances() -> dict:
    """Return ``{name: cm}`` for all five sensors."""
    backend = hal.get_backend()
    return {name: backend.read_distance_cm(name) for name in SENSOR_NAMES}
