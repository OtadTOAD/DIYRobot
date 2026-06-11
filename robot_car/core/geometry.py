"""Shared geometry helpers."""

import math

from robot_car import config


def wrap_angle(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def body_to_world(pose, dx: float, dy: float) -> tuple:
    """Transform a robot-frame point ``(dx forward, dy left)`` into world metres."""
    x, y, theta = pose
    c, s = math.cos(theta), math.sin(theta)
    return (x + dx * c - dy * s, y + dx * s + dy * c)


def sensor_origin(pose, sensor: str) -> tuple:
    """World position of a sensor's mount for this pose (applies SENSOR_OFFSETS)."""
    return body_to_world(pose, *config.SENSOR_OFFSETS.get(sensor, (0.0, 0.0)))
