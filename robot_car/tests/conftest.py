"""Shared pytest fixtures."""

import os

import pytest

# Force the simulator backend for the whole test session, regardless of host.
os.environ.setdefault("ROBOT_BACKEND", "sim")

from robot_car import state           # noqa: E402
from robot_car.core import simulator  # noqa: E402
from robot_car.hardware import motors, sensor_scheduler  # noqa: E402


@pytest.fixture(autouse=True)
def clean_state():
    """Reset shared state before and after every test."""
    state.reset()
    motors.reset()
    sensor_scheduler.reset_scheduler()   # drop any cached sensor readings
    yield
    state.reset()
    motors.reset()
    sensor_scheduler.reset_scheduler()


@pytest.fixture
def world():
    """A fresh, manually-stepped simulator world (physics thread not started)."""
    return simulator.reset_world(world_name="room")
