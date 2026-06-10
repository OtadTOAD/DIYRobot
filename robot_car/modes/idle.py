"""Idle mode (F-15) -- motors stopped, waiting for the next command."""

from robot_car import state
from robot_car.hardware import motors


class Idle:
    def __init__(self, context):
        self.ctx = context

    def run(self, stop_event) -> None:
        motors.stop()
        state.set_mode("idle")
        # Nothing to do until the mode controller starts a different behaviour.
        while not stop_event.is_set() and not state.stop_event.is_set():
            stop_event.wait(0.1)
        motors.stop()
