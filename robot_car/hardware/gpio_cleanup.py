"""Graceful shutdown -- stop motors and release the active backend.

Registered with ``atexit`` and called explicitly from the SIGINT handler in main.py
so that motors never latch on and GPIO never lingers in an undefined state.
"""

import atexit

from robot_car.hardware import hal, motors

_done = False


def cleanup() -> None:
    """Idempotent: stop motors then tear down GPIO / simulator."""
    global _done
    if _done:
        return
    _done = True
    try:
        motors.stop()
    except Exception:
        pass
    hal.reset_backend()


def register() -> None:
    atexit.register(cleanup)
