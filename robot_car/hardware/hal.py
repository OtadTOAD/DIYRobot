"""Hardware abstraction layer -- selects the active backend once at startup.

A *backend* is any object implementing the methods below. Two exist:

    backends/sim.py   SimBackend   -- talks to the 2D simulator (laptop / CI)
    backends/real.py  RealBackend  -- talks to pigpio + cv2 on a Raspberry Pi

The public ``hardware/*`` modules (motors, sensors, camera, odometry) call
``get_backend()`` and never import GPIO libraries directly, so the same higher-level
code runs in both worlds.

Backend interface:
    start() -> None                       initialise hardware / start the sim physics
    motor_set(left: float, right: float)  wheel speeds, -1..1 (fraction of full PWM)
    motor_stop() -> None
    ping_sensor(sensor: str) -> float       one blocking ping, cm ('front'|'right'|
                                            'back'|'left'|'down'); driven only by the
                                            sensor_scheduler, never by consumers
    read_encoder_pulses() -> (int, int)     (left, right) pulses since last call
    camera_read() -> np.ndarray | None      latest BGR frame
    cleanup() -> None                        stop motors, release GPIO / sim
"""

import threading

from robot_car.hardware import platform_detect

_backend = None
_lock = threading.Lock()


def _create_backend():
    if platform_detect.ACTIVE_BACKEND == "pi":
        from robot_car.hardware.backends.real import RealBackend
        return RealBackend()
    from robot_car.hardware.backends.sim import SimBackend
    return SimBackend()


def get_backend():
    """Return the process-wide backend singleton."""
    global _backend
    with _lock:
        if _backend is None:
            _backend = _create_backend()
        return _backend


def reset_backend():
    """Drop the cached backend (used by tests)."""
    global _backend
    # A new backend/world invalidates any cached sensor readings. Lazy import avoids
    # an import cycle (sensor_scheduler imports hal).
    from robot_car.hardware import sensor_scheduler
    sensor_scheduler.reset_scheduler()
    with _lock:
        if _backend is not None:
            try:
                _backend.cleanup()
            except Exception:
                pass
        _backend = None
