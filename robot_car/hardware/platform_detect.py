"""Decide at runtime whether we are running on real Raspberry Pi hardware.

The rest of the codebase is hardware-agnostic: it asks ``select_backend()`` which
backend to use and never imports ``RPi.GPIO`` / ``pigpio`` directly. On a non-Pi
machine (a laptop, CI) the simulator backend is selected automatically so the full
stack -- SLAM, A*, explore/navigate, web UI -- runs unchanged.

Selection order:
    1. config.BACKEND / env ROBOT_BACKEND == 'pi'  -> force real
    2. config.BACKEND / env ROBOT_BACKEND == 'sim' -> force simulator
    3. 'auto' -> real if a Pi is detected AND the GPIO libraries import, else sim
"""

from robot_car import config


def _is_raspberry_pi() -> bool:
    """Best-effort Raspberry Pi detection via the device-tree model string."""
    try:
        with open("/proc/device-tree/model", "r") as fh:
            model = fh.read()
        if "raspberry pi" in model.lower():
            return True
    except OSError:
        pass
    return False


def _gpio_available() -> bool:
    """True if the Pi GPIO libraries can actually be imported."""
    try:
        import RPi.GPIO  # noqa: F401
        return True
    except Exception:
        return False


def detect_backend() -> str:
    """Return either ``'pi'`` or ``'sim'``."""
    forced = config.BACKEND.lower()
    if forced == "pi":
        return "pi"
    if forced == "sim":
        return "sim"
    # auto
    if _is_raspberry_pi() and _gpio_available():
        return "pi"
    return "sim"


# Resolved once at import time; cheap and stable for the process lifetime.
ACTIVE_BACKEND = detect_backend()
IS_PI = ACTIVE_BACKEND == "pi"
