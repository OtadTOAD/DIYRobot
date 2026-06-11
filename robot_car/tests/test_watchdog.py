"""P0-3 -- dead-man motor gating and the thread watchdog."""

import time

from robot_car import config, state
from robot_car.core import simulator
from robot_car.core.watchdog import Watchdog
from robot_car.hardware import hal, motors


def _fresh():
    hal.reset_backend()
    simulator.reset_world(world_name="empty", start_pose=(0.0, 0.0, 0.0))
    motors.reset()
    state.clear_heartbeats()


def test_motors_drive_when_safety_never_started():
    # No safety heartbeat ever recorded -> not gated (unit tests drive directly).
    _fresh()
    motors.set_speed(0.5, 0.5)
    assert motors.get_last_command() == (0.5, 0.5)


def test_motors_drive_with_fresh_safety_heartbeat():
    _fresh()
    state.beat("safety")
    motors.set_speed(0.5, 0.5)
    assert motors.get_last_command() == (0.5, 0.5)


def test_motors_refuse_drive_on_stale_safety_heartbeat():
    _fresh()
    state.beat("safety")
    # Force the heartbeat into the past, past the dead-man timeout.
    with state._heartbeat_lock:
        state._heartbeats["safety"] = (
            time.monotonic() - config.SAFETY_DEADMAN_TIMEOUT_S - 0.1)
    motors.set_speed(0.5, 0.5)
    assert motors.get_last_command() == (0.0, 0.0)     # forced to a stop


def test_watchdog_reports_stall_once_then_recovery():
    _fresh()
    logs = []
    state.add_log_listener(lambda level, msg: logs.append((level, msg)))
    wd = Watchdog()

    with state._heartbeat_lock:
        state._heartbeats["slam"] = time.monotonic() - config.WATCHDOG_STALE_S - 1.0
    wd._check()
    wd._check()                                        # idempotent: only one report
    errors = [m for lvl, m in logs if lvl == "error" and "slam" in m]
    assert len(errors) == 1

    state.beat("slam")                                 # thread came back
    wd._check()
    assert any(lvl == "ok" and "slam" in m and "recovered" in m for lvl, m in logs)
