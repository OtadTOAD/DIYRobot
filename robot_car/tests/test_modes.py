"""Phase G -- mode behaviours and the mode controller (unit-level)."""

import math
import threading

import numpy as np
import pytest

from robot_car import config, state
from robot_car.context import RobotContext
from robot_car.controller import ModeController, InvalidTransition
from robot_car.core import simulator
from robot_car.core.slam import SlamSystem
from robot_car.hardware import hal, motors
from robot_car.modes.explore import cluster_frontiers
from robot_car.modes.navigate import Navigator, NO_PATH


@pytest.fixture
def ctx():
    hal.reset_backend()
    simulator.reset_world(world_name="empty", start_pose=(0.0, 0.0, 0.0))
    state.set_pose((0.0, 0.0, 0.0))
    return RobotContext()


def test_cluster_frontiers_finds_region():
    grid = np.full((60, 60), config.GRID_UNKNOWN, dtype=np.uint8)
    grid[20:30, 20:30] = config.GRID_FREE       # known-free island in unknown space
    centroids = cluster_frontiers(grid)
    assert len(centroids) >= 1
    col, row = centroids[0]
    assert 18 <= col <= 31 and 18 <= row <= 31


def test_step_toward_drives_when_aligned(ctx):
    state.set_pose((0.0, 0.0, 0.0))
    nav = Navigator(ctx)
    target = ctx.slam.grid.world_to_grid(0.5, 0.0)   # straight ahead
    nav._step_toward(target)
    left, right = motors.get_last_command()
    assert left > 0 and right > 0                    # driving forward


def test_step_toward_rotates_when_misaligned(ctx):
    state.set_pose((0.0, 0.0, 0.0))
    nav = Navigator(ctx)
    target = ctx.slam.grid.world_to_grid(0.0, 0.5)   # 90 deg to the left
    nav._step_toward(target)
    left, right = motors.get_last_command()
    assert left < 0 and right > 0                    # rotating counter-clockwise


def test_step_toward_reached(ctx):
    state.set_pose((0.0, 0.0, 0.0))
    nav = Navigator(ctx)
    here = ctx.slam.grid.world_to_grid(0.0, 0.0)
    from robot_car.modes.navigate import REACHED
    assert nav._step_toward(here) == REACHED


def test_scan_motion_guard_skips_fast_rotation():
    ok = SlamSystem._scan_motion_ok
    assert ok(None, (0.0, 0.0, 0.0)) is True            # first cycle always maps
    assert ok((0.0, 0.0, 0.0), (0.0, 0.0, math.radians(2))) is True   # slow turn
    assert ok((0.0, 0.0, 0.0), (0.0, 0.0, math.radians(20))) is False  # spin -> skip
    assert ok((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)) is False              # big jump -> skip


def test_navigate_stall_watchdog_gives_up(ctx, monkeypatch):
    # Pose never changes (no sim physics / SLAM running), so the robot can make no
    # progress toward a reachable goal -- the watchdog must abandon it, not loop.
    monkeypatch.setattr(config, "NAV_PROGRESS_TIMEOUT_S", 0.3)
    state.set_pose((0.0, 0.0, 0.0))
    nav = Navigator(ctx)
    goal = ctx.slam.grid.world_to_grid(1.0, 0.0)
    assert nav.run(goal, threading.Event()) == NO_PATH


def test_controller_navigate_without_map_rejected(ctx):
    state.set_grid(None)
    controller = ModeController(ctx)
    with pytest.raises(InvalidTransition):
        controller.navigate_to((10, 10))


def test_controller_marks_mode(ctx):
    controller = ModeController(ctx)
    seen = []
    controller.on_mode = seen.append
    controller.start_idle()
    assert controller.mode == "idle"
    assert state.get_mode() == "idle"
    controller.stop()
    assert "idle" in seen
