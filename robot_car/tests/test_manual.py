"""F-21 -- manual control override (teleop): mixing, dead-man, directional gate."""

import time

import pytest

from robot_car import config, state
from robot_car.context import RobotContext
from robot_car.controller import ModeController
from robot_car.core import simulator
from robot_car.hardware import hal, motors
from robot_car.modes.manual import Manual, mix_drive, gate_direction


@pytest.fixture
def ctx():
    hal.reset_backend()
    simulator.reset_world(world_name="empty", start_pose=(0.0, 0.0, 0.0))
    state.set_pose((0.0, 0.0, 0.0))
    return RobotContext()


# -- pure mixing / gating ----------------------------------------------------
def test_mix_drive_pure_forward_and_spin():
    left, right = mix_drive(1.0, 0.0)
    assert left == right == config.MANUAL_LINEAR_SPEED
    left, right = mix_drive(0.0, 1.0)              # angular>0 = CCW
    assert left < 0 < right


def test_gate_direction_blocks_into_obstacle_only():
    near = config.STOP_THRESHOLD_CM - 1
    far = config.STOP_THRESHOLD_CM + 50
    assert gate_direction(1.0, near, far) == 0.0   # front blocked -> no forward
    assert gate_direction(-1.0, near, far) == -1.0  # but reverse still allowed
    assert gate_direction(-1.0, far, config.SIDE_STOP_THRESHOLD_CM - 1) == 0.0
    assert gate_direction(1.0, far, far) == 1.0


# -- Manual.step behaviour ---------------------------------------------------
def test_stale_command_stops(ctx):
    state.set_manual_cmd(1.0, 0.0)
    # Forge the timestamp into the past, past the dead-man timeout.
    with state.manual_lock:
        state.manual_cmd = (1.0, 0.0, time.monotonic() - config.MANUAL_CMD_TIMEOUT_S - 1)
    Manual(ctx).step()
    assert motors.get_last_command() == (0.0, 0.0)


def test_fresh_forward_drives(ctx):
    state.set_manual_cmd(1.0, 0.0)
    Manual(ctx).step()
    left, right = motors.get_last_command()
    assert left > 0 and right > 0


def test_front_block_refuses_forward_allows_reverse():
    hal.reset_backend()
    simulator.reset_world(world_name="empty", start_pose=(1.85, 0.0, 0.0))  # ~15cm to wall
    state.set_pose((1.85, 0.0, 0.0))
    ctx = RobotContext()
    man = Manual(ctx)

    state.set_manual_cmd(1.0, 0.0)                 # forward into the wall -> blocked
    man.step()
    assert motors.get_last_command() == (0.0, 0.0)

    state.set_manual_cmd(-1.0, 0.0)                # reverse away -> allowed
    man.step()
    assert motors.get_last_command()[0] < 0


def test_cliff_latch_freezes(ctx):
    state.set_drop_latched(True)
    state.set_manual_cmd(1.0, 0.0)
    Manual(ctx).step()
    assert motors.get_last_command() == (0.0, 0.0)


# -- controller integration --------------------------------------------------
def test_manual_reachable_from_explore_and_back(ctx):
    controller = ModeController(ctx)
    controller.start_explore()
    controller.start_manual()
    assert controller.mode == "manual"
    assert state.get_mode() == "manual"
    assert ctx.slam.mapping is False               # mapping off in manual
    controller.start_idle()
    assert controller.mode == "idle"
    controller.stop()
