"""Phase G/J -- full-stack integration in the simulator.

Spins up the real backend (sim), SLAM and safety threads, then drives the actual
explore and navigate behaviours end-to-end against a synthetic world -- exactly the
code path that runs on the Pi, minus the GPIO.
"""

import math
import os
import threading
import time

import numpy as np
import pytest

from robot_car import config, state
from robot_car.context import RobotContext
from robot_car.core import simulator
from robot_car.core.safety_monitor import SafetyMonitor
from robot_car.hardware import hal, motors
from robot_car.modes.explore import Explorer
from robot_car.modes.navigate import Navigator, REACHED


class Stack:
    """Bring up backend + SLAM + safety; tear everything down cleanly."""

    def __init__(self, world="empty", start=(0.0, 0.0, 0.0)):
        state.reset()
        hal.reset_backend()
        simulator.reset_world(world_name=world, start_pose=start)
        state.set_pose(start)
        self.ctx = RobotContext()
        self.ctx.slam.odometry.set_pose(start)
        self.safety = SafetyMonitor()

    def __enter__(self):
        hal.get_backend().start()        # sim physics thread
        self.ctx.slam.start()
        self.safety.start()
        time.sleep(0.3)                  # let the first SLAM cycles publish a grid
        return self

    def __exit__(self, *exc):
        state.stop_event.set()
        motors.stop()
        time.sleep(0.2)
        hal.reset_backend()


def test_navigate_reaches_goal_in_sim():
    with Stack(world="empty", start=(0.0, 0.0, 0.0)) as s:
        nav = Navigator(s.ctx)
        goal = s.ctx.slam.grid.world_to_grid(0.7, 0.0)   # 0.7 m straight ahead
        stop = threading.Event()

        result = {}
        t = threading.Thread(target=lambda: result.setdefault("r", nav.run(goal, stop)))
        t.start()
        t.join(timeout=25)
        stop.set()
        t.join(timeout=2)

        x, y, _ = state.get_pose()
        assert result.get("r") == REACHED
        assert math.hypot(0.7 - x, 0.0 - y) < 0.15


def test_explore_builds_and_saves_map_in_sim(tmp_path):
    # Save into a temp maps dir so we don't litter the repo.
    config.MAPS_DIR = str(tmp_path)
    with Stack(world="room", start=(0.0, 0.0, 0.0)) as s:
        explorer = Explorer(s.ctx, save_name="unit_room")
        stop = threading.Event()
        t = threading.Thread(target=lambda: explorer.run(stop))
        t.start()
        # Give exploration a bounded budget, then stop (saves partial map).
        time.sleep(12)
        stop.set()
        t.join(timeout=5)

        grid = state.get_grid()
        occupied = int((grid > config.INFLATION_OCCUPIED_THRESHOLD).sum())
        free = int((grid < config.FRONTIER_FREE_THRESHOLD).sum())
        assert occupied > 50            # walls were mapped
        assert free > 200               # open space was mapped
        assert os.path.exists(os.path.join(str(tmp_path), "unit_room.map"))
