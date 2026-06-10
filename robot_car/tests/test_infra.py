"""Phase A -- infrastructure sanity checks."""

import math

from robot_car import config, state
from robot_car.core import simulator
from robot_car.hardware import platform_detect


def test_backend_is_sim_in_tests():
    assert platform_detect.detect_backend() == "sim"


def test_config_invariants():
    assert config.MAP_ORIGIN_COL == config.MAP_WIDTH_CELLS // 2
    assert config.DIST_PER_PULSE > 0
    assert 0 < config.P_FREE < 0.5 < config.P_OCCUPIED < 1
    assert config.WALKABLE_THRESHOLD <= config.GRID_OCCUPIED
    assert math.isclose(config.DIAGONAL_COST, math.sqrt(2))


def test_state_accessors_roundtrip():
    state.set_pose((1.0, 2.0, 0.5))
    assert state.get_pose() == (1.0, 2.0, 0.5)
    state.set_blocked(True)
    assert state.is_blocked() is True
    state.set_mode("explore")
    assert state.get_mode() == "explore"


def test_sim_world_raycast_sees_walls():
    world = simulator.reset_world(world_name="empty", start_pose=(0.0, 0.0, 0.0))
    # 'empty' room spans -2..2 m; front sensor points +x toward the wall at x=2.
    front = world.read_distance_cm("front")
    assert 180 < front < 220        # ~2 m with noise
    # The downward sensor reads the normal floor distance on flat ground.
    down = world.read_distance_cm("down")
    assert abs(down - config.DROP_NORMAL_CM) < 3


def test_sim_motion_and_encoders():
    world = simulator.reset_world(world_name="empty", start_pose=(0.0, 0.0, 0.0))
    world.set_motor(1.0, 1.0)       # full forward
    for _ in range(100):
        world.step(0.01)            # 1 second total
    x, y, theta = world.get_truth_pose()
    assert x > 0.1                  # moved forward in +x
    assert abs(theta) < 0.05        # roughly straight
    left, right = world.read_encoder_pulses()
    assert left > 0 and right > 0
