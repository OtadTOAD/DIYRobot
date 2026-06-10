"""Phase C -- safety monitor decision logic and flag handling."""

from robot_car import config, state
from robot_car.core import safety_monitor as sm
from robot_car.core import simulator
from robot_car.hardware import hal, sensors

INF = float("inf")


def _dist(front=INF, left=INF, right=INF, back=INF, down=config.DROP_NORMAL_CM):
    return {"front": front, "left": left, "right": right, "back": back, "down": down}


def test_evaluate_clear():
    assert sm.evaluate(_dist(), advisory=False, latched=False) == sm.CLEAR


def test_evaluate_front_block():
    d = _dist(front=config.STOP_THRESHOLD_CM - 1)
    assert sm.evaluate(d, advisory=False, latched=False) == sm.BLOCK_FRONT


def test_evaluate_side_block_only_when_collision_imminent():
    d = _dist(left=config.SIDE_STOP_THRESHOLD_CM - 1)
    assert sm.evaluate(d, advisory=False, latched=False) == sm.BLOCK_SIDE
    # Skirting a wall at front-threshold range must NOT block -- planned paths
    # legitimately run this close, and a side hold would deadlock navigation.
    d = _dist(left=config.STOP_THRESHOLD_CM - 1)
    assert sm.evaluate(d, advisory=False, latched=False) == sm.CLEAR


def test_advisory_tightens_threshold():
    # A reading clear of the base threshold but inside the advisory-tightened one.
    gap = config.STOP_THRESHOLD_CM + config.ADVISORY_TIGHTEN_CM - 1
    d = _dist(front=gap)
    assert sm.evaluate(d, advisory=False, latched=False) == sm.CLEAR
    assert sm.evaluate(d, advisory=True, latched=False) == sm.BLOCK_FRONT


def test_evaluate_drop_cliff_and_obstacle():
    assert sm.evaluate(_dist(down=config.DROP_FLOOR_GONE_CM + 5),
                       advisory=False, latched=False) == sm.DROP_CLIFF
    assert sm.evaluate(_dist(down=config.DROP_OBSTACLE_CM - 1),
                       advisory=False, latched=False) == sm.DROP_OBSTACLE


def test_latched_overrides_everything():
    assert sm.evaluate(_dist(), advisory=False, latched=True) == sm.LATCHED


def test_cliff_latches_and_acknowledge_clears():
    monitor = sm.SafetyMonitor()
    monitor._act(sm.DROP_CLIFF)
    assert state.is_blocked() is True
    assert state.is_drop_latched() is True
    sm.acknowledge_drop()
    assert state.is_drop_latched() is False
    assert state.is_blocked() is False


def test_clear_decision_unblocks():
    state.set_blocked(True)
    sm.SafetyMonitor()._act(sm.CLEAR)
    assert state.is_blocked() is False


def test_real_sensors_trigger_front_block_near_wall():
    hal.reset_backend()
    # Wall of the 'empty' room is at x = 2 m; sit 15 cm from it.
    simulator.reset_world(world_name="empty", start_pose=(1.85, 0.0, 0.0))
    distances = sensors.get_all_distances()
    decision = sm.evaluate(distances, advisory=False, latched=False)
    assert decision == sm.BLOCK_FRONT
