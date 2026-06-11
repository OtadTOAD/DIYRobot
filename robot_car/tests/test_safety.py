"""Phase C -- safety monitor decision logic and flag handling."""

from robot_car import config, state
from robot_car.core import safety_monitor as sm
from robot_car.core import simulator
from robot_car.hardware import hal, motors, sensors

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
    monitor.step(sm.DROP_CLIFF)
    assert state.is_blocked() is True
    assert state.is_drop_latched() is True
    sm.acknowledge_drop()
    assert state.is_drop_latched() is False
    assert state.is_blocked() is False


def test_clear_decision_unblocks():
    state.set_blocked(True)
    sm.SafetyMonitor().step(sm.CLEAR)
    assert state.is_blocked() is False


# ---------------------------------------------------------------------------
# P0-2 -- stepped, non-blocking recovery state machine
# ---------------------------------------------------------------------------
def test_front_block_steps_reverse_then_pivot_then_unblocks():
    _fresh_motors()
    mon = sm.SafetyMonitor()
    clear = _dist()
    # First cycle on the hazard: stop + block + queue reverse(+pivot), no driving yet.
    mon.step(sm.BLOCK_FRONT, clear)
    assert state.is_blocked() is True
    assert len(mon._recovery) == 2
    # Reverse phase drives backward, every cycle still monitoring (no sleep).
    mon.step(sm.BLOCK_FRONT, clear)
    assert motors.get_last_command()[0] < 0
    # Drive the whole plan; when it finishes it unblocks for a cycle so the navigator
    # can retry (a still-present BLOCK_FRONT then simply re-queues another attempt).
    saw_unblocked = False
    for _ in range(config.SAFETY_REVERSE_CYCLES + config.SAFETY_PIVOT_CYCLES + 1):
        mon.step(sm.BLOCK_FRONT, clear)
        if not state.is_blocked():
            saw_unblocked = True
            break
    assert saw_unblocked


def test_reverse_phase_is_back_sensor_gated():
    _fresh_motors()
    mon = sm.SafetyMonitor()
    mon.step(sm.BLOCK_FRONT, _dist())                 # queue reverse + pivot
    blocked_behind = _dist(back=config.SAFETY_BACK_GATE_CM - 1)
    # With an obstacle behind, the reverse phase is abandoned -- it must never command
    # a backward drive into it; the next phase (pivot) takes over.
    for _ in range(config.SAFETY_REVERSE_CYCLES + 1):
        mon.step(sm.BLOCK_FRONT, blocked_behind)
        assert motors.get_last_command()[0] >= 0      # never reversed


def test_cliff_preempts_active_recovery():
    _fresh_motors()
    mon = sm.SafetyMonitor()
    mon.step(sm.BLOCK_FRONT, _dist())
    mon.step(sm.BLOCK_FRONT, _dist())                 # mid-reverse
    mon.step(sm.DROP_CLIFF, _dist(down=config.DROP_FLOOR_GONE_CM + 5))
    assert mon._recovery == []                        # maneuver dropped
    assert state.is_drop_latched() is True


def test_manual_mode_suppresses_recovery():
    _fresh_motors()
    state.set_mode("manual")
    mon = sm.SafetyMonitor()
    mon.step(sm.BLOCK_FRONT, _dist())
    assert state.is_blocked() is True                 # still decides "blocked"
    assert mon._recovery == []                        # but starts no maneuver


def _fresh_motors():
    from robot_car.hardware import hal
    hal.reset_backend()
    simulator.reset_world(world_name="empty", start_pose=(0.0, 0.0, 0.0))
    motors.reset()


def test_real_sensors_trigger_front_block_near_wall():
    hal.reset_backend()
    # Wall of the 'empty' room is at x = 2 m; sit 15 cm from it.
    simulator.reset_world(world_name="empty", start_pose=(1.85, 0.0, 0.0))
    distances = sensors.get_all_distances()
    decision = sm.evaluate(distances, advisory=False, latched=False)
    assert decision == sm.BLOCK_FRONT
