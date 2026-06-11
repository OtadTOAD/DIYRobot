"""P0-1 -- the single ultrasonic sensor scheduler and its non-blocking cache."""

import time

from robot_car import config, state
from robot_car.core import simulator
from robot_car.hardware import hal, motors, sensor_scheduler, sensors


def _fresh(world="empty", pose=(0.0, 0.0, 0.0)):
    hal.reset_backend()
    return simulator.reset_world(world_name=world, start_pose=pose)


def test_cold_read_pings_without_a_running_thread():
    # No scheduler thread started: the first consumer read falls back to a
    # synchronous ping so the value is real, not a placeholder.
    _fresh()
    readings = sensors.get_all_readings()
    assert set(readings) == set(sensors.SENSOR_NAMES)
    assert 180 < readings["front"].distance_cm < 220     # ~2 m wall ahead
    assert readings["front"].stale is False


def test_reading_goes_stale_with_age():
    _fresh()
    sched = sensor_scheduler.get_scheduler()
    raw = sched.read("front")
    fresh = sensors._to_reading(raw)
    assert fresh.stale is False
    # Forge a timestamp older than the stale age.
    old = sensor_scheduler.Reading(raw.distance_cm,
                                   raw.timestamp - config.SENSOR_STALE_AGE_S - 0.1)
    assert sensors._to_reading(old).stale is True


def test_travel_priority_follows_motor_command():
    _fresh()
    motors.set_speed(0.5, 0.5)
    assert sensor_scheduler._travel_priority() == "front"
    motors.set_speed(-0.5, -0.5)
    assert sensor_scheduler._travel_priority() == "back"
    motors.set_speed(-0.4, 0.4)                # pivot -- no net translation
    assert sensor_scheduler._travel_priority() is None
    motors.stop()
    assert sensor_scheduler._travel_priority() is None


def test_priority_sensor_is_interleaved_every_other_slot():
    _fresh()
    motors.set_speed(0.5, 0.5)                 # priority = front
    sched = sensor_scheduler.SensorScheduler()
    picks = [sched._next_sensor() for _ in range(6)]
    # Every other pick is the travel sensor; the rest advance the round-robin.
    assert picks[0::2] == ["front", "front", "front"]
    assert picks[1::2] == ["front", "right", "back"]


def test_running_scheduler_keeps_cache_fresh():
    state.stop_event.clear()
    _fresh()
    sched = sensor_scheduler.start_scheduler()
    try:
        time.sleep(0.4)                        # several ping slots
        readings = sensors.get_all_readings()
        assert all(r.age < config.SENSOR_STALE_AGE_S for r in readings.values())
        assert all(not r.stale for r in readings.values())
    finally:
        state.stop_event.set()
        sched.join(timeout=1.0)
