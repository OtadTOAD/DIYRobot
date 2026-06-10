"""Phase B -- hardware HAL + odometry against the simulator backend."""

import math

from robot_car import config
from robot_car.core import simulator, odometry
from robot_car.hardware import hal, motors, sensors


def _fresh_world(name="empty", pose=(0.0, 0.0, 0.0)):
    hal.reset_backend()
    return simulator.reset_world(world_name=name, start_pose=pose)


def test_backend_is_sim():
    hal.reset_backend()
    assert hal.get_backend().name == "sim"


def test_motors_drive_world_forward():
    world = _fresh_world()
    motors.set_speed(1.0, 1.0)
    for _ in range(100):
        world.step(0.01)
    motors.stop()
    x, y, theta = world.get_truth_pose()
    assert x > 0.1
    assert math.isclose(world._left_cmd, 0.0)   # stop() cleared the command


def test_motors_record_last_command():
    _fresh_world()
    motors.set_speed(0.5, -0.5)
    assert motors.get_last_command() == (0.5, -0.5)


def test_sensors_report_distances():
    _fresh_world()
    readings = sensors.get_all_distances()
    assert set(readings) == set(sensors.SENSOR_NAMES)
    assert 180 < readings["front"] < 220        # ~2 m wall ahead


def test_odometry_straight_line():
    world = _fresh_world()
    odo = odometry.Odometry()
    motors.set_speed(1.0, 1.0)
    for _ in range(100):
        world.step(0.01)
    odo.update()
    x, y, theta = odo.get_pose()
    truth = world.get_truth_pose()
    # Encoder estimate should track ground truth within a few cm.
    assert abs(x - truth[0]) < 0.05
    assert abs(theta) < 0.1


def test_odometry_in_place_turn():
    world = _fresh_world()
    odo = odometry.Odometry()
    motors.set_speed(-0.6, 0.6)        # spin left (CCW)
    for _ in range(100):
        world.step(0.01)
    odo.update()
    _, _, theta = odo.get_pose()
    assert theta > 0.1                  # turned counter-clockwise
