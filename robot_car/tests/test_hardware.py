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


# ---------------------------------------------------------------------------
# Generated 'bsp' world (recursive-division floor plan)
# ---------------------------------------------------------------------------
def _segment_distances(points, walls):
    """(P,) min distance from each point to any wall segment, vectorised."""
    import numpy as np
    a = np.array([w[0] for w in walls])            # (N, 2)
    b = np.array([w[1] for w in walls])
    e = b - a                                      # (N, 2)
    length2 = np.maximum((e ** 2).sum(axis=1), 1e-12)
    rel = points[:, None, :] - a[None, :, :]       # (P, N, 2)
    t = np.clip((rel * e).sum(axis=2) / length2, 0.0, 1.0)
    closest = a[None, :, :] + t[:, :, None] * e[None, :, :]
    d = np.linalg.norm(points[:, None, :] - closest, axis=2)
    return d.min(axis=1)


def test_bsp_world_is_deterministic_per_seed():
    walls_a, _ = simulator.generate_bsp_world(seed=7)
    walls_b, _ = simulator.generate_bsp_world(seed=7)
    walls_c, _ = simulator.generate_bsp_world(seed=8)
    assert walls_a == walls_b
    assert walls_a != walls_c
    assert len(walls_a) > 4                        # outer box + interior dividers


def test_bsp_world_origin_clear_and_enclosed():
    import numpy as np
    for seed in range(5):
        world = simulator.reset_world(world_name="bsp", start_pose=(0.0, 0.0, 0.0),
                                      seed=seed)
        # The lattice half-cell offset keeps every wall line off the start pose.
        assert not world._collides(0.0, 0.0)
        # Enclosed: a ray in any direction must hit a wall (no escape to infinity).
        for k in range(16):
            angle = 2 * math.pi * k / 16
            hits = [simulator._ray_segment_distance(0.0, 0.0, math.cos(angle),
                                                    math.sin(angle), ax, ay, bx, by)
                    for (ax, ay), (bx, by) in world.walls]
            assert any(h is not None for h in hits)


def test_bsp_world_rooms_all_reachable():
    # Every door the generator carves must actually connect its two rooms for a
    # robot-sized disc: flood-fill the free space at robot clearance and require a
    # single connected component.
    import numpy as np
    from scipy.ndimage import label

    clearance = config.ROBOT_RADIUS + config.INFLATION_MARGIN_M
    step = 0.1
    for seed in range(5):
        walls, _ = simulator.generate_bsp_world(seed=seed)
        xs = np.array([c for seg in walls for c, _ in seg])
        ys = np.array([r for seg in walls for _, r in seg])
        gx = np.arange(xs.min() + step, xs.max() - step / 2, step)
        gy = np.arange(ys.min() + step, ys.max() - step / 2, step)
        pts = np.array([(x, y) for y in gy for x in gx])
        free = (_segment_distances(pts, walls) > clearance).reshape(len(gy), len(gx))
        _, n_components = label(free)
        assert n_components == 1, f"seed {seed}: {n_components} disconnected regions"
