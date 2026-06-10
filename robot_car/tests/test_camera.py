"""Phase D -- camera visual odometry + appearance detection."""

import numpy as np

from robot_car import config
from robot_car.core import simulator
from robot_car.hardware import camera


def test_vo_recovers_forward_motion():
    # Facing the front wall of the 'empty' world at VO_ASSUMED_DEPTH_M (2.0 m), so
    # the scale-to-metres conversion of the forward-camera model is exact.
    world = simulator.reset_world(world_name="empty", start_pose=(0.0, 0.0, 0.0))
    frame0 = world.render_camera()
    world.set_truth_pose(0.15, 0.0, 0.0)     # jump forward 15 cm
    frame1 = world.render_camera()

    g0 = _gray(frame0)
    g1 = _gray(frame1)
    feats = camera.detect_features(g0)
    assert feats is not None and len(feats) > 20

    est, conf, good_prev, good_next = camera.estimate_motion(g0, g1, feats)
    assert conf > 0.3                        # confident: lots of consistent flow
    assert abs(est[0] - 0.15) < 0.05         # recovers ~+0.15 m forward (robot frame)
    assert est[1] == 0.0                     # lateral unobservable -> reported 0
    assert abs(est[2]) < 0.05                # negligible rotation


def test_vo_recovers_rotation():
    # 0.1 rad is a realistic per-frame yaw at CAMERA_HZ while pivoting.
    world = simulator.reset_world(world_name="empty", start_pose=(0.0, 0.0, 0.0))
    g0 = _gray(world.render_camera())
    world.set_truth_pose(0.0, 0.0, 0.1)      # rotate +0.1 rad (CCW/left)
    g1 = _gray(world.render_camera())
    feats = camera.detect_features(g0)
    est, conf, _, _ = camera.estimate_motion(g0, g1, feats)
    assert abs(est[2] - 0.1) < 0.04
    assert abs(est[0]) < 0.05                # no forward motion


def test_vo_confidence_collapses_on_untrackable_rotation():
    # A whole-frame jump too large for LK must yield low confidence, not a wrong
    # but confident estimate -- fusion then ignores the camera for that cycle.
    world = simulator.reset_world(world_name="empty", start_pose=(0.0, 0.0, 0.0))
    g0 = _gray(world.render_camera())
    world.set_truth_pose(0.0, 0.0, 0.4)
    g1 = _gray(world.render_camera())
    feats = camera.detect_features(g0)
    _, conf, _, _ = camera.estimate_motion(g0, g1, feats)
    assert conf < 0.3


def test_vo_zero_when_stationary():
    world = simulator.reset_world(world_name="empty", start_pose=(0.0, 0.0, 0.0))
    g = _gray(world.render_camera())
    feats = camera.detect_features(g)
    est, conf, _, _ = camera.estimate_motion(g, g, feats)
    # Identical frames -> ~zero motion estimate (drives slip detection downstream).
    assert abs(est[0]) < 0.01 and abs(est[1]) < 0.01 and abs(est[2]) < 0.01


def test_appearance_flags_obstacle_blob():
    world = simulator.reset_world(world_name="empty", start_pose=(0.0, 0.0, 0.0))
    world.camera_obstacle = False
    clear_frame = world.render_camera()
    advisory_clear, _ = camera.detect_appearance(clear_frame)

    world.camera_obstacle = True
    blob_frame = world.render_camera()
    advisory_blob, contours = camera.detect_appearance(blob_frame)
    assert advisory_blob is True
    assert len(contours) >= 1


def test_render_is_trackable_in_generated_world():
    # The raycast view of a BSP-generated floor plan must feed both pipelines:
    # enough corners for VO and no phantom advisory from the wall/floor rendering.
    world = simulator.reset_world(world_name="bsp", start_pose=(0.0, 0.0, 0.0))
    frame = world.render_camera()
    assert frame.shape == (config.CAMERA_HEIGHT, config.CAMERA_WIDTH, 3)
    assert frame.dtype == np.uint8

    feats = camera.detect_features(_gray(frame))
    assert feats is not None and len(feats) > 20

    world.set_truth_pose(0.05, 0.0, 0.02)    # small drive-and-turn step
    est, conf, _, _ = camera.estimate_motion(_gray(frame), _gray(world.render_camera()), feats)
    assert conf > 0.3
    assert abs(est[2] - 0.02) < 0.02


def _gray(frame):
    import cv2
    return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
