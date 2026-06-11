"""Phase E -- localization fusion and slip detection."""

import math

import pytest

from robot_car import config, state
from robot_car.core import localization as loc


def test_vo_confidence_is_min_over_window():
    # One lucky frame must not launder a delta accumulated over garbage ones (P1-5).
    state.add_vo((0.1, 0.0, 0.01), 0.9)
    state.add_vo((0.1, 0.0, 0.01), 0.2)
    state.add_vo((0.1, 0.0, 0.01), 0.8)
    delta, conf = state.consume_vo()
    assert conf == 0.2
    assert delta[0] == pytest.approx(0.3)
    assert state.consume_vo()[1] == 0.0              # reset after consume


def test_fuse_falls_back_to_encoder_when_others_zero():
    enc = (1.0, 2.0, 0.5)
    out = loc.fuse_poses(enc, (9, 9, 9), (9, 9, 9), w_e=0.55, w_s=0.0, w_v=0.0)
    assert out == enc


def test_fuse_blends_equal_weights():
    a = (0.0, 0.0, 0.0)
    b = (2.0, 4.0, 0.0)
    out = loc.fuse_poses(a, b, (0, 0, 0), w_e=1.0, w_s=1.0, w_v=0.0)
    assert math.isclose(out[0], 1.0)
    assert math.isclose(out[1], 2.0)


def test_fuse_angle_wraps_correctly():
    # Average of +170deg and -170deg should be ~180deg, not ~0.
    a = (0.0, 0.0, math.radians(170))
    b = (0.0, 0.0, math.radians(-170))
    out = loc.fuse_poses(a, b, (0, 0, 0), 1.0, 1.0, 0.0)
    assert abs(abs(out[2]) - math.pi) < 0.05


def test_weights_slip_halves_encoder():
    w_e, _, _ = loc.compute_weights(0.0, False, 0.0, slip=False)
    w_e_slip, _, _ = loc.compute_weights(0.0, False, 0.0, slip=True)
    assert math.isclose(w_e_slip, w_e * config.SLIP_ENCODER_PENALTY)


def test_weights_scan_zero_when_not_accepted():
    _, w_s, _ = loc.compute_weights(0.9, False, 0.0, slip=False)
    assert w_s == 0.0
    _, w_s_ok, _ = loc.compute_weights(0.9, True, 0.0, slip=False)
    assert w_s_ok > 0.0


def test_detect_slip():
    # Encoders say we moved, confident camera says we didn't -> slip.
    assert loc.detect_slip(0.10, (0.0, 0.0, 0.0), vo_confidence=0.8) is True
    # Camera agrees we moved -> no slip.
    assert loc.detect_slip(0.10, (0.09, 0.0, 0.0), vo_confidence=0.8) is False
    # Camera not confident -> don't override encoders.
    assert loc.detect_slip(0.10, (0.0, 0.0, 0.0), vo_confidence=0.1) is False
    # Barely moved -> no slip check.
    assert loc.detect_slip(0.001, (0.0, 0.0, 0.0), vo_confidence=0.9) is False
