"""Three-source localization fusion (F-11).

Combines the encoder dead-reckoning pose, the scan-matching correction and the
visual-odometry estimate into a single pose, weighting each by a confidence that
adapts to conditions (slam_and_localization_design.md section 9):

  * encoders     -- base estimate, weight halved when wheel slip is detected;
  * scan match    -- weight proportional to match score, ~0 when below threshold
    (featureless areas);
  * visual odom   -- weight proportional to the camera's flow confidence (drops in
    poor lighting / fast motion / low texture).

Angles are blended through their sin/cos components so the result is correct across
the +-pi wrap.
"""

import math

from robot_car import config


def compute_weights(scan_score: float, scan_accepted: bool,
                    vo_confidence: float, slip: bool):
    """Return adaptive ``(w_encoder, w_scan, w_visual)`` weights."""
    w_e = config.WEIGHT_ENCODER * (config.SLIP_ENCODER_PENALTY if slip else 1.0)
    w_s = config.WEIGHT_SCAN * (max(0.0, min(1.0, scan_score)) if scan_accepted else 0.0)
    w_v = config.WEIGHT_VISUAL * max(0.0, min(1.0, vo_confidence))
    return w_e, w_s, w_v


def fuse_poses(enc_pose, scan_pose, vo_pose, w_e, w_s, w_v):
    """Weighted blend of three poses. Falls back to the encoder pose if all zero."""
    total = w_e + w_s + w_v
    if total <= 1e-9:
        return tuple(enc_pose)
    we, ws, wv = w_e / total, w_s / total, w_v / total

    x = we * enc_pose[0] + ws * scan_pose[0] + wv * vo_pose[0]
    y = we * enc_pose[1] + ws * scan_pose[1] + wv * vo_pose[1]
    sin_t = we * math.sin(enc_pose[2]) + ws * math.sin(scan_pose[2]) + wv * math.sin(vo_pose[2])
    cos_t = we * math.cos(enc_pose[2]) + ws * math.cos(scan_pose[2]) + wv * math.cos(vo_pose[2])
    theta = math.atan2(sin_t, cos_t)
    return (x, y, theta)


def detect_slip(encoder_distance: float, vo_estimate: tuple, vo_confidence: float) -> bool:
    """Wheel slip: encoders report motion but a confident camera sees almost none."""
    if encoder_distance <= config.SLIP_THRESHOLD:
        return False
    if vo_confidence < 0.3:
        return False        # can't trust the camera to contradict the encoders
    visual_distance = math.hypot(vo_estimate[0], vo_estimate[1])
    return visual_distance < config.SLIP_THRESHOLD * 0.3
