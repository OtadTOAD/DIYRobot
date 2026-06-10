"""Encoder dead-reckoning odometry (F-04).

Differential-drive kinematics from wheel pulse counts. The LM393 encoders are
single-channel and cannot report direction, so the sign of each wheel's travel is
taken from the last motor command (see hardware/motors.get_last_command).

An :class:`Odometry` instance maintains the encoder-only pose estimate. The SLAM /
localization thread owns one instance, calls :meth:`update` once per cycle, and
feeds the result into the three-source fusion (F-11). On its own this is the "base
motion estimate" of the localization stack.
"""

import math

from robot_car import config
from robot_car.hardware import hal, motors


def _sign(value: float) -> float:
    if value > 1e-6:
        return 1.0
    if value < -1e-6:
        return -1.0
    return 1.0  # coasting with no command -> assume forward


class Odometry:
    def __init__(self, start_pose=(0.0, 0.0, 0.0)):
        self.x, self.y, self.theta = (float(v) for v in start_pose)
        self.last_distance = 0.0   # |centre displacement| last update (slip checks)

    def set_pose(self, pose) -> None:
        self.x, self.y, self.theta = (float(v) for v in pose)

    def get_pose(self) -> tuple:
        return (self.x, self.y, self.theta)

    def update(self) -> tuple:
        """Consume encoder pulses and advance the pose. Returns the new pose."""
        left_pulses, right_pulses = hal.get_backend().read_encoder_pulses()
        cmd_left, cmd_right = motors.get_last_command()

        delta_left = _sign(cmd_left) * left_pulses * config.DIST_PER_PULSE
        delta_right = _sign(cmd_right) * right_pulses * config.DIST_PER_PULSE

        delta_c = (delta_left + delta_right) / 2.0
        delta_theta = (delta_right - delta_left) / config.WHEEL_BASE

        # Integrate around the mid-heading for a better arc approximation.
        mid = self.theta + delta_theta / 2.0
        self.x += delta_c * math.cos(mid)
        self.y += delta_c * math.sin(mid)
        self.theta = _wrap(self.theta + delta_theta)
        self.last_distance = abs(delta_c)
        return (self.x, self.y, self.theta)


def _wrap(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi
